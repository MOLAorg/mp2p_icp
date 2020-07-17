/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Base.cpp
 * @brief  Virtual interface for ICP algorithms. Useful for RTTI class searches.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/ICP_Base.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>  // TODO: remove
#include <mp2p_icp/Matcher_Points_InlierRatio.h>  // TODO: remove
#include <mp2p_icp/covariance.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(ICP_Base, mrpt::rtti::CObject, mp2p_icp);

MRPT_TODO("Refactor: ICP_base -> ICP. OLAE/Horn/... => `optimizer`");

using namespace mp2p_icp;

void ICP_Base::align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // Count of points:
    ASSERT_(pcs1.size() > 0);
    ASSERT_(pcs2.size() > 0);

    // ------------------------------------------------------
    // Main ICP loop
    // ------------------------------------------------------
    ICP_State state(pcs1, pcs2);

    state.current_solution = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    auto prev_solution     = state.current_solution;

    for (result.nIterations = 0; result.nIterations < p.maxIterations;
         result.nIterations++)
    {
        ICP_iteration_result iter_result;
        state.currentIteration = result.nIterations;

        // Call to algorithm-specific implementation of one ICP iteration:
        impl_ICP_iteration(state, p, iter_result);

        if (!iter_result.success)
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            break;
        }

        // Update to new solution:
        state.current_solution = iter_result.new_solution;

        // If matching has not changed, we are done:
        const auto deltaSol = state.current_solution - prev_solution;
        const mrpt::math::CVectorFixed<double, 6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.blockCopy<3, 1>(0, 0).norm();
        const double delta_rot = dSol.blockCopy<3, 1>(3, 0).norm();

#if 0
        std::cout << "Dxyz: " << std::abs(delta_xyz)
                  << " Drot:" << std::abs(delta_rot)
                  << " p: " << state.current_solution.asString() << "\n";
#endif

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        prev_solution = state.current_solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Quality:
    result.quality = evaluateQuality(
        qualityEvaluators_, pcs1, pcs2, state.current_solution,
        state.currentPairings);

    // Store output:
    result.optimal_tf.mean = state.current_solution;
    result.optimal_scale   = state.current_scale;
    result.finalPairings   = std::move(state.currentPairings);

    // Covariance:
    mp2p_icp::CovarianceParameters covParams;

    result.optimal_tf.cov = mp2p_icp::covariance(
        result.finalPairings, result.optimal_tf.mean, covParams);

    MRPT_END
}

Pairings ICP_Base::runMatchers(ICP_State& s)
{
    Pairings pairings;

    ASSERT_(!matchers_.empty());

    for (const auto& matcher : matchers_)
    {
        ASSERT_(matcher);

        MatchContext mc;
        Pairings     pc;
        mc.icpIteration = s.currentIteration;
        matcher->match(s.pc1, s.pc2, s.current_solution, mc, pc);

        pairings.push_back(pc);
    }

    return pairings;
}

void ICP_Base::initializeMatchers(const mrpt::containers::Parameters& params)
{
    matchers_.clear();

    ASSERT_(params.isSequence());
    for (const auto& entry : params.asSequence())
    {
        const auto& e = std::any_cast<mrpt::containers::Parameters>(entry);

        const auto sClass = e["class"].as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto m = std::dynamic_pointer_cast<Matcher>(o);
        ASSERTMSG_(m, "Matcher class seems not to be derived from Matcher");

        m->initialize(e["params"]);
        matchers_.push_back(m);
    }
}

void ICP_Base::initializeQualityEvaluators(
    const mrpt::containers::Parameters& params,
    ICP_Base::quality_eval_list_t&      lst)
{
    lst.clear();

    ASSERT_(params.isSequence());
    const auto numEntries = params.asSequence().size();

    for (const auto& entry : params.asSequence())
    {
        const auto& e = std::any_cast<mrpt::containers::Parameters>(entry);

        const auto sClass = e["class"].as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto m = std::dynamic_pointer_cast<QualityEvaluator>(o);
        ASSERTMSG_(m, "Class seems not to be derived from QualityEvaluator");
        m->initialize(e["params"]);

        double weight = 1.0;
        if (numEntries > 0) weight = e.getOrDefault<double>("weight", weight);
        lst.emplace_back(m, weight);
    }
}

void ICP_Base::initializeQualityEvaluators(
    const mrpt::containers::Parameters& params)
{
    initializeQualityEvaluators(params, qualityEvaluators_);
}

double ICP_Base::evaluateQuality(
    const quality_eval_list_t& evaluators, const pointcloud_t& pcGlobal,
    const pointcloud_t& pcLocal, const mrpt::poses::CPose3D& localPose,
    const Pairings& finalPairings)
{
    ASSERT_(!evaluators.empty());

    double sumW = .0, sumEvals = .0;
    for (const auto& e : evaluators)
    {
        const double w = e.relativeWeight;
        ASSERT_(w > 0);
        sumEvals +=
            w * e.obj->evaluate(pcGlobal, pcLocal, localPose, finalPairings);
        sumW += w;
    }
    ASSERT_(sumW > 0);

    return sumEvals / sumW;
}
