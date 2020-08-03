/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP.h
 * @brief  Generic ICP algorithm container.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/covariance.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

IMPLEMENTS_MRPT_OBJECT(ICP, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void ICP::align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& initialGuessM2wrtM1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START

    ASSERT_(!matchers_.empty());
    ASSERT_(!solvers_.empty());
    ASSERT_(!quality_evaluators_.empty());

    ASSERT_(!pcs1.empty());
    ASSERT_(!pcs2.empty());

    // Reset output:
    result = Results();

    // ------------------------------------------------------
    // Main ICP loop
    // ------------------------------------------------------
    ICP_State state(pcs1, pcs2);

    state.currentSolution.optimalPose =
        mrpt::poses::CPose3D(initialGuessM2wrtM1);
    auto prev_solution = state.currentSolution.optimalPose;

    for (result.nIterations = 0; result.nIterations < p.maxIterations;
         result.nIterations++)
    {
        state.currentIteration = result.nIterations;

        // Matchings
        // ---------------------------------------
        MatchContext mc;
        mc.icpIteration = state.currentIteration;

        state.currentPairings = run_matchers(
            matchers_, state.pc1, state.pc2, state.currentSolution.optimalPose,
            mc);

        if (state.currentPairings.empty())
        {
            result.terminationReason = IterTermReason::NoPairings;
            break;
        }

        // Optimal relative pose:
        // ---------------------------------------
        SolverContext sc;
        sc.icpIteration                   = state.currentIteration;
        sc.guessRelativePose.value()      = state.currentSolution.optimalPose;
        sc.maxInnerLoopIterations.value() = p.maxInnerLoopIterations;

        // Compute the optimal pose:
        const bool solvedOk = run_solvers(
            solvers_, state.currentPairings, state.currentSolution,
            p.pairingsWeightParameters, sc);

        if (!solvedOk)
        {
            result.terminationReason = IterTermReason::SolverError;
            break;
        }

        // Updated solution is already in "state.currentSolution".

        // Termination criterion: small delta:
        const auto deltaSol = state.currentSolution.optimalPose - prev_solution;
        const mrpt::math::CVectorFixed<double, 6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.blockCopy<3, 1>(0, 0).norm();
        const double delta_rot = dSol.blockCopy<3, 1>(3, 0).norm();

#if 0
        std::cout << "Dxyz: " << std::abs(delta_xyz)
                  << " Drot:" << std::abs(delta_rot)
                  << " p: " << state.currentSolution.asString() << "\n";
#endif

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        prev_solution = state.currentSolution.optimalPose;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Quality:
    result.quality = evaluate_quality(
        quality_evaluators_, pcs1, pcs2, state.currentSolution.optimalPose,
        state.currentPairings);

    // Store output:
    result.optimal_tf.mean = state.currentSolution.optimalPose;
    result.optimalScale    = state.currentSolution.optimalScale;
    result.finalPairings   = std::move(state.currentPairings);

    // Covariance:
    mp2p_icp::CovarianceParameters covParams;

    result.optimal_tf.cov = mp2p_icp::covariance(
        result.finalPairings, result.optimal_tf.mean, covParams);

    MRPT_END
}

Pairings ICP::run_matchers(
    const matcher_list_t& matchers, const pointcloud_t& pc1,
    const pointcloud_t& pc2, const mrpt::poses::CPose3D& pc2_wrt_pc1,
    const MatchContext& mc)
{
    Pairings pairings;
    for (const auto& matcher : matchers)
    {
        ASSERT_(matcher);
        Pairings pc;
        matcher->match(pc1, pc2, pc2_wrt_pc1, mc, pc);
        pairings.push_back(pc);
    }
    return pairings;
}

bool ICP::run_solvers(
    const solver_list_t& solvers, const Pairings& pairings,
    OptimalTF_Result& out, const WeightParameters& wp, const SolverContext& sc)
{
    for (const auto& solver : solvers)
    {
        ASSERT_(solver);
        if (solver->optimal_pose(pairings, out, wp, sc)) return true;
    }
    return false;
}

void ICP::initialize_matchers(const mrpt::containers::Parameters& params)
{
    initialize_matchers(params, matchers_);
}

void ICP::initialize_matchers(
    const mrpt::containers::Parameters& params, ICP::matcher_list_t& lst)
{
    lst.clear();

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
        lst.push_back(m);
    }
}

void ICP::initialize_quality_evaluators(
    const mrpt::containers::Parameters& params, ICP::quality_eval_list_t& lst)
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

void ICP::initialize_quality_evaluators(
    const mrpt::containers::Parameters& params)
{
    initialize_quality_evaluators(params, quality_evaluators_);
}

double ICP::evaluate_quality(
    const quality_eval_list_t& evaluators, const pointcloud_t& pcGlobal,
    const pointcloud_t& pcLocal, const mrpt::poses::CPose3D& localPose,
    const Pairings& finalPairings)
{
    ASSERT_(!evaluators.empty());

    double sumW = .0, sumEvals = .0;
    for (const auto& e : evaluators)
    {
        const double w = e.relativeWeight;
        ASSERT_ABOVE_(w, 0);
        const double eval =
            e.obj->evaluate(pcGlobal, pcLocal, localPose, finalPairings);
        sumEvals += w * eval;
        sumW += w;
    }
    ASSERT_(sumW > 0);

    return sumEvals / sumW;
}
