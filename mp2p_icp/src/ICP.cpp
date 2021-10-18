/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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

#include <regex>

IMPLEMENTS_MRPT_OBJECT(ICP, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void ICP::align(
    const metric_map_t& pcLocal, const metric_map_t& pcGlobal,
    const mrpt::math::TPose3D& initialGuessLocalWrtGlobal, const Parameters& p,
    Results& result, const mrpt::optional_ref<LogRecord>& outputDebugInfo)
{
    using namespace std::string_literals;

    MRPT_START

    // ----------------------------
    // Initial sanity checks
    // ----------------------------
    ASSERT_(!matchers_.empty());
    ASSERT_(!solvers_.empty());
    ASSERT_(!quality_evaluators_.empty());

    ASSERT_(!pcGlobal.empty());
    ASSERT_(!pcLocal.empty());

    // ----------------------------
    // Preparation
    // ----------------------------
    // Reset output:
    result = Results();

    // Prepare output debug records:
    std::optional<LogRecord> currentLog;

    const bool generateDebugRecord =
        outputDebugInfo.has_value() || p.generateDebugFiles;

    if (generateDebugRecord)
    {
        currentLog.emplace();
        currentLog->pcGlobal = pcGlobal.get_shared_from_this_or_clone();
        currentLog->pcLocal  = pcLocal.get_shared_from_this_or_clone();
        currentLog->initialGuessLocalWrtGlobal = initialGuessLocalWrtGlobal;
        currentLog->icpParameters              = p;
    }

    // ------------------------------------------------------
    // Main ICP loop
    // ------------------------------------------------------
    ICP_State state(pcGlobal, pcLocal);
    if (currentLog) state.log = &currentLog.value();

    state.currentSolution.optimalPose =
        mrpt::poses::CPose3D(initialGuessLocalWrtGlobal);
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
            matchers_, state.pcGlobal, state.pcLocal,
            state.currentSolution.optimalPose, mc);

        if (state.currentPairings.empty())
        {
            result.terminationReason = IterTermReason::NoPairings;
            break;
        }

        // Optimal relative pose:
        // ---------------------------------------
        SolverContext sc;
        sc.icpIteration = state.currentIteration;
        sc.guessRelativePose.emplace(state.currentSolution.optimalPose);

        // Compute the optimal pose:
        const bool solvedOk = run_solvers(
            solvers_, state.currentPairings, state.currentSolution, sc);

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

        if (p.debugPrintIterationProgress)
        {
            printf(
                "[ICP] Iter=%3u Delta_xyz=%9.02e, Delta_rot=%6.03f deg, "
                "(xyzypr)=%s pairs=%s\n",
                static_cast<unsigned int>(state.currentIteration),
                std::abs(delta_xyz), mrpt::RAD2DEG(std::abs(delta_rot)),
                state.currentSolution.optimalPose.asString().c_str(),
                state.currentPairings.contents_summary().c_str());
        }

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        prev_solution = state.currentSolution.optimalPose;
    }

    // ----------------------------
    // Fill in "result"
    // ----------------------------
    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Quality:
    result.quality = evaluate_quality(
        quality_evaluators_, pcGlobal, pcLocal,
        state.currentSolution.optimalPose, state.currentPairings);

    // Store output:
    result.optimal_tf.mean = state.currentSolution.optimalPose;
    result.optimalScale    = state.currentSolution.optimalScale;
    result.finalPairings   = std::move(state.currentPairings);

    // Covariance:
    mp2p_icp::CovarianceParameters covParams;

    result.optimal_tf.cov = mp2p_icp::covariance(
        result.finalPairings, result.optimal_tf.mean, covParams);

    // ----------------------------
    // Log records
    // ----------------------------
    // Store results into log struct:
    if (currentLog) currentLog->icpResult = result;

    // Save log to disk:
    if (currentLog.has_value()) save_log_file(*currentLog, p);

    // return log info:
    if (currentLog && outputDebugInfo.has_value())
        outputDebugInfo.value().get() = std::move(currentLog.value());

    MRPT_END
}

void ICP::save_log_file(const LogRecord& log, const Parameters& p)
{
    using namespace std::string_literals;

    if (!p.generateDebugFiles) return;

    // global log file record counter:
    static unsigned int logFileCounter = 0;
    static std::mutex   counterMtx;
    unsigned int        RECORD_UNIQUE_ID;
    {
        counterMtx.lock();
        RECORD_UNIQUE_ID = logFileCounter++;
        counterMtx.unlock();
    }

    std::string filename = p.debugFileNameFormat;

    {
        const std::string expr  = "\\$UNIQUE_ID";
        const auto        value = mrpt::format("%05u", RECORD_UNIQUE_ID);
        filename = std::regex_replace(filename, std::regex(expr), value);
    }

    {
        const std::string expr  = "\\$GLOBAL_ID";
        const auto        value = mrpt::format(
            "%05u", static_cast<unsigned int>(
                        (log.pcGlobal && log.pcGlobal->id.has_value())
                            ? log.pcGlobal->id.value()
                            : 0));
        filename = std::regex_replace(filename, std::regex(expr), value);
    }

    {
        const std::string expr = "\\$GLOBAL_LABEL";
        const auto value = (log.pcGlobal && log.pcGlobal->label.has_value())
                               ? log.pcGlobal->label.value()
                               : ""s;
        filename = std::regex_replace(filename, std::regex(expr), value);
    }
    {
        const std::string expr  = "\\$LOCAL_ID";
        const auto        value = mrpt::format(
            "%05u", static_cast<unsigned int>(
                        (log.pcLocal && log.pcLocal->id.has_value())
                            ? log.pcLocal->id.value()
                            : 0));
        filename = std::regex_replace(filename, std::regex(expr), value);
    }

    {
        const std::string expr = "\\$LOCAL_LABEL";
        const auto value       = (log.pcLocal && log.pcLocal->label.has_value())
                               ? log.pcLocal->label.value()
                               : ""s;
        filename = std::regex_replace(filename, std::regex(expr), value);
    }

    const bool saveOk = log.save_to_file(filename);
    if (!saveOk)
    {
        std::cerr << "[ERROR] Could not save icp log file to '" << filename
                  << "'" << std::endl;
    }
}

bool ICP::run_solvers(
    const solver_list_t& solvers, const Pairings& pairings,
    OptimalTF_Result& out, const SolverContext& sc)
{
    for (const auto& solver : solvers)
    {
        ASSERT_(solver);
        if (solver->optimal_pose(pairings, out, sc)) return true;
    }
    return false;
}

void ICP::initialize_solvers(const mrpt::containers::yaml& params)
{
    initialize_solvers(params, solvers_);
}

void ICP::initialize_solvers(
    const mrpt::containers::yaml& params, ICP::solver_list_t& lst)
{
    lst.clear();

    ASSERT_(params.isSequence());
    for (const auto& entry : params.asSequence())
    {
        const auto& e = entry.asMap();
        // disabled?
        if (e.count("enabled") && e.at("enabled").as<bool>() == false) continue;

        const auto sClass = e.at("class").as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto m = std::dynamic_pointer_cast<Solver>(o);
        ASSERTMSG_(
            m, mrpt::format(
                   "`%s` class seems not to be derived from Solver",
                   sClass.c_str()));

        m->initialize(e.at("params"));
        lst.push_back(m);
    }
}

void ICP::initialize_matchers(const mrpt::containers::yaml& params)
{
    initialize_matchers(params, matchers_);
}

void ICP::initialize_matchers(
    const mrpt::containers::yaml& params, matcher_list_t& lst)
{
    lst.clear();

    ASSERT_(params.isSequence());
    for (const auto& entry : params.asSequence())
    {
        const auto& e = entry.asMap();
        // disabled?
        if (e.count("enabled") && e.at("enabled").as<bool>() == false) continue;

        const auto sClass = e.at("class").as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto m = std::dynamic_pointer_cast<Matcher>(o);
        ASSERTMSG_(
            m, mrpt::format(
                   "`%s` class seems not to be derived from Matcher",
                   sClass.c_str()));

        m->initialize(e.at("params"));
        lst.push_back(m);
    }
}

void ICP::initialize_quality_evaluators(
    const mrpt::containers::yaml& params, ICP::quality_eval_list_t& lst)
{
    lst.clear();

    ASSERT_(params.isSequence());
    const auto numEntries = params.asSequence().size();

    for (const auto& entry : params.asSequence())
    {
        const auto& e = entry.asMap();
        // disabled?
        if (e.count("enabled") && e.at("enabled").as<bool>() == false) continue;

        const auto sClass = e.at("class").as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto m = std::dynamic_pointer_cast<QualityEvaluator>(o);
        ASSERTMSG_(
            m, mrpt::format(
                   "`%s` class seems not to be derived from QualityEvaluator",
                   sClass.c_str()));

        m->initialize(e.at("params"));

        double weight = 1.0;
        if (numEntries > 0 && e.count("weight") > 0)
            weight = e.at("weight").as<double>();
        lst.emplace_back(m, weight);
    }
}

void ICP::initialize_quality_evaluators(const mrpt::containers::yaml& params)
{
    initialize_quality_evaluators(params, quality_evaluators_);
}

double ICP::evaluate_quality(
    const quality_eval_list_t& evaluators, const metric_map_t& pcGlobal,
    const metric_map_t& pcLocal, const mrpt::poses::CPose3D& localPose,
    const Pairings& finalPairings)
{
    ASSERT_(!evaluators.empty());

    double sumW = .0, sumEvals = .0;
    for (const auto& e : evaluators)
    {
        const double w = e.relativeWeight;
        ASSERT_GT_(w, 0);
        const double eval =
            e.obj->evaluate(pcGlobal, pcLocal, localPose, finalPairings);
        sumEvals += w * eval;
        sumW += w;
    }
    ASSERT_(sumW > 0);

    return sumEvals / sumW;
}
