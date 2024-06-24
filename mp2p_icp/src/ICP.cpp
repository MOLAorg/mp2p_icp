/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mrpt/core/lock_helper.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/se3.h>

#include <regex>

IMPLEMENTS_MRPT_OBJECT(ICP, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void ICP::align(
    const metric_map_t& pcLocal, const metric_map_t& pcGlobal,
    const mrpt::math::TPose3D& initialGuessLocalWrtGlobal, const Parameters& p,
    Results&                                                 result,
    const std::optional<mrpt::poses::CPose3DPDFGaussianInf>& prior,
    const mrpt::optional_ref<LogRecord>&                     outputDebugInfo)
{
    using namespace std::string_literals;

    MRPT_START

    mrpt::system::CTimeLoggerEntry tle(profiler_, "align");

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
    mrpt::system::CTimeLoggerEntry tle1(profiler_, "align.1_prepare");
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

    tle1.stop();

    // ------------------------------------------------------
    // Add our own parameters to the user's one, or just
    // define a new one if none is provided.
    // ------------------------------------------------------
    std::set<ParameterSource*> activeParamSouces;

    auto lambdaAddOwnParams = [&](Parameterizable& obj)
    {
        ParameterSource* ps = obj.attachedSource();
        if (!ps)
        {
            obj.attachToParameterSource(ownParamSource_);
            ps = &ownParamSource_;
        }
        ps->updateVariable("ICP_ITERATION", result.nIterations);
        activeParamSouces.insert(ps);
    };
    auto lambdaRealizeParamSources = [&]()
    {
        for (auto& ps : activeParamSouces) ps->realize();
    };

    // ------------------------------------------------------
    // Main ICP loop
    // ------------------------------------------------------
    mrpt::system::CTimeLoggerEntry tle2(profiler_, "align.2_create_state");

    ICP_State state(pcGlobal, pcLocal);
    if (currentLog) state.log = &currentLog.value();

    tle2.stop();

    const auto initGuess = mrpt::poses::CPose3D(initialGuessLocalWrtGlobal);

    state.currentSolution.optimalPose = initGuess;

    mrpt::poses::CPose3D prev_solution = state.currentSolution.optimalPose;
    std::optional<mrpt::poses::CPose3D> prev2_solution;  // 2 steps ago
    std::optional<mrpt::poses::CPose3D> lastCorrection;
    SolverContext                       sc;
    sc.prior = prior;

    for (result.nIterations = 0; result.nIterations < p.maxIterations;
         result.nIterations++)
    {
        mrpt::system::CTimeLoggerEntry tle3(profiler_, "align.3_iter");

        // Update iteration count, both in direct C++ structure...
        state.currentIteration = result.nIterations;

        // ...and via programmable formulas:
        for (auto& obj : matchers_) lambdaAddOwnParams(*obj);
        for (auto& obj : solvers_) lambdaAddOwnParams(*obj);
        for (auto& [obj, _] : quality_evaluators_) lambdaAddOwnParams(*obj);
        lambdaRealizeParamSources();

        // Matchings
        // ---------------------------------------
        MatchContext mc;
        mc.icpIteration = state.currentIteration;

        mrpt::system::CTimeLoggerEntry tle4(profiler_, "align.3.1_matchers");

        state.currentPairings = run_matchers(
            matchers_, state.pcGlobal, state.pcLocal,
            state.currentSolution.optimalPose, mc);

        tle4.stop();

        if (state.currentPairings.empty())
        {
            result.terminationReason = IterTermReason::NoPairings;
            if (p.debugPrintIterationProgress)
            {
                printf(
                    "[ICP] Iter=%3u No pairings!\n",
                    static_cast<unsigned int>(state.currentIteration));
            }
            break;
        }

        // Optimal relative pose:
        // ---------------------------------------
        mrpt::system::CTimeLoggerEntry tle5(profiler_, "align.3.2_solvers");

        sc.icpIteration = state.currentIteration;
        sc.guessRelativePose.emplace(state.currentSolution.optimalPose);
        sc.currentCorrectionFromInitialGuess =
            state.currentSolution.optimalPose - initGuess;
        sc.lastIcpStepIncrement = lastCorrection;

        // Compute the optimal pose:
        const bool solvedOk = run_solvers(
            solvers_, state.currentPairings, state.currentSolution, sc);

        tle5.stop();

        if (!solvedOk)
        {
            result.terminationReason = IterTermReason::SolverError;
            if (p.debugPrintIterationProgress)
            {
                printf(
                    "[ICP] Iter=%3u Solver returned false\n",
                    static_cast<unsigned int>(state.currentIteration));
            }
            break;
        }

        // Updated solution is already in "state.currentSolution".
        mrpt::system::CTimeLoggerEntry tle6(
            profiler_, "align.3.3_end_criterions");

        // Termination criterion: small delta:
        auto lambdaCalcIncrs = [](const mrpt::poses::CPose3D& deltaSol)
            -> std::tuple<double, double>
        {
            const mrpt::math::CVectorFixed<double, 6> dSol =
                mrpt::poses::Lie::SE<3>::log(deltaSol);
            const double delta_xyz = dSol.blockCopy<3, 1>(0, 0).norm();
            const double delta_rot = dSol.blockCopy<3, 1>(3, 0).norm();
            return {delta_xyz, delta_rot};
        };

        // Keep the minimum step between the current increment, and the
        // increment from current solution to two timesteps ago. This is to
        // detect bistable, oscillating solutions.
        const auto deltaSol = state.currentSolution.optimalPose - prev_solution;
        lastCorrection      = deltaSol;  // save for the next solver context

        auto [delta_xyz, delta_rot] = lambdaCalcIncrs(deltaSol);

        if (prev2_solution.has_value())
        {
            auto [delta_xyz2, delta_rot2] = lambdaCalcIncrs(
                state.currentSolution.optimalPose - *prev2_solution);

            mrpt::keep_min(delta_xyz, delta_xyz2);
            mrpt::keep_min(delta_rot, delta_rot2);
        }

        if (p.debugPrintIterationProgress)
        {
            printf(
                "[ICP] Iter=%3u Δt=%9.02e, ΔR=%6.03f deg, "
                "(xyzypr)=%s pairs=%s\n",
                static_cast<unsigned int>(state.currentIteration),
                std::abs(delta_xyz), mrpt::RAD2DEG(std::abs(delta_rot)),
                state.currentSolution.optimalPose.asString().c_str(),
                state.currentPairings.contents_summary().c_str());
        }

        const bool stalled =
            (std::abs(delta_xyz) < p.minAbsStep_trans &&
             std::abs(delta_rot) < p.minAbsStep_rot);

        // store partial solutions for logging/debuging?
        if (p.saveIterationDetails &&
            (p.decimationIterationDetails == 0 ||
             state.currentIteration % p.decimationIterationDetails == 0 ||
             stalled))
        {
            if (!currentLog->iterationsDetails.has_value())
                currentLog->iterationsDetails.emplace();

            auto& id =
                currentLog->iterationsDetails.value()[state.currentIteration];
            id.optimalPose = state.currentSolution.optimalPose;
            id.pairings    = state.currentPairings;
        }

        // End criteria?
        if (stalled)
        {
            result.terminationReason = IterTermReason::Stalled;

            if (p.debugPrintIterationProgress)
            {
                printf(
                    "[ICP] Iter=%3u Solver stalled.\n",
                    static_cast<unsigned int>(state.currentIteration));
            }

            break;
        }

        // Quality checkpoints to abort ICP iterations as useless?
        if (auto itQ = p.quality_checkpoints.find(state.currentIteration);
            itQ != p.quality_checkpoints.end())
        {
            const double minQuality = itQ->second;

            for (auto& e : quality_evaluators_) lambdaAddOwnParams(*e.obj);
            lambdaRealizeParamSources();

            const double quality = evaluate_quality(
                quality_evaluators_, pcGlobal, pcLocal,
                state.currentSolution.optimalPose, state.currentPairings);

            if (quality < minQuality)
            {
                result.terminationReason =
                    IterTermReason::QualityCheckpointFailed;
                if (p.debugPrintIterationProgress)
                {
                    printf(
                        "[ICP] Iter=%3u quality checkpoint did not pass: %f < "
                        "%f\n",
                        static_cast<unsigned int>(state.currentIteration),
                        quality, minQuality);
                }
                break;  // abort ICP
            }
        }

        // Process user hooks:
        if (iteration_hook_)
        {
            IterationHook_Input hi;
            hi.currentIteration = state.currentIteration;
            hi.currentPairings  = &state.currentPairings;
            hi.currentSolution  = &state.currentSolution;
            hi.pcGlobal         = &state.pcGlobal;
            hi.pcLocal          = &state.pcLocal;

            const auto ho = iteration_hook_(hi);

            if (ho.request_stop)
            {
                // abort ICP
                result.terminationReason = IterTermReason::HookRequest;
                break;
            }
        }

        // roll values back:
        prev2_solution = prev_solution;
        prev_solution  = state.currentSolution.optimalPose;
    }

    // ----------------------------
    // Fill in "result"
    // ----------------------------
    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Quality:
    mrpt::system::CTimeLoggerEntry tle7(profiler_, "align.4_quality");

    for (auto& e : quality_evaluators_) lambdaAddOwnParams(*e.obj);
    lambdaRealizeParamSources();

    result.quality = evaluate_quality(
        quality_evaluators_, pcGlobal, pcLocal,
        state.currentSolution.optimalPose, state.currentPairings);

    tle7.stop();

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
    mrpt::system::CTimeLoggerEntry tle8(profiler_, "align.5_save_log");

    if (currentLog)
    {
        // Store results into log struct:
        currentLog->icpResult = result;

        // Store dynamic variables:
        if (!matchers().empty())
        {
            currentLog->dynamicVariables = matchers()
                                               .begin()
                                               ->get()
                                               ->attachedSource()
                                               ->getVariableValues();
        }

        currentLog->icpResult = result;

        // Save log to disk (if enabled), applying filters beforehand:
        if (p.functor_before_logging_local)
        {
            auto pc = mp2p_icp::metric_map_t::Create();
            *pc     = *currentLog->pcLocal;
            p.functor_before_logging_local(*pc);
            currentLog->pcLocal = pc;
        }
        if (p.functor_before_logging_global)
        {
            auto pc = mp2p_icp::metric_map_t::Create();
            *pc     = *currentLog->pcGlobal;
            p.functor_before_logging_global(*pc);
            currentLog->pcGlobal = pc;
        }

        save_log_file(*currentLog, p);

        // return log info:
        if (outputDebugInfo.has_value())
            outputDebugInfo.value().get() = std::move(currentLog.value());
    }

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
        auto lck = mrpt::lockHelper(counterMtx);

        RECORD_UNIQUE_ID = logFileCounter++;

        if (p.decimationDebugFiles > 1 &&
            (RECORD_UNIQUE_ID % p.decimationDebugFiles) != 0)
            return;  // skip due to decimation
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

    // make sure directory exist:
    const auto baseDir = mrpt::system::extractFileDirectory(filename);
    if (!mrpt::system::directoryExists(baseDir))
    {
        const bool ok = mrpt::system::createDirectory(baseDir);
        if (!ok)
        {
            std::cerr << "[ICP::save_log_file] Could not create directory to "
                         "save icp log file: '"
                      << baseDir << "'" << std::endl;
        }
        else
        {
            std::cerr
                << "[ICP::save_log_file] Created output directory for logs: '"
                << baseDir << "'" << std::endl;
        }
    }

    // Save it:
    const bool saveOk = log.save_to_file(filename);
    if (!saveOk)
    {
        std::cerr << "[ICP::save_log_file] Could not save icp log file to '"
                  << filename << "'" << std::endl;
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
        const auto evalResult =
            e.obj->evaluate(pcGlobal, pcLocal, localPose, finalPairings);

        if (evalResult.hard_discard) return 0;  // hard limit

        sumEvals += w * evalResult.quality;
        sumW += w;
    }
    ASSERT_(sumW > 0);

    return sumEvals / sumW;
}
