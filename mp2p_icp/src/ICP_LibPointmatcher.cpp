/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_LibPointmatcher.cpp
 * @brief  ICP wrapper on libpointmatcher
 * @author Jose Luis Blanco Claraco
 * @date   May 31, 2020
 */

#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/covariance.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

#include <fstream>
#include <sstream>

#if defined(MP2P_HAS_LIBPOINTMATCHER)
#include <pointmatcher/IO.h>
#include <pointmatcher/LoggerImpl.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/TransformationsImpl.h>
#endif

IMPLEMENTS_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp::ICP, mp2p_icp)

using namespace mp2p_icp;

bool ICP_LibPointmatcher::methodAvailable()
{
#if defined(MP2P_HAS_LIBPOINTMATCHER)
    return true;
#else
    return false;
#endif
}

#if defined(MP2P_HAS_LIBPOINTMATCHER)
static PointMatcher<double>::DataPoints pointsToPM(const metric_map_t& pc)
{
    // TODO: Convert pointclouds in a more efficient way (!)
    std::stringstream ss;

    for (const auto& ly : pc.layers)
    {
        // const std::string&                 name = ly.first;
        auto pts = mp2p_icp::MapToPointsMap(*ly.second);
        if (!pts) continue;  // Not a point cloud layer

        const auto xs = pts->getPointsBufferRef_x();
        const auto ys = pts->getPointsBufferRef_y();
        const auto zs = pts->getPointsBufferRef_z();
        for (size_t i = 0; i < xs.size(); i++)
            ss << mrpt::format("%f %f %f\n", xs[i], ys[i], zs[i]);
    }
    ss.seekg(0);
    return PointMatcherIO<double>::loadCSV(ss);
}
#endif

void ICP_LibPointmatcher::initialize_derived(
    const mrpt::containers::yaml& params)
{
    std::stringstream ss;
    params.printAsYAML(ss);
    pm_icp_yaml_settings_ = ss.str();
}

void ICP_LibPointmatcher::align(
    [[maybe_unused]] const metric_map_t&        pcLocal,
    [[maybe_unused]] const metric_map_t&        pcGlobal,
    [[maybe_unused]] const mrpt::math::TPose3D& initialGuessLocalWrtGlobal,
    [[maybe_unused]] const Parameters& p, [[maybe_unused]] Results& result,
    [[maybe_unused]] const std::optional<mrpt::poses::CPose3DPDFGaussianInf>&
                                                          prior,
    [[maybe_unused]] const mrpt::optional_ref<LogRecord>& outputDebugInfo)
{
    using namespace std::string_literals;

    MRPT_START
#if defined(MP2P_HAS_LIBPOINTMATCHER)

    ASSERT_EQUAL_(pcLocal.layers.size(), pcGlobal.layers.size());
    ASSERT_(!pcLocal.empty() && !pcGlobal.empty());

    ASSERTMSG_(
        !pm_icp_yaml_settings_.empty(),
        "You must call initialize_derived() first, or initialize from a YAML "
        "file with a `derived` section with the LibPointMathcer-specific "
        "configuration.");

    ICP_State state(pcLocal, pcGlobal);

    state.currentSolution = OptimalTF_Result();
    state.currentSolution.optimalPose =
        mrpt::poses::CPose3D(initialGuessLocalWrtGlobal);
    auto prev_solution = state.currentSolution.optimalPose;

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

    // Count of points:
    ASSERT_(pcLocal.size() > 0);
    ASSERT_(pcGlobal.size() > 0);

    using PM = PointMatcher<double>;
    using DP = PM::DataPoints;

    // Load point clouds
    const DP ptsLocal  = pointsToPM(pcLocal);
    const DP ptsGlobal = pointsToPM(pcGlobal);

    ASSERT_GT_(ptsLocal.getNbPoints(), 0);
    ASSERT_GT_(ptsGlobal.getNbPoints(), 0);

    // Create the default ICP algorithm
    PM::ICP icp;

    {
        // load YAML config
        std::stringstream ss;
        ss << pm_icp_yaml_settings_;
        ss.seekg(0);
        icp.loadFromYaml(ss);
    }

    int cloudDimension = ptsLocal.getEuclideanDim();
    ASSERT_EQUAL_(cloudDimension, 3U);
    ASSERT_EQUAL_(ptsLocal.getEuclideanDim(), ptsGlobal.getEuclideanDim());

    PM::TransformationParameters initTransfo =
        initialGuessLocalWrtGlobal.getHomogeneousMatrix().asEigen();

    TransformationsImpl<double>::RigidTransformation rigidTrans;

    if (!rigidTrans.checkParameters(initTransfo))
    {
        MRPT_LOG_WARN(
            "Initial transformation is not rigid, SE(3) identity will be used");
        initTransfo = PM::TransformationParameters::Identity(
            cloudDimension + 1, cloudDimension + 1);
    }

    const DP ptsLocalTf = rigidTrans.compute(ptsLocal, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T;
    try
    {
        T = icp(ptsLocalTf, ptsGlobal);

        // PM gives us the transformation wrt the initial transformation,
        // since we already applied that transf. to the input point cloud!
        state.currentSolution.optimalPose =
            mrpt::poses::CPose3D(initialGuessLocalWrtGlobal) +
            mrpt::poses::CPose3D(mrpt::math::CMatrixDouble44(T));

        // result.quality = icp.errorMinimizer->getWeightedPointUsedRatio();
    }
    catch (const PM::ConvergenceError&)
    {
        // No good pairing candidates.
        result.quality = 0;
    }

    // Output in MP2P_ICP format:
    if (!icp.transformationCheckers.empty())
        result.nIterations =
            icp.transformationCheckers.at(0)->getConditionVariables()[0];
    else
        result.nIterations = 1;

    // Quality:
    result.quality = evaluate_quality(
        quality_evaluators_, pcGlobal, pcLocal,
        state.currentSolution.optimalPose, result.finalPairings);

    result.terminationReason = IterTermReason::Stalled;
    result.optimalScale      = 1.0;
    result.optimal_tf.mean   = state.currentSolution.optimalPose;

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

#else
    THROW_EXCEPTION("This method requires MP2P built against libpointmatcher");
#endif
    MRPT_END
}
