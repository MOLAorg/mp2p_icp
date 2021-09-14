/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
static PointMatcher<double>::DataPoints pointsToPM(const pointcloud_t& pc)
{
    // TODO: Convert pointclouds in a more efficient way (!)
    std::stringstream ss;

    for (const auto& ly : pc.point_layers)
    {
        // const std::string&                 name = ly.first;
        const mrpt::maps::CPointsMap::Ptr& pts = ly.second;
        ASSERT_(pts);

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

void ICP_LibPointmatcher::align(
    [[maybe_unused]] const pointcloud_t&        pcs1,
    [[maybe_unused]] const pointcloud_t&        pcs2,
    [[maybe_unused]] const mrpt::math::TPose3D& initialGuessLocalWrtGlobal,
    [[maybe_unused]] const Parameters& p, [[maybe_unused]] Results& result)
{
    using namespace std::string_literals;

    MRPT_START
#if defined(MP2P_HAS_LIBPOINTMATCHER)

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    ICP_State state(pcs1, pcs2);

    state.currentSolution = OptimalTF_Result();
    state.currentSolution.optimalPose =
        mrpt::poses::CPose3D(initialGuessLocalWrtGlobal);
    auto prev_solution = state.currentSolution.optimalPose;

    // Reset output:
    result = Results();

    // Count of points:
    ASSERT_(pcs1.size() > 0);
    ASSERT_(pcs2.size() > 0);

    const char* icpConfigFmt = R"XXX(
readingDataPointsFilters:
  - RandomSamplingDataPointsFilter:
      prob: %f

referenceDataPointsFilters:
  - SurfaceNormalDataPointsFilter:
      knn: %u

matcher:
  KDTreeMatcher:
    knn: %u

outlierFilters:
  - %s:
%s

errorMinimizer:
  %s

transformationCheckers:
  - CounterTransformationChecker:
      maxIterationCount: %u
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.0001
      minDiffTransErr: 0.001
      smoothLength: 4

inspector:
  NullInspector

logger:
  NullLogger
)XXX";

    using PM = PointMatcher<double>;
    using DP = PM::DataPoints;

    // Load point clouds
    const DP ptsFrom = pointsToPM(pcs1);
    const DP ptsTo   = pointsToPM(pcs2);

    ASSERT_GT_(ptsFrom.getNbPoints(), 0);
    ASSERT_GT_(ptsTo.getNbPoints(), 0);

    // Create the default ICP algorithm
    PM::ICP icp;

    {
        const auto& plm = parametersLibpointmatcher;

        std::string outlierParams;
        for (const auto& op : plm.outlierParams)
        {
            outlierParams += "      ";
            outlierParams += op.first;
            outlierParams += ": ";
            outlierParams += std::to_string(op.second);
            outlierParams += "\n";
        }

        // load YAML config
        std::stringstream ss;
        ss << mrpt::format(
            icpConfigFmt, plm.RandomSamplingDataPointsFilter_prob,
            plm.SurfaceNormalDataPointsFilter_knn, plm.KDTreeMatcher_knn,
            plm.outlierFilter.c_str(), outlierParams.c_str(),
            plm.errorMinimizer.c_str(), p.maxIterations);

        ss.seekg(0);
        icp.loadFromYaml(ss);
    }

    int cloudDimension = ptsFrom.getEuclideanDim();
    ASSERT_EQUAL_(cloudDimension, 3U);
    ASSERT_EQUAL_(ptsFrom.getEuclideanDim(), ptsTo.getEuclideanDim());

    PM::TransformationParameters initTransfo =
        initialGuessLocalWrtGlobal.getHomogeneousMatrix().asEigen();

    TransformationsImpl<double>::RigidTransformation rigidTrans;

    if (!rigidTrans.checkParameters(initTransfo))
    {
        MRPT_LOG_WARN(
            "Initial transformation is not rigid, identity will be used");
        initTransfo = PM::TransformationParameters::Identity(
            cloudDimension + 1, cloudDimension + 1);
    }

    const DP initializedData = rigidTrans.compute(ptsTo, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T;
    try
    {
        T = icp(initializedData, ptsFrom);

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

    // Generate some pairings for the quality evaluation:
    mp2p_icp::Matcher_Points_DistanceThreshold pm(0.1);
    pm.match(
        pcs1, pcs2, state.currentSolution.optimalPose, {},
        result.finalPairings);

    // Quality:
    result.quality = evaluate_quality(
        quality_evaluators_, pcs1, pcs2, state.currentSolution.optimalPose,
        result.finalPairings);

    result.terminationReason = IterTermReason::Stalled;
    result.optimalScale      = 1.0;
    result.optimal_tf.mean   = state.currentSolution.optimalPose;

    mp2p_icp::CovarianceParameters covParams;

    result.optimal_tf.cov = mp2p_icp::covariance(
        result.finalPairings, result.optimal_tf.mean, covParams);
#else
    THROW_EXCEPTION("This method requires MP2P built against libpointmatcher");
#endif
    MRPT_END
}
