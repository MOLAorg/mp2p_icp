/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_LibPointmatcher.cpp
 * @brief  ICP wrapper on libpointmatcher
 * @author Jose Luis Blanco Claraco
 * @date   May 31, 2020
 */

#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

#include <fstream>
#include <sstream>

#if defined(MP2P_HAS_LIBPOINTMATCHER)
#include <pointmatcher/IO.h>
#include <pointmatcher/PointMatcher.h>
#endif

IMPLEMENTS_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

#if defined(MP2P_HAS_LIBPOINTMATCHER)
static PointMatcher<float>::DataPoints pointsToPM(
    const pointcloud_t& pc, const std::map<std::string, double>& layerWeights)
{
    // TODO: Convert pointclouds in a more efficient way (!)
    std::stringstream ss;

    for (const auto& layerNameWeight : layerWeights)
    {
        const std::string&                 name = layerNameWeight.first;
        const mrpt::maps::CPointsMap::Ptr& pts  = pc.point_layers.at(name);

        const auto xs = pts->getPointsBufferRef_x();
        const auto ys = pts->getPointsBufferRef_y();
        const auto zs = pts->getPointsBufferRef_z();
        for (size_t i = 0; i < xs.size(); i++)
            ss << xs[i] << " " << ys[i] << " " << zs[i] << "\n";
    }
    ss.seekg(0);
    return PointMatcherIO<float>::loadCSV(ss);
}
#endif

void ICP_LibPointmatcher::align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
#if defined(MP2P_HAS_LIBPOINTMATCHER)

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (const auto& kv1 : pcs1.point_layers)
    {
        // Ignore this layer?
        if (p.weight_pt2pt_layers.count(kv1.first) == 0) continue;

        pointcount1 += kv1.second->size();
        pointcount2 += pcs2.point_layers.at(kv1.first)->size();
    }
    ASSERT_(pointcount1 > 0 || !pcs1.planes.empty());
    ASSERT_(pointcount2 > 0 || !pcs2.planes.empty());

    MRPT_TODO("Obviously, fix this!! :-)");
    std::string icpImplConfigFile = "/home/jlblanco/libpm-icp.yaml";

    using PM = PointMatcher<float>;
    using DP = PM::DataPoints;

    // Load point clouds
    const DP ptsFrom = pointsToPM(pcs1, p.weight_pt2pt_layers);
    const DP ptsTo   = pointsToPM(pcs2, p.weight_pt2pt_layers);

    ASSERT_ABOVE_(ptsFrom.getNbPoints(), 0);
    ASSERT_ABOVE_(ptsTo.getNbPoints(), 0);

    // Create the default ICP algorithm
    PM::ICP icp;
    {
        // load YAML config
        std::ifstream ifs(icpImplConfigFile);
        if (!ifs.good())
        {
            THROW_EXCEPTION_FMT(
                "Cannot open config file %s", icpImplConfigFile.c_str());
        }
        icp.loadFromYaml(ifs);
    }

    int cloudDimension = ptsFrom.getEuclideanDim();
    ASSERT_EQUAL_(cloudDimension, 3U);
    ASSERT_EQUAL_(ptsFrom.getEuclideanDim(), ptsTo.getEuclideanDim());

    PM::TransformationParameters initTransfo =
        init_guess_m2_wrt_m1.getHomogeneousMatrix().cast_float().asEigen();

    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo))
    {
        MRPT_LOG_WARN(
            "Initial transformation is not rigid, identity will be used");
        initTransfo = PM::TransformationParameters::Identity(
            cloudDimension + 1, cloudDimension + 1);
    }

    const DP initializedData = rigidTrans->compute(ptsTo, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(initializedData, ptsFrom);

    MRPT_LOG_DEBUG_FMT(
        "match ratio: %.02f%%",
        icp.errorMinimizer->getWeightedPointUsedRatio() * 100.0);

    // Transform data to express it in ref
    DP data_out(initializedData);
    icp.transformations.apply(data_out, T);

    // Output in MP2P_ICP format:
    result.terminationReason = IterTermReason::Stalled;
    result.goodness          = icp.errorMinimizer->getWeightedPointUsedRatio();
    result.nIterations       = 10;  //!
    result.optimal_scale     = 1.0;
    result.quality           = 1.0;
    result.optimal_tf.mean =
        mrpt::poses::CPose3D(mrpt::math::CMatrixDouble44(T));
    result.optimal_tf.cov = icp.errorMinimizer->getCovariance();

#else
    THROW_EXCEPTION("This method requires MP2P built against libpointmatcher")
#endif
    MRPT_END
}
