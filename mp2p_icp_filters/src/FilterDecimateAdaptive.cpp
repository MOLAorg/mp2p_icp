/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateAdaptive.cpp
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#include <mp2p_icp/estimate_points_eigen.h>
#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mrpt/bayes/CParticleFilterCapable.h>  // computeResampling()
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/random/random_shuffle.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterDecimateAdaptive, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateAdaptive::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_REQ(c, desired_output_point_count);

    MCP_LOAD_OPT(c, assumed_minimum_pointcloud_bbox);
    MCP_LOAD_OPT(c, maximum_voxel_count_per_dimension);
    MCP_LOAD_OPT(c, minimum_input_points_per_voxel);
}

FilterDecimateAdaptive::FilterDecimateAdaptive()
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateAdaptive");
}

void FilterDecimateAdaptive::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    MRPT_END
}

void FilterDecimateAdaptive::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPc = nullptr;
    if (auto itLy = inOut.layers.find(params_.output_pointcloud_layer);
        itLy != inOut.layers.end())
    {
        outPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!outPc)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.",
                params_.output_pointcloud_layer.c_str());
    }
    else
    {
        auto newMap = mrpt::maps::CSimplePointsMap::Create();
        inOut.layers[params_.output_pointcloud_layer] = newMap;

        outPc = newMap.get();
    }

    const auto& _ = params_;  // shortcut

    outPc->reserve(outPc->size() + _.desired_output_point_count);

    // Estimate voxel size dynamically from the input cloud:
    filter_grid_.clear();

    auto inputBbox = pc.boundingBox();
    auto bboxSize  = mrpt::math::TVector3Df(inputBbox.max - inputBbox.min);
    mrpt::keep_max(bboxSize.x, _.assumed_minimum_pointcloud_bbox);
    mrpt::keep_max(bboxSize.y, _.assumed_minimum_pointcloud_bbox);
    mrpt::keep_max(bboxSize.z, _.assumed_minimum_pointcloud_bbox);

    const float largest_dim = bboxSize.norm();  // diagonal

    const float voxel_size = largest_dim / _.maximum_voxel_count_per_dimension;

    // Parse input cloud thru voxelization:
    filter_grid_.setResolution(voxel_size);
    filter_grid_.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    //    std::vector<double> logWeights;
    std::vector<size_t> weightedPointIndices;

    weightedPointIndices.reserve(
        filter_grid_.pts_voxels.size() * _.minimum_input_points_per_voxel);

    std::size_t nTotalVoxels = 0;
    for (const auto& [idx, data] : filter_grid_.pts_voxels)
    {
        if (!data.indices.empty()) nTotalVoxels++;
        if (data.indices.size() < _.minimum_input_points_per_voxel) continue;

#if 0
        // Analyze the voxel contents:
        double logWeight = 0.0;
        if (data.indices.size() > 3)
        {
            // Find eigenvalues & eigenvectors:
            const mp2p_icp::PointCloudEigen stats =
                mp2p_icp::estimate_points_eigen(
                    xs.data(), ys.data(), zs.data(), data.indices);

            const double e0 = stats.eigVals[0], e1 = stats.eigVals[1],
                         e2 = stats.eigVals[2];

            // w= (sqrt(e0)*sqrt(e1)*sqrt(e2))/n
            // log(w) = - log(n)
            logWeight = 0.5 * (std::log(e0) + std::log(e1) + std::log(e2)) -
                        std::log(data.indices.size());
        }
        else
        {
            // Not enough information to analyze the spatial distribution of
            // points.
            logWeight = -20;
        }
#endif

        weightedPointIndices.push_back(data.indices.front());
    }

    // Perform uniform weighted resampling:
    // -----------------------------------------
#if 0
    std::vector<size_t> pickedPointIdxs;

    mrpt::bayes::CParticleFilterCapable::computeResampling(
        mrpt::bayes::CParticleFilter::prMultinomial, logWeights,
        pickedPointIdxs);

    mrpt::random::shuffle(pickedPointIdxs.begin(), pickedPointIdxs.end());

    std::set<size_t> usedIdx;

    for (size_t i = 0; i < _.desired_output_point_count; i++)
    {
        size_t ptIdx = pickedPointIdxs[i];
#endif
    for (size_t ptIdx : weightedPointIndices)
    {
        outPc->insertPointFast(xs[ptIdx], ys[ptIdx], zs[ptIdx]);
    }

    MRPT_LOG_DEBUG_STREAM(
        "voxel_size=" << voxel_size <<  //
        ", used voxels=" << nTotalVoxels);

    MRPT_END
}
