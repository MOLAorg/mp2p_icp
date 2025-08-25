/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterDecimateAdaptive.cpp
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>

IMPLEMENTS_MRPT_OBJECT(FilterDecimateAdaptive, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateAdaptive::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, enabled);

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

    if (!params_.enabled) return;

    // In:
    const auto& pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc =
        GetOrCreatePointLayer(inOut, params_.output_pointcloud_layer);

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
    filter_grid_.setConfiguration(voxel_size, true);
    filter_grid_.processPointCloud(pc);

    struct DataPerVoxel
    {
        const PointCloudToVoxelGrid::voxel_t* voxel     = nullptr;
        uint32_t                              nextIdx   = 0;
        bool                                  exhausted = false;
    };

    // A list of all "valid" voxels:
    std::vector<DataPerVoxel> voxels;
    voxels.reserve(filter_grid_.size());

    std::size_t nTotalVoxels = 0;
    filter_grid_.visit_voxels(
        [&](const PointCloudToVoxelGrid::indices_t&, const PointCloudToVoxelGrid::voxel_t& data)
        {
            if (!data.indices.empty()) nTotalVoxels++;
            if (data.indices.size() < _.minimum_input_points_per_voxel) return;

            voxels.emplace_back().voxel = &data;
        });

    // Perform resampling:
    // -------------------
    const size_t nVoxels           = voxels.size();
    size_t       voxelIdxIncrement = 1;
    if (params_.desired_output_point_count < nVoxels)
    {
        voxelIdxIncrement = std::max<size_t>(
            1, mrpt::round(nVoxels / static_cast<float>(params_.desired_output_point_count)));
    }

    bool anyInsertInTheRound = false;

    for (size_t i = 0; outPc->size() < params_.desired_output_point_count;)
    {
        auto& ith = voxels[i];
        if (!ith.exhausted)
        {
            auto ptIdx = ith.voxel->indices[ith.nextIdx++];
            outPc->insertPointFrom(pc, ptIdx);
            anyInsertInTheRound = true;

            if (ith.nextIdx >= ith.voxel->indices.size()) ith.exhausted = true;
        }

        i += voxelIdxIncrement;
        if (i >= nVoxels)
        {
            // one round done.
            i = (i + 123653 /*a large arbitrary prime*/) % nVoxels;

            if (!anyInsertInTheRound)
            {
                // This means there is no more points and we must end
                // despite we didn't reached the user's desired number of
                // points:
                break;
            }

            anyInsertInTheRound = false;
        }
    }

    MRPT_LOG_DEBUG_STREAM(
        "voxel_size=" << voxel_size <<  //
        ", used voxels=" << nTotalVoxels);

    MRPT_END
}
