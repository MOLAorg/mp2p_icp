/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateVoxels.cpp
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 10, 2021
 */

#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/random/RandomGenerators.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterDecimateVoxels, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateVoxels::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);

    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_REQ(c, voxel_filter_resolution);
    MCP_LOAD_REQ(c, use_voxel_average);

    MCP_LOAD_OPT(c, init_extension_min_x);
    MCP_LOAD_OPT(c, init_extension_min_y);
    MCP_LOAD_OPT(c, init_extension_min_z);
    MCP_LOAD_OPT(c, init_extension_max_x);
    MCP_LOAD_OPT(c, init_extension_max_y);
    MCP_LOAD_OPT(c, init_extension_max_z);
}

FilterDecimateVoxels::FilterDecimateVoxels() = default;

void FilterDecimateVoxels::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    filter_grid_.resize(
        {params_.init_extension_min_x, params_.init_extension_min_y,
         params_.init_extension_min_z},
        {params_.init_extension_max_x, params_.init_extension_max_y,
         params_.init_extension_max_z},
        params_.voxel_filter_resolution);

    MRPT_END
}

void FilterDecimateVoxels::filter(mp2p_icp::pointcloud_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layers[params_.input_pointcloud_layer];
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    auto& outPc = inOut.point_layers[params_.output_pointcloud_layer];

    // Create if new: Append to existing layer, if already existed.
    if (!outPc) outPc = mrpt::maps::CSimplePointsMap::Create();

    outPc->reserve(outPc->size() + pc.size() / 10);

    filter_grid_.clear();
    filter_grid_.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    auto rng = mrpt::random::CRandomGenerator();
    // TODO?: rng.randomize(seed);

    size_t nonEmptyVoxels = 0;
    for (const auto& vxl_pts : filter_grid_.pts_voxels)
    {
        if (vxl_pts.indices.empty()) continue;

        nonEmptyVoxels++;

        if (params_.use_voxel_average)
        {
            // Analyze the voxel contents:
            auto        mean  = mrpt::math::TPoint3Df(0, 0, 0);
            const float inv_n = (1.0f / vxl_pts.indices.size());
            for (size_t i = 0; i < vxl_pts.indices.size(); i++)
            {
                const auto pt_idx = vxl_pts.indices[i];
                mean.x += xs[pt_idx];
                mean.y += ys[pt_idx];
                mean.z += zs[pt_idx];
            }
            mean *= inv_n;

            // Insert the mean:
            outPc->insertPointFast(mean.x, mean.y, mean.z);
        }
        else
        {
            // Insert a randomly-picked point:
            const auto idxInVoxel =
                rng.drawUniform64bit() % vxl_pts.indices.size();

            const auto pt_idx = vxl_pts.indices.at(idxInVoxel);
            outPc->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
        }
    }
    MRPT_LOG_DEBUG_STREAM("Voxel counts: total=" << nonEmptyVoxels);

    MRPT_END
}
