/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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

MRPT_TODO("Define enum type for selected operation?");

void FilterDecimateVoxels::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, error_on_missing_input_layer);
    MCP_LOAD_OPT(c, use_random_point_within_voxel);

    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_REQ(c, voxel_filter_resolution);
    MCP_LOAD_REQ(c, use_voxel_average);
    MCP_LOAD_REQ(c, use_closest_to_voxel_average);
}

FilterDecimateVoxels::FilterDecimateVoxels()
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateVoxels");
}

void FilterDecimateVoxels::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    filter_grid_single_.reset();
    filter_grid_.reset();

    if (useSingleGrid())
    {
        auto& grid = filter_grid_single_.emplace();
        grid.setResolution(params_.voxel_filter_resolution);
    }
    else
    {
        auto& grid = filter_grid_.emplace();
        grid.setResolution(params_.voxel_filter_resolution);
    }

    MRPT_END
}

void FilterDecimateVoxels::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

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

    // In:
    mrpt::maps::CPointsMap* pcPtr = nullptr;
    if (auto itLy = inOut.layers.find(params_.input_pointcloud_layer);
        itLy != inOut.layers.end())
    {
        pcPtr = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!pcPtr)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.",
                params_.input_pointcloud_layer.c_str());
    }
    else
    {
        // Input layer doesn't exist:
        if (params_.error_on_missing_input_layer)
        {
            THROW_EXCEPTION_FMT(
                "Input layer '%s' not found on input map.",
                params_.input_pointcloud_layer.c_str());
        }
        else
        {
            // Silently return with an unmodified output layer "outPc"
            return;
        }
    }

    const auto& pc = *pcPtr;

    // Do filter:
    outPc->reserve(outPc->size() + pc.size() / 10);

    size_t nonEmptyVoxels = 0;

    if (useSingleGrid())
    {
        ASSERTMSG_(
            filter_grid_single_.has_value(),
            "Has you called initialize() after updating/loading parameters?");

        auto& grid = filter_grid_single_.value();
        grid.clear();
        grid.processPointCloud(pc);

        const auto& xs = pc.getPointsBufferRef_x();
        const auto& ys = pc.getPointsBufferRef_y();
        const auto& zs = pc.getPointsBufferRef_z();

        for (const auto& vxl_pts : grid.pts_voxels)
        {
            if (!vxl_pts.second.index.has_value()) continue;

            nonEmptyVoxels++;

            const auto pt_idx = vxl_pts.second.index.value();
            outPc->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
        }
    }
    else
    {
        ASSERTMSG_(
            filter_grid_.has_value(),
            "Has you called initialize() after updating/loading parameters?");

        auto& grid = filter_grid_.value();
        grid.clear();
        grid.processPointCloud(pc);

        const auto& xs = pc.getPointsBufferRef_x();
        const auto& ys = pc.getPointsBufferRef_y();
        const auto& zs = pc.getPointsBufferRef_z();

        auto rng = mrpt::random::CRandomGenerator();
        // TODO?: rng.randomize(seed);

        for (const auto& vxl_pts : grid.pts_voxels)
        {
            if (vxl_pts.second.indices.empty()) continue;

            nonEmptyVoxels++;

            if (params_.use_voxel_average ||
                params_.use_closest_to_voxel_average)
            {
                // Analyze the voxel contents:
                auto        mean  = mrpt::math::TPoint3Df(0, 0, 0);
                const float inv_n = (1.0f / vxl_pts.second.indices.size());
                for (size_t i = 0; i < vxl_pts.second.indices.size(); i++)
                {
                    const auto pt_idx = vxl_pts.second.indices[i];
                    mean.x += xs[pt_idx];
                    mean.y += ys[pt_idx];
                    mean.z += zs[pt_idx];
                }
                mean *= inv_n;

                if (params_.use_closest_to_voxel_average)
                {
                    std::optional<float>  minSqrErr;
                    std::optional<size_t> bestIdx;

                    for (size_t i = 0; i < vxl_pts.second.indices.size(); i++)
                    {
                        const auto  pt_idx = vxl_pts.second.indices[i];
                        const float sqrErr = mrpt::square(xs[pt_idx] - mean.x) +
                                             mrpt::square(ys[pt_idx] - mean.y) +
                                             mrpt::square(zs[pt_idx] - mean.z);

                        if (!minSqrErr.has_value() || sqrErr < *minSqrErr)
                        {
                            minSqrErr = sqrErr;
                            bestIdx   = pt_idx;
                        }
                    }
                    // Insert the closest to the mean:
                    outPc->insertPointFast(
                        xs[*bestIdx], ys[*bestIdx], zs[*bestIdx]);
                }
                else
                {
                    // Insert the mean:
                    outPc->insertPointFast(mean.x, mean.y, mean.z);
                }
            }
            else
            {
                // Insert a randomly-picked point:
                const auto idxInVoxel = params_.use_random_point_within_voxel
                                            ? (rng.drawUniform64bit() %
                                               vxl_pts.second.indices.size())
                                            : 0UL;

                const auto pt_idx = vxl_pts.second.indices.at(idxInVoxel);
                outPc->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
    }  // end: non-single grid
    MRPT_LOG_DEBUG_STREAM(
        "Voxel count=" << nonEmptyVoxels
                       << ", input_layer=" << params_.input_pointcloud_layer
                       << " (" << pc.size() << " points)"
                       << ", output_layer=" << params_.output_pointcloud_layer);

    MRPT_END
}
