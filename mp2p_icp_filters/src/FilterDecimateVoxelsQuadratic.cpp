/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateVoxelsQuadratic.cpp
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 14, 2023
 */

#include <mp2p_icp_filters/FilterDecimateVoxelsQuadratic.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/random/RandomGenerators.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterDecimateVoxelsQuadratic, mp2p_icp_filters::FilterBase,
    mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateVoxelsQuadratic::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, error_on_missing_input_layer);
    MCP_LOAD_OPT(c, use_random_point_within_voxel);

    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_REQ(c, voxel_filter_resolution);
    MCP_LOAD_REQ(c, quadratic_reference_radius);
    MCP_LOAD_REQ(c, use_voxel_average);
    MCP_LOAD_REQ(c, use_closest_to_voxel_average);
}

FilterDecimateVoxelsQuadratic::FilterDecimateVoxelsQuadratic() = default;

void FilterDecimateVoxelsQuadratic::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    filter_grid_.setResolution(params_.voxel_filter_resolution);
    quadratic_reference_radius_inv_ = 1.0f / params_.quadratic_reference_radius;

    MRPT_END
}

void FilterDecimateVoxelsQuadratic::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPc =
        GetOrCreatePointLayer(inOut, params_.output_pointcloud_layer);

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

    // Non-linear transform input cloud:
    const auto& rawPc = *pcPtr;

    mrpt::maps::CSimplePointsMap pc;
    pc.reserve(rawPc.size());

    const auto& xs = rawPc.getPointsBufferRef_x();
    const auto& ys = rawPc.getPointsBufferRef_y();
    const auto& zs = rawPc.getPointsBufferRef_z();
    for (size_t i = 0; i < xs.size(); i++)
    {
        pc.insertPointFast(
            real2grid(xs[i]), real2grid(ys[i]), real2grid(zs[i]));
    }
    pc.mark_as_modified();

    // Do filter:
    outPc->reserve(outPc->size() + pc.size() / 10);

    filter_grid_.clear();
    filter_grid_.processPointCloud(pc);

    //    const auto& xs = pc.getPointsBufferRef_x();
    //    const auto& ys = pc.getPointsBufferRef_y();
    //    const auto& zs = pc.getPointsBufferRef_z();

    auto rng = mrpt::random::CRandomGenerator();
    // TODO?: rng.randomize(seed);

    // Inverse nonlinear transformation:
    auto lambdaInsertPt = [&outPc](float x, float y, float z) {
        outPc->insertPointFast(x, y, z);
    };

    size_t nonEmptyVoxels = 0;
    for (const auto& vxl_pts : filter_grid_.pts_voxels)
    {
        if (vxl_pts.second.indices.empty()) continue;

        nonEmptyVoxels++;

        if (params_.use_voxel_average || params_.use_closest_to_voxel_average)
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
                lambdaInsertPt(xs[*bestIdx], ys[*bestIdx], zs[*bestIdx]);
            }
            else
            {
                // Insert the mean:
                lambdaInsertPt(mean.x, mean.y, mean.z);
            }
        }
        else
        {
            // Insert a randomly-picked point:
            const auto idxInVoxel =
                params_.use_random_point_within_voxel
                    ? (rng.drawUniform64bit() % vxl_pts.second.indices.size())
                    : 0UL;

            const auto pt_idx = vxl_pts.second.indices.at(idxInVoxel);
            lambdaInsertPt(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
        }
    }
    outPc->mark_as_modified();

    MRPT_LOG_DEBUG_STREAM("Voxel counts: total=" << nonEmptyVoxels);

    MRPT_END
}
