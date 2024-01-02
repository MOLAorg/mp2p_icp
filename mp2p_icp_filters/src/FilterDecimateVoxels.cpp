/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/random/RandomGenerators.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterDecimateVoxels, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

MRPT_TODO("Define enum type for selected operation?");

void FilterDecimateVoxels::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterDecimateVoxels& parent)
{
    ASSERTMSG_(
        c.has("input_pointcloud_layer"),
        "YAML configuration must have an entry `input_pointcloud_layer` with a "
        "scalar or sequence.");

    input_pointcloud_layer.clear();

    auto cfgIn = c["input_pointcloud_layer"];
    if (cfgIn.isScalar())
    {
        input_pointcloud_layer.push_back(cfgIn.as<std::string>());
    }
    else
    {
        ASSERTMSG_(
            cfgIn.isSequence(),
            "YAML configuration must have an entry `input_pointcloud_layer` "
            "with a scalar or sequence.");

        for (const auto& s : cfgIn.asSequence())
            input_pointcloud_layer.push_back(s.as<std::string>());
    }
    ASSERT_(!input_pointcloud_layer.empty());

    MCP_LOAD_OPT(c, error_on_missing_input_layer);
    MCP_LOAD_OPT(c, use_random_point_within_voxel);

    MCP_LOAD_REQ(c, output_pointcloud_layer);

    DECLARE_PARAMETER_IN_REQ(c, voxel_filter_resolution, parent);

    MCP_LOAD_OPT(c, use_voxel_average);
    MCP_LOAD_OPT(c, use_closest_to_voxel_average);
}

FilterDecimateVoxels::FilterDecimateVoxels()
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateVoxels");
}

void FilterDecimateVoxels::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    filter_grid_single_.reset();
    filter_grid_.reset();

    if (useSingleGrid())
    {  // Create:
        filter_grid_single_.emplace();
    }
    else
    {  // Create:
        filter_grid_.emplace();
    }

    MRPT_END
}

void FilterDecimateVoxels::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    std::vector<mrpt::maps::CPointsMap*> pcPtrs;
    size_t                               reserveSize = 0;
    for (const auto& inputLayer : params_.input_pointcloud_layer)
    {
        if (auto itLy = inOut.layers.find(inputLayer);
            itLy != inOut.layers.end())
        {
            auto pcPtr = mp2p_icp::MapToPointsMap(*itLy->second);
            if (!pcPtr)
                THROW_EXCEPTION_FMT(
                    "Layer '%s' must be of point cloud type.",
                    inputLayer.c_str());

            pcPtrs.push_back(pcPtr);
            reserveSize += pcPtr->size() / 10;  // heuristic
        }
        else
        {
            // Input layer doesn't exist:
            if (params_.error_on_missing_input_layer)
            {
                THROW_EXCEPTION_FMT(
                    "Input layer '%s' not found on input map.",
                    inputLayer.c_str());
            }
            else
            {
                // Silently return with an unmodified output layer "outPc"
                continue;
            }
        }
    }

    ASSERT_(!pcPtrs.empty());

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPc = GetOrCreatePointLayer(
        inOut, params_.output_pointcloud_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type */
        pcPtrs.at(0)->GetRuntimeClass()->className);

    outPc->reserve(outPc->size() + reserveSize);

    // Do filter:
    size_t nonEmptyVoxels = 0;

    if (useSingleGrid())
    {
        ASSERTMSG_(
            filter_grid_single_.has_value(),
            "Has you called initialize() after updating/loading parameters?");

        auto& grid = filter_grid_single_.value();
        grid.setResolution(params_.voxel_filter_resolution);
        grid.clear();

        // 1st) go thru all the input layers:
        for (const auto& pcPtr : pcPtrs)
        {
            const auto& pc = *pcPtr;
            grid.processPointCloud(pc);
        }

        // 2nd) collect grid results:
        for (const auto& [idx, vxl] : grid.pts_voxels)
        {
            if (!vxl.pointIdx.has_value()) continue;

            nonEmptyVoxels++;

            outPc->insertPointFrom(*vxl.source.value(), vxl.pointIdx.value());
        }
    }
    else
    {
        ASSERTMSG_(
            pcPtrs.size() == 1,
            "Only one input layer allowed when requiring the non-single "
            "decimating grid");

        const auto& pc = *pcPtrs.at(0);

        ASSERTMSG_(
            filter_grid_.has_value(),
            "Has you called initialize() after updating/loading parameters?");

        auto& grid = filter_grid_.value();
        grid.setResolution(params_.voxel_filter_resolution);
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

    outPc->mark_as_modified();

    MRPT_LOG_DEBUG_STREAM(
        "Voxel count=" << nonEmptyVoxels
                       << ", output_layer=" << params_.output_pointcloud_layer
                       << " type=" << outPc->GetRuntimeClass()->className
                       << " useSingleGrid="
                       << (useSingleGrid() ? "Yes" : "No"));

    MRPT_END
}
