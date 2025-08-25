/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateVoxels.cpp
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 10, 2021
 */

#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/random/RandomGenerators.h>

//
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>

IMPLEMENTS_MRPT_OBJECT(FilterDecimateVoxels, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

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
        {
            input_pointcloud_layer.push_back(s.as<std::string>());
        }
    }
    ASSERT_(!input_pointcloud_layer.empty());

    MCP_LOAD_OPT(c, error_on_missing_input_layer);
    MCP_LOAD_REQ(c, decimate_method);

    MCP_LOAD_REQ(c, output_pointcloud_layer);
    MCP_LOAD_OPT(c, minimum_input_points_to_filter);

    DECLARE_PARAMETER_IN_REQ(c, voxel_filter_resolution, parent);
    MCP_LOAD_OPT(c, use_tsl_robin_map);

    if (c.has("flatten_to"))
    {
        flatten_to = c["flatten_to"].as<double>();
    }
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
        if (auto itLy = inOut.layers.find(inputLayer); itLy != inOut.layers.end())
        {
            auto pcPtr = mp2p_icp::MapToPointsMap(*itLy->second);
            if (!pcPtr)
            {
                THROW_EXCEPTION_FMT("Layer '%s' must be of point cloud type.", inputLayer.c_str());
            }

            pcPtrs.push_back(pcPtr);
            reserveSize += pcPtr->size() / 10;  // heuristic
        }
        else
        {
            // Input layer doesn't exist:
            if (params_.error_on_missing_input_layer)
            {
                THROW_EXCEPTION_FMT("Input layer '%s' not found on input map.", inputLayer.c_str());
            }

            // Silently return with an unmodified output layer "outPc"
            continue;
        }
    }

    ASSERT_(!pcPtrs.empty());

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params_.output_pointcloud_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type */
        pcPtrs.at(0)->GetRuntimeClass()->className);

    outPc->reserve(outPc->size() + reserveSize);

    // Skip filtering for layers with less than
    // "minimum_input_points_to_filter":
    if (params_.minimum_input_points_to_filter > 0)
    {
        std::vector<size_t> idxsToRemove;
        for (size_t mapIdx = 0; mapIdx < pcPtrs.size(); mapIdx++)
        {
            if (pcPtrs[mapIdx]->size() > params_.minimum_input_points_to_filter)
            {  // just proceed as standard with this large map:
                continue;
            }
            // Remove this one from the list of maps to filter,
            // and just add all points:
            idxsToRemove.push_back(mapIdx);

            const auto& xs = pcPtrs[mapIdx]->getPointsBufferRef_x();
            const auto& ys = pcPtrs[mapIdx]->getPointsBufferRef_y();

            for (size_t i = 0; i < xs.size(); i++)
            {
                if (params_.flatten_to.has_value())
                {
                    outPc->insertPointFast(xs[i], ys[i], *params_.flatten_to);
                }
                else
                {
                    outPc->insertPointFrom(*pcPtrs[mapIdx], i);
                }
            }
        }
        for (auto it = idxsToRemove.rbegin(); it != idxsToRemove.rend(); ++it)
        {
            pcPtrs.erase(pcPtrs.begin() + *it);
        }

    }  // end handle special case minimum_input_points_to_filter

    // Do filter:
    size_t nonEmptyVoxels = 0;

    if (useSingleGrid())
    {
        ASSERTMSG_(
            filter_grid_single_.has_value(),
            "Has you called initialize() after updating/loading parameters?");

        auto& grid = filter_grid_single_.value();
        grid.setConfiguration(params_.voxel_filter_resolution, params_.use_tsl_robin_map);
        grid.clear();

        // 1st) go thru all the input layers:
        for (const auto& pcPtr : pcPtrs)
        {
            const auto& pc = *pcPtr;
            grid.processPointCloud(pc);

            const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(pc);
            ASSERT_(sanityPassed);
        }

        // 2nd) collect grid results:
        std::set<PointCloudToVoxelGridSingle::indices_t, PointCloudToVoxelGridSingle::IndicesHash>
            flattenUsedBins;

        grid.visit_voxels(
            [&](const PointCloudToVoxelGridSingle::indices_t& idx,
                const PointCloudToVoxelGridSingle::voxel_t&   vxl)
            {
                if (!vxl.pointIdx.has_value())
                {
                    return;
                }

                nonEmptyVoxels++;

                if (params_.flatten_to.has_value())
                {
                    const PointCloudToVoxelGridSingle::indices_t flattenIdx = {idx.cx_, idx.cy_, 0};

                    // first time?
                    if (flattenUsedBins.count(flattenIdx) != 0)
                    {
                        return;  // nope. Skip this point.
                    }

                    // First time we see this (x,y) cell:
                    flattenUsedBins.insert(flattenIdx);

                    outPc->insertPointFast(vxl.point->x, vxl.point->y, *params_.flatten_to);
                }
                else
                {
                    outPc->insertPointFrom(*vxl.source.value(), *vxl.pointIdx);
                }
            });
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
        grid.setConfiguration(params_.voxel_filter_resolution, params_.use_tsl_robin_map);
        grid.clear();

        grid.processPointCloud(pc);

        const auto& xs = pc.getPointsBufferRef_x();
        const auto& ys = pc.getPointsBufferRef_y();
        const auto& zs = pc.getPointsBufferRef_z();

        auto rng = mrpt::random::CRandomGenerator();
        // TODO?: rng.randomize(seed);

        std::set<PointCloudToVoxelGrid::indices_t, PointCloudToVoxelGrid::IndicesHash>
            flattenUsedBins;

        grid.visit_voxels(
            [&](const PointCloudToVoxelGrid::indices_t& idx,
                const PointCloudToVoxelGrid::voxel_t&   vxl)
            {
                if (vxl.indices.empty()) return;

                nonEmptyVoxels++;
                std::optional<mrpt::math::TPoint3Df> insertPt;
                size_t insertPtIdx;  // valid only if insertPt is empty

                if (params_.decimate_method == DecimateMethod::VoxelAverage ||
                    params_.decimate_method == DecimateMethod::ClosestToAverage)
                {
                    // Analyze the voxel contents:
                    auto        mean  = mrpt::math::TPoint3Df(0, 0, 0);
                    const float inv_n = (1.0f / vxl.indices.size());
                    for (size_t i = 0; i < vxl.indices.size(); i++)
                    {
                        const auto pt_idx = vxl.indices[i];
                        mean.x += xs[pt_idx];
                        mean.y += ys[pt_idx];
                        mean.z += zs[pt_idx];
                    }
                    mean *= inv_n;

                    if (params_.decimate_method == DecimateMethod::ClosestToAverage)
                    {
                        std::optional<float>  minSqrErr;
                        std::optional<size_t> bestIdx;

                        for (size_t i = 0; i < vxl.indices.size(); i++)
                        {
                            const auto  pt_idx = vxl.indices[i];
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
                        insertPtIdx = *bestIdx;
                    }
                    else
                    {
                        // Insert the mean:
                        insertPt = {mean.x, mean.y, mean.z};
                    }
                }
                else
                {
                    // Insert a randomly-picked point:
                    const auto idxInVoxel = (params_.decimate_method == DecimateMethod::RandomPoint)
                                                ? (rng.drawUniform64bit() % vxl.indices.size())
                                                : 0UL;

                    const auto pt_idx = vxl.indices.at(idxInVoxel);
                    insertPtIdx       = pt_idx;
                }

                // insert it, if passed the flatten filter:
                if (params_.flatten_to.has_value())
                {
                    const PointCloudToVoxelGrid::indices_t flattenIdx = {idx.cx_, idx.cy_, 0};

                    // first time?
                    if (flattenUsedBins.count(flattenIdx) != 0) return;  // nope. Skip this point.

                    // First time we see this (x,y) cell:
                    flattenUsedBins.insert(flattenIdx);

                    if (!insertPt)
                        insertPt.emplace(xs[insertPtIdx], ys[insertPtIdx], zs[insertPtIdx]);
                    outPc->insertPointFast(insertPt->x, insertPt->y, *params_.flatten_to);
                }
                else
                {
                    if (insertPt)
                        outPc->insertPointFast(insertPt->x, insertPt->y, insertPt->z);
                    else
                    {
                        outPc->insertPointFrom(pc, insertPtIdx);
                    }
                }
            });

    }  // end: non-single grid

    outPc->mark_as_modified();

    MRPT_LOG_DEBUG_STREAM(
        "Voxel count=" << nonEmptyVoxels << ", output_layer=" << params_.output_pointcloud_layer
                       << " type=" << outPc->GetRuntimeClass()->className
                       << " useSingleGrid=" << (useSingleGrid() ? "Yes" : "No"));

    MRPT_END
}
