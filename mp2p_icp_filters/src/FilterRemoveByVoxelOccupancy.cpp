/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterRemoveByVoxelOccupancy.cpp
 * @brief  Removes points from an input point cloud layer by occupancy of
 * another input voxel layer. Can be used to remove dynamic objects.
 * @author Jose Luis Blanco Claraco
 * @date   May 28, 2024
 */

#include <mp2p_icp_filters/FilterRemoveByVoxelOccupancy.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CVoxelMap.h>

IMPLEMENTS_MRPT_OBJECT(FilterRemoveByVoxelOccupancy, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterRemoveByVoxelOccupancy::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterRemoveByVoxelOccupancy& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, input_voxel_layer);
    MCP_LOAD_OPT(c, output_layer_static_objects);
    MCP_LOAD_OPT(c, output_layer_dynamic_objects);

    DECLARE_PARAMETER_IN_REQ(c, occupancy_threshold, parent);
}

FilterRemoveByVoxelOccupancy::FilterRemoveByVoxelOccupancy()
{
    mrpt::system::COutputLogger::setLoggerName("FilterRemoveByVoxelOccupancy");
}

void FilterRemoveByVoxelOccupancy::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterRemoveByVoxelOccupancy::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    ASSERT_LE_(params_.occupancy_threshold, 1.0);
    ASSERT_GE_(params_.occupancy_threshold, 0.0);

    // In pts:
    ASSERTMSG_(
        inOut.layers.count(params_.input_pointcloud_layer) != 0,
        mrpt::format("Input layer '%s' not found.", params_.input_pointcloud_layer.c_str()));

    const auto mapPtr = inOut.layers.at(params_.input_pointcloud_layer);
    ASSERT_(mapPtr);

    const auto pcPtr = mp2p_icp::MapToPointsMap(*mapPtr);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' could not be converted into a "
                   "point cloud (class='%s')",
                   params_.input_pointcloud_layer.c_str(), mapPtr->GetRuntimeClass()->className));

    // In voxels:
    ASSERTMSG_(
        inOut.layers.count(params_.input_voxel_layer) != 0,
        mrpt::format("Input layer '%s' not found.", params_.input_voxel_layer.c_str()));

    const auto voxelMapPtr = inOut.layers.at(params_.input_voxel_layer);
    ASSERT_(voxelMapPtr);

    const auto voxelPtr = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(voxelMapPtr);
    ASSERTMSG_(
        voxelPtr,
        mrpt::format(
            "Input voxel layer '%s' not of type mrpt::maps::CVoxelMap"
            "(actual class='%s')",
            params_.input_voxel_layer.c_str(), voxelMapPtr->GetRuntimeClass()->className));

    // Outputs:
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPcStatic = GetOrCreatePointLayer(
        inOut, params_.output_layer_static_objects,
        /*allow empty name for nullptr*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPcStatic) outPcStatic->reserve(outPcStatic->size() + pcPtr->size() / 2);

    mrpt::maps::CPointsMap::Ptr outPcDynamic = GetOrCreatePointLayer(
        inOut, params_.output_layer_dynamic_objects,
        /*allow empty name for nullptr*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPcDynamic) outPcDynamic->reserve(outPcDynamic->size() + pcPtr->size() / 2);

    ASSERTMSG_(
        outPcStatic || outPcDynamic,
        "At least one 'output_layer_static_objects' or "
        "'output_layer_dynamic_objects' output layers must be provided.");

    // Process occupancy input value so it lies within [0,0.5]:
    const double occFree  = params_.occupancy_threshold > 0.5 ? (1.0 - params_.occupancy_threshold)
                                                              : params_.occupancy_threshold;
    const double occThres = 1.0 - occFree;

    // Process:
    const auto&  xs = pcPtr->getPointsBufferRef_x();
    const auto&  ys = pcPtr->getPointsBufferRef_y();
    const auto&  zs = pcPtr->getPointsBufferRef_z();
    const size_t N  = xs.size();

    size_t nStatic = 0, nDynamic = 0;

    for (size_t i = 0; i < N; i++)
    {
        double prob_occupancy = 0.5;
        bool   withinMap      = voxelPtr->getPointOccupancy(xs[i], ys[i], zs[i], prob_occupancy);
        if (!withinMap) continue;  // undefined! pt out of voxelmap

        mrpt::maps::CPointsMap* trgMap = nullptr;

        if (prob_occupancy > occThres)
        {
            trgMap = outPcStatic.get();
            nDynamic++;
        }
        else if (prob_occupancy < occFree)
        {
            trgMap = outPcDynamic.get();
            nStatic++;
        }

        if (!trgMap) continue;

        trgMap->insertPointFrom(*pcPtr, i);
    }

    MRPT_LOG_DEBUG_STREAM(
        "Parsed " << N << " points: static=" << nStatic << ", dynamic=" << nDynamic);

    MRPT_END
}
