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
 * @file   FilterVoxelSlice.cpp
 * @brief  Takes an input layer of type CVoxelMap (Bonxai) and extracts one 2D
 * slice as an occupancy gridmap.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2024
 */

#include <mp2p_icp_filters/FilterVoxelSlice.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationPointCloud.h>

IMPLEMENTS_MRPT_OBJECT(FilterVoxelSlice, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterVoxelSlice::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterVoxelSlice& parent)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer);

    DECLARE_PARAMETER_IN_REQ(c, slice_z_min, parent);
    DECLARE_PARAMETER_IN_REQ(c, slice_z_max, parent);
}

FilterVoxelSlice::FilterVoxelSlice()
{
    mrpt::system::COutputLogger::setLoggerName("FilterVoxelSlice");
}

void FilterVoxelSlice::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterVoxelSlice::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    ASSERT_GE_(params_.slice_z_max, params_.slice_z_min);

    // In:
    ASSERT_(!params_.input_layer.empty());
    ASSERTMSG_(
        inOut.layers.count(params_.input_layer),
        mrpt::format("Input layer '%s' was not found.", params_.input_layer.c_str()));

    const auto in            = inOut.layers.at(params_.input_layer);
    auto       inVoxelMap    = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(in);
    auto       inVoxelMapRGB = std::dynamic_pointer_cast<mrpt::maps::CVoxelMapRGB>(in);

    if (!inVoxelMap && !inVoxelMapRGB)
    {
        THROW_EXCEPTION_FMT(
            "Input layer is of class '%s', expected one of: "
            "mrpt::maps::CVoxelMap, mrpt::maps::CVoxelMapRGB",
            in->GetRuntimeClass()->className);
    }

    // Out:
    ASSERT_(!params_.output_layer.empty());

    auto occGrid                       = mrpt::maps::COccupancyGridMap2D::Create();
    inOut.layers[params_.output_layer] = occGrid;

    // Set the grid "height" (z):
    occGrid->insertionOptions.mapAltitude = 0.5 * (params_.slice_z_max + params_.slice_z_min);

    // make the conversion:
    if (inVoxelMap)
    {
        auto& grid =
            const_cast<Bonxai::VoxelGrid<mrpt::maps::CVoxelMap::voxel_node_t>&>(inVoxelMap->grid());

        const mrpt::math::TBoundingBoxf bbox = inVoxelMap->boundingBox();

        occGrid->setSize(bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y, grid.resolution);

        const auto zCoordMin =
            Bonxai::PosToCoord({0., 0., params_.slice_z_min}, grid.inv_resolution);
        const auto zCoordMax =
            Bonxai::PosToCoord({0., 0., params_.slice_z_max}, grid.inv_resolution);

        // Go thru all voxels:
        auto lmbdPerVoxel =
            [&](mrpt::maps::CVoxelMap::voxel_node_t& data, const Bonxai::CoordT& coord)
        {
            // are we at the correct height?
            if (coord.z < zCoordMin.z || coord.z > zCoordMax.z) return;
            const auto pt = Bonxai::CoordToPos(coord, grid.resolution);

            const double freeness = inVoxelMap->l2p(data.occupancy);

            // Bayesian fuse information:
            occGrid->updateCell(occGrid->x2idx(pt.x), occGrid->y2idx(pt.y), freeness);
        };  // end lambda for each voxel

        grid.forEachCell(lmbdPerVoxel);
    }
    else if (inVoxelMapRGB)
    {
        // ...
    }

    MRPT_END
}
