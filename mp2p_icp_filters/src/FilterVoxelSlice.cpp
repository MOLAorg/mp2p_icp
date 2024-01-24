/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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

IMPLEMENTS_MRPT_OBJECT(
    FilterVoxelSlice, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterVoxelSlice::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterVoxelSlice& parent)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer);

    DECLARE_PARAMETER_IN_REQ(c, slice_z, parent);
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

    // In:
    ASSERT_(!params_.input_layer.empty());
    ASSERTMSG_(
        inOut.layers.count(params_.input_layer),
        mrpt::format(
            "Input layer '%s' was not found.", params_.input_layer.c_str()));

    const auto in   = inOut.layers.at(params_.input_layer);
    auto inVoxelMap = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(in);
    auto inVoxelMapRGB =
        std::dynamic_pointer_cast<mrpt::maps::CVoxelMapRGB>(in);

    if (!inVoxelMap && !inVoxelMapRGB)
    {
        THROW_EXCEPTION_FMT(
            "Input layer is of class '%s', expected one of: "
            "mrpt::maps::CVoxelMap, mrpt::maps::CVoxelMapRGB",
            in->GetRuntimeClass()->className);
    }

    // Out:
    ASSERT_(!params_.output_layer.empty());

    auto occGrid = mrpt::maps::COccupancyGridMap2D::Create();
    inOut.layers[params_.output_layer] = occGrid;

    // make the conversion:
    if (inVoxelMap)
    {
        auto& grid =
            const_cast<Bonxai::VoxelGrid<mrpt::maps::CVoxelMap::voxel_node_t>&>(
                inVoxelMap->grid());

        const mrpt::math::TBoundingBoxf bbox = inVoxelMap->boundingBox();

        occGrid->setSize(
            bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y, grid.resolution);

        const auto zCoord =
            Bonxai::PosToCoord({0., 0., params_.slice_z}, grid.inv_resolution);

        // Go thru all voxels:
        auto lmbdPerVoxel = [&](mrpt::maps::CVoxelMap::voxel_node_t& data,
                                const Bonxai::CoordT&                coord) {
            // are we at the correct height?
            if (coord.z != zCoord.z) return;
            const auto pt = Bonxai::CoordToPos(coord, grid.resolution);

            const double freeness = inVoxelMap->l2p(data.occupancy);

            occGrid->setCell(
                occGrid->x2idx(pt.x), occGrid->y2idx(pt.y), freeness);
        };  // end lambda for each voxel

        grid.forEachCell(lmbdPerVoxel);
    }
    else if (inVoxelMapRGB)
    {
        // ...
    }

    MRPT_END
}
