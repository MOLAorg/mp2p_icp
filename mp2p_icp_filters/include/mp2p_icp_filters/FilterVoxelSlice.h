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
 * @file   FilterVoxelSlice.h
 * @brief  Takes an input layer of type CVoxelMap (Bonxai) and extracts one 2D
 * slice as an occupancy gridmap.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/TPose3D.h>

namespace mp2p_icp_filters
{
/** Takes an input layer of type mrpt::maps::CVoxelMap (Bonxai) and extracts one
 * 2D slice as an occupancy gridmap, taken at a given height ("z") range of
 * values. That is, each voxel column will be collapsed into a 2D grid cell.
 *
 * If the output layer already exists, it will be overwritten.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterVoxelSlice : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterVoxelSlice, mp2p_icp_filters)
   public:
    FilterVoxelSlice();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterVoxelSlice& parent);

        std::string input_layer;
        std::string output_layer;

        double slice_z_min = 0;  // [m]
        double slice_z_max = 0;  // [m]
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
