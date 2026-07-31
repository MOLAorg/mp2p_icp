/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterDecimateAdaptive.h
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/DecimateMethod.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

#include <string>
#include <vector>

namespace mp2p_icp_filters
{
/** Accepts as input a point cloud layer, voxelizes it, and generates a new
 * point cloud layer with an adaptive sampling.
 *
 * More than one output layer can be requested (see Parameters::outputs), each
 * one with its own target point count. All of them are then generated from one
 * single voxelization pass over the input, which is the dominant cost, instead
 * of chaining several instances of this filter.
 *
 * Which point represents each voxel is selected with `decimate_method`, as in
 * FilterDecimateVoxels. Note though that this filter walks the voxels in
 * several rounds until the desired point count is reached, so:
 * - DecimateMethod::FirstPoint (the default, and the fastest) and
 *   DecimateMethod::RandomPoint take successive points out of each voxel, in
 *   insertion order or starting at a random offset, respectively.
 * - DecimateMethod::ClosestToAverage and DecimateMethod::VoxelAverage yield ONE
 *   point per voxel only, so the output cannot be larger than the number of
 *   valid voxels no matter the desired point count. They are also more
 *   expensive, since all voxel points must be traversed to get the average.
 *   DecimateMethod::VoxelAverage generates new points, so per-point fields
 *   (intensity, ring, timestamp, ...) are not propagated to the output.
 *
 * When built with TBB, voxels are accumulated per thread and not merged, so the
 * per-voxel averages above are computed over the subset of points that fell in
 * the same thread block.
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimateAdaptive : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimateAdaptive, mp2p_icp_filters)
   public:
    FilterDecimateAdaptive();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    /** One requested output layer, with its own target point count. */
    struct OutputTarget
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string  output_pointcloud_layer;
        unsigned int desired_output_point_count = 1000;
    };

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** The requested output layers. Loaded either from the YAML sequence
         * "outputs", or from the single-output keys "output_pointcloud_layer"
         * and "desired_output_point_count". */
        std::vector<OutputTarget> outputs;

        /** Voxels with less points will not generate anything at the output
         * layer */
        unsigned int minimum_input_points_per_voxel = 1;

        float voxel_size = 0.10;

        /** The method to pick what point will be used as representative of each
         * voxel. See notes on top of this class. */
        DecimateMethod decimate_method = DecimateMethod::FirstPoint;

        /// When TBB is enabled, the grainsize for splitting the input clouds into threads
        size_t parallelization_grain_size = 16UL * 1024UL;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

   private:
    /** The PointCloudToVoxelGrid filter objects.
     * Hidden inside a PIMP to allow TBB parallel implementation without polluting user headers with
     * tbb.
     */
    struct Impl;
    mutable mrpt::pimpl<Impl> impl_;
};

/** @} */

}  // namespace mp2p_icp_filters
