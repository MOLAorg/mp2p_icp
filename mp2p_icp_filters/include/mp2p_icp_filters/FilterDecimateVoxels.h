/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateVoxels.h
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 10, 2021
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mp2p_icp_filters
{
/** Enum to select what method to use to pick the downsampled point for each
 *  voxel in FilterDecimateVoxels.
 *
 * \ingroup mp2p_icp_filters_grp
 */
enum class DecimateMethod : uint8_t
{
    /** Pick the first point that was put int the voxel */
    FirstPoint = 0,
    /** Closest to the average of all voxel points */
    ClosestToAverage,
    /** Average of all voxel points */
    VoxelAverage,
    /** Pick one of the voxel points at random */
    RandomPoint
};

/** Builds a new layer with a decimated version of one or more input layers,
 * merging their contents.
 *
 * This builds a voxel grid from the input point cloud, and then takes either,
 * the mean of the points in the voxel, or one of the points picked at random,
 * depending on the parameter `decimate_method`.
 *
 * If the given output pointcloud layer already exists, new points will be
 * appended, without clearing the former contents.
 *
 * If the parameter `flatten_to` is defined, this filter will also "flatten" or
 * "summarize" the 3D points into a 2D planar (constant height `z`) cloud.
 *
 * Additional input point fields (ring, intensity, timestamp) will be copied
 * into the output target cloud, except when using the `flatten_to` option.
 *
 * If `minimum_input_points_to_filter` is defined, input clouds smaller than
 * that size will not be decimated at all.
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimateVoxels : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimateVoxels, mp2p_icp_filters)
   public:
    FilterDecimateVoxels();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterDecimateVoxels& parent);

        /** One or more input layers, from which to read (and merge) input
         * points */
        std::vector<std::string> input_pointcloud_layer = {mp2p_icp::metric_map_t::PT_LAYER_RAW};

        /** Whether to throw an exception if the input layer does not exist, or,
         * otherwise, it should be silently ignored producing an empty output.
         */
        bool error_on_missing_input_layer = true;

        /** The output point cloud layer name */
        std::string output_pointcloud_layer;

        /** Size of each voxel edge [meters] */
        float voxel_filter_resolution = 1.0f;  // [m]

        /** Whether to use as container implementation
         * tsl::robin_map (true, default), or a std::map (false) */
        bool voxel_use_tsl_robin_map = true;

        /** If !=0 and there are less input points that this number,
         *  all points will be just moved through without decimation.
         */
        uint32_t minimum_input_points_to_filter = 0;

        /// See description on top of this page.
        std::optional<double> flatten_to;

        /** The method to pick what point will be used as representative of each
         * voxel */
        DecimateMethod decimate_method = DecimateMethod::FirstPoint;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    mutable std::optional<PointCloudToVoxelGrid>       filter_grid_;
    mutable std::optional<PointCloudToVoxelGridSingle> filter_grid_single_;

    bool useSingleGrid() const { return params_.decimate_method == DecimateMethod::FirstPoint; }
};

/** @} */

}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::DecimateMethod)
MRPT_FILL_ENUM(DecimateMethod::FirstPoint);
MRPT_FILL_ENUM(DecimateMethod::ClosestToAverage);
MRPT_FILL_ENUM(DecimateMethod::VoxelAverage);
MRPT_FILL_ENUM(DecimateMethod::RandomPoint);
MRPT_ENUM_TYPE_END()
