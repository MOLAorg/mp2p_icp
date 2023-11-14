/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateVoxelsQuadratic.h
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 14, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Builds a new layer with a decimated version of an input layer, with a
 *  non-linear spatial lattice with wider voxels near the origin.
 *
 * This builds a voxel grid from the input point cloud, and then takes either,
 * the mean of the points in the voxel, or one of the points picked at random,
 * depending on the parameter `use_voxel_average`.
 *
 * If the given output pointcloud layer already exists, new points will be
 * appended, without clearing the former contents.
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimateVoxelsQuadratic : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimateVoxelsQuadratic, mp2p_icp_filters)
   public:
    FilterDecimateVoxelsQuadratic();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Whether to throw an exception if the input layer does not exist, or,
         * otherwise, it should be silently ignored producing an empty output.
         */
        bool error_on_missing_input_layer = true;

        /** The output point cloud layer name */
        std::string output_pointcloud_layer;

        /** Size of each voxel edge [meters] */
        double voxel_filter_resolution = .20;  // [m]

        /** Radius of the non-linear quadratic coordinates mapping [meters] */
        double quadratic_reference_radius = 20.0;  // [m]

        /** If enabled, the mean of each voxel is taken instead of any of
         *  the original points. */
        bool use_voxel_average = false;

        /** If enabled, the *actual* data point closest to the mean of each
         * voxel is taken as representative for each voxel. */
        bool use_closest_to_voxel_average = false;

        /** If false (default), the first point in each voxel will be returned
         * as voxel representative. Otherwise, one picked at random. */
        bool use_random_point_within_voxel = false;
    };

    /** Algorithm parameters */
    Parameters params_;

    inline float real2grid(float x) const
    {
        if (std::abs(x) > params_.quadratic_reference_radius)
            return x;
        else
            return mrpt::sign(x) * mrpt::square(x) *
                   quadratic_reference_radius_inv_;
    }
    inline float grid2real(float y) const
    {
        if (std::abs(y) > params_.quadratic_reference_radius)
            return y;
        else
            return std::sqrt(y * params_.quadratic_reference_radius) *
                   mrpt::sign(y);
    }

   private:
    mutable PointCloudToVoxelGrid filter_grid_;

    float quadratic_reference_radius_inv_ = 1.0f;
};

/** @} */

}  // namespace mp2p_icp_filters
