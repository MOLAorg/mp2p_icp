/* _
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
 * @file   FilterMLS.h
 * @brief  Applies a Moving Least Squares (MLS) filter to a point cloud.
 * @author Jose Luis Blanco and Google Gemini
 * @date   Oct 26, 2025
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mp2p_icp_filters
{
/** Applies a Moving Least Squares (MLS) surface reconstruction filter to a
 * point cloud.
 *
 * This filter can be used for smoothing and normal estimation. It can also
 * project points from a different cloud (`distinct_cloud_layer`) onto the
 * surface reconstructed from the `input_pointcloud_layer`.
 *
 * The computed normals are stored as new fields in the output point map
 * ("normal_x", "normal_y", "normal_z").
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterMLS : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterMLS, mp2p_icp_filters)
   public:
    FilterMLS();

    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    /** Defines the projection method onto the fitted polynomial surface,
     * inspired by pcl::MLSResult::ProjectionMethod.
     */
    enum class ProjectionMethod
    {
        /** Project along the surface normal to the polynomial. */
        SIMPLE = 0
    };

    /** Defines the upsampling method,
     * inspired by pcl::MovingLeastSquares::UpsamplingMethod.
     */
    enum class UpsamplingMethod
    {
        /** No upsampling, just project the original points. */
        NONE = 0,
        /** Project the points of the distinct cloud to the MLS surface. */
        DISTINCT_CLOUD
    };

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        std::string output_pointcloud_layer = "mls";

        /** (Optional) If `upsampling_method` is `DISTINCT_CLOUD`,
         * this layer's points will be projected onto the MLS surface
         * built from `input_pointcloud_layer`.
         */
        std::string distinct_cloud_layer;

        /** Search radius for finding neighbors to fit the MLS surface
         *. */
        float search_radius = 0.05f;

        /** Order of the polynomial to fit. (1=planar, 2=quadratic)
         */
        int polynomial_order = 2;

        /** Method to use for projecting points onto the fitted surface. */
        ProjectionMethod projection_method = ProjectionMethod::SIMPLE;

        /** Method to use for upsampling or smoothing. */
        UpsamplingMethod upsampling_method = UpsamplingMethod::NONE;

        /** Minimum number of neighbors to compute a fit. */
        int min_neighbors_for_fit = 3;

        /// When TBB is enabled, the grainsize for splitting the input clouds
        size_t parallelization_grain_size = 1024UL;
    };

    /** Algorithm parameters */
    Parameters params;

   private:
    /** Hidden implementation details (e.g., Eigen types)
     *
     */
    struct Impl;
    mutable mrpt::pimpl<Impl> impl_;
};

}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::FilterMLS::ProjectionMethod)
MRPT_FILL_ENUM(FilterMLS::ProjectionMethod::SIMPLE);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::FilterMLS::UpsamplingMethod)
MRPT_FILL_ENUM(FilterMLS::UpsamplingMethod::NONE);
MRPT_FILL_ENUM(FilterMLS::UpsamplingMethod::DISTINCT_CLOUD);
MRPT_ENUM_TYPE_END()
