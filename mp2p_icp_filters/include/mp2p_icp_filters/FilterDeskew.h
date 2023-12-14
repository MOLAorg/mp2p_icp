/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDeskew.h
 * @brief  Deskew (motion compensate) a pointcloud from a moving LIDAR
 * @author Jose Luis Blanco Claraco
 * @date   Dec 13, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TTwist3D.h>

namespace mp2p_icp_filters
{
/** Builds a new layer with a deskewed (motion compensated) version of an
 *  input pointcloud from a moving LIDAR, where points are time-stamped.
 *
 * Important notes:
 * - The `time` field of each point is assumed to be in seconds since the
 * begining of the scan (e.g. from 0.0 to 0.1 for 10 Hz scans).
 * - The input layer must contain a point cloud in the format
 * mrpt::maps::CPointsMapXYZIRT so timestamps are present.
 * - If the input layer is of a different type, or the `time` field is missing,
 * an exception will be thrown by default, unless the option
 * `silently_ignore_no_timestamps` is set to `true`, in which case the input
 * cloud will be just moved forward to the output.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDeskew : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDeskew, mp2p_icp_filters)
   public:
    FilterDeskew();

    /** Parameters:
     *
     * \code
     * params:
     *   input_pointcloud_layer: 'raw'
     *   output_pointcloud_layer: 'deskewed'
     *   silently_ignore_no_timestamps: false
     *   # These (vx,...wz) are variable names that must be defined via the
     *   # mp2p_icp::Parameterizable API to update them dynamically.
     *   twist: [VX,VY,VZ,WX,WY,WZ]
     * \endcode
     *
     */
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    /** An input layer, from which to read input points
     *  Points must be already in the vehicle frame.
     */
    std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

    /** The output point cloud layer name */
    std::string output_pointcloud_layer;

    /** The class name for output layer if it does not exist and needs to be
     * created */
    std::string output_layer_class = "mrpt::maps::CPointsXYZI";

    /** Whether to skip throwing an exception if the input layer does not
     * contain timestamps.
     */
    bool silently_ignore_no_timestamps = false;

    /** The velocity (linear and angular) of the vehicle in the local
     * vehicle frame. See FilterDeskew::initialize for an example of how
     * to define it via dynamic variables.
     */
    mrpt::math::TTwist3D twist;
};

/** @} */

}  // namespace mp2p_icp_filters
