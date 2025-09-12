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
#include <mrpt/typemeta/TEnumType.h>

namespace mp2p_icp_filters
{
/** Enum to select the pointcloud motion compensation method in FilterDeskew.
 *
 * Refer to mathematical description for each method in the paper [TO-DO].
 *
 * \ingroup mp2p_icp_filters_grp
 */
enum class MotionCompensationMethod : uint8_t
{
    /** No compensation: all points are considered to be at vehicle pose for `reference_time`=0 */
    None = 0,
    /** Constant linear and angular velocity model to interpolate between key-frames */
    Linear,
    /** IMU-based integration between IMU frames using Euler integration of constant linear
       acceleration and angular velocity */
    IMU,
    /** IMU-based integration between IMU frames using (h) higher-order terms: constant jerk and
     * angular acceleration */
    IMUh,
    /** IMU-based integration between IMU frames using (t) trapezoidal integration of constant
     * linear acceleration and angular velocity */
    IMUt
};

/** Builds a new layer with a deskewed (motion compensated) version of an
 *  input pointcloud from a moving LIDAR, where points are time-stamped.
 *
 * Important notes:
 * - The `time` field of each point is assumed to be in seconds since the
 *   reference time point, which can be the start or middle point of the scan. This can be
 *   controlled by adding a FilterAdjustTimestamps before this block.
 *
 * - The input layer must contain a point cloud in the format
 *   mrpt::maps::CPointsMapXYZIRT so timestamps are present.
 *
 * - If the input layer is of a different type, or the `time` field is missing,
 *   an exception will be thrown by default, unless the option
 *   `silently_ignore_no_timestamps` is set to `true`, in which case the input
 *   cloud will be just moved forward to the output.
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
     *   method: 'MotionCompensationMethod::Linear'
     *   # silently_ignore_no_timestamps: false
     *   # These (vx,...,wz) are variable names that must be defined via the
     *   # mp2p_icp::Parameterizable API to update them dynamically.
     *   twist: [vx,vy,vz,wx,wy,wz]
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
    std::string output_layer_class = "mrpt::maps::CPointsMapXYZI";

    /** Whether to skip throwing an exception if the input layer does not
     * contain timestamps.
     */
    bool silently_ignore_no_timestamps = false;

    /** If enabled (true), the constant `twist` field is ignored and the precise twist trajectory
     *  is retrieved from the LocalVelocityBuffer from the ParameterSource attached to this block.
     */
    MotionCompensationMethod method = MotionCompensationMethod::Linear;

    /** The velocity (linear and angular) of the vehicle in the local
     * vehicle frame. See FilterDeskew::initialize for an example of how
     * to define it via dynamic variables.
     * This will be used only for `method=MotionCompensationMethod::Linear`; otherwise, it can be
     * left as an empty `std::optional`.
     */
    std::optional<mrpt::math::TTwist3D> twist;
};

/** @} */

}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::MotionCompensationMethod)
MRPT_FILL_ENUM(MotionCompensationMethod::None);
MRPT_FILL_ENUM(MotionCompensationMethod::Linear);
MRPT_FILL_ENUM(MotionCompensationMethod::IMU);
MRPT_FILL_ENUM(MotionCompensationMethod::IMUh);
MRPT_FILL_ENUM(MotionCompensationMethod::IMUt);
MRPT_ENUM_TYPE_END()
