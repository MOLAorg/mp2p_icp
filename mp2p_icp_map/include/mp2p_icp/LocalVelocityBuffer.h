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

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <map>

namespace mp2p_icp
{
/** Holds a short time window of local velocities (vw,vy,vz) and (wx,wy,wz), from external
 * estimators or an IMU. This is used to compute the local velocity of the sensor at each point in
 * time for precise scan deformation/deskew.
 *
 * Its state can be (de)serialized to/from YAML.
 *
 * \ingroup mp2p_icp_map_grp
 */
class LocalVelocityBuffer
{
   public:
    LocalVelocityBuffer() = default;

    using TimeStamp       = double;  // seconds in UNIX epoch
    using LinearVelocity  = mrpt::math::TVector3D;
    using AngularVelocity = mrpt::math::TVector3D;
    using Trajectory      = std::map<double, mrpt::poses::CPose3D>;  // t=0 is the reference time

    struct Parameters
    {
        double max_time_window        = 1.5;  // seconds
        double tolerance_search_stamp = 10e-3;  // seconds
    };

    Parameters parameters;

    /// Add a new linear velocity entry (in the vehicle frame of reference)
    void add_linear_velocity(const TimeStamp& time, const LinearVelocity& v_vehicle);

    /// Add a new angular velocity entry (in the vehicle frame of reference)
    void add_angular_velocity(const TimeStamp& time, const AngularVelocity& w_vehicle);

    /// reset the buffer, clearing all entries
    void clear()
    {
        linear_velocities_.clear();
        angular_velocities_.clear();
    }

    /// Stores the current state and parameters to a YAML dictionary.
    mrpt::containers::yaml toYAML() const;

    /// Restores the current state and parameters from a YAML dictionary.
    void fromYAML(const mrpt::containers::yaml& y);

    /// Get the current linear velocities map (in the vehicle frame of reference)
    const auto& get_linear_velocities() const { return linear_velocities_; }

    /// Get the current angular velocities map (in the vehicle frame of reference)
    const auto& get_angular_velocities() const { return angular_velocities_; }

    /// Set the reference time for lidar scans:
    void set_reference_zero_time(const TimeStamp& time) { reference_zero_time = time; }

    /// Get the reference time for lidar scans:
    TimeStamp get_reference_zero_time() const { return reference_zero_time; }

    /** Build the reconstructed vehicle trajectory in the time range `ref_time ± half_time_span`.
     *
     * The closest velocity entry timestamp to the reference time is searched and used as if it was
     * exactly the reference time.
     * In the returned trajectory, t=0 is the reference time.
     * \return The generated trajectory, or an empty one if there is no enough data to build it.
     */
    Trajectory reconstruct_poses_around_reference_time(double half_time_span) const;

   private:
    std::map<TimeStamp, LinearVelocity>  linear_velocities_;  // in the vehicle frame
    std::map<TimeStamp, AngularVelocity> angular_velocities_;  // in the vehicle frame
    TimeStamp reference_zero_time = 0.0;  //!< Reference time for each lidar scan

    void delete_too_old_entries(const TimeStamp& now);
};

// So we can build libraries downstream in backward compatible way:
#define MP2P_ICP_HAS_VELOCITY_BUFFER 1

}  // namespace mp2p_icp
