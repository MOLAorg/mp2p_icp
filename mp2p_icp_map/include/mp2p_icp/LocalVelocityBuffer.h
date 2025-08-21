/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <map>

namespace mp2p_icp
{
/** Holds a short time window of local velocities (vw,vy,vz) and (wx,wy,wz), from external
 * estimators or an IMU. This is used to compute the local velocity of the sensor at each point in
 * time for precise scan deformation/deskew.
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

    struct Parameters
    {
        double max_time_window = 1.5;  // seconds
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

    /// Get the current linear velocities map (in the vehicle frame of reference)
    const auto& get_linear_velocities() const { return linear_velocities_; }

    /// Get the current angular velocities map (in the vehicle frame of reference)
    const auto& get_angular_velocities() const { return angular_velocities_; }

    /// Set the reference time for lidar scans:
    void set_reference_zero_time(const TimeStamp& time) { reference_zero_time = time; }

    /// Get the reference time for lidar scans:
    TimeStamp get_reference_zero_time() const { return reference_zero_time; }

    /// Build the trajectory around the currently-set reference zero time:
    mrpt::poses::CPose3DInterpolator build_interpolated_poses_around_reference_time() const;

   private:
    std::map<TimeStamp, LinearVelocity>  linear_velocities_;  // in the vehicle frame
    std::map<TimeStamp, AngularVelocity> angular_velocities_;  // in the vehicle frame
    TimeStamp reference_zero_time = 0.0;  //!< Reference time for each lidar scan

    void delete_too_old_entries(const TimeStamp& now);
};

}  // namespace mp2p_icp
