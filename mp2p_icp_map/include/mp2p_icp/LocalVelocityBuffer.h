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
#include <optional>

namespace mp2p_icp
{
/** Holds a short window of local velocities, accelerations, and global orientation,
 * from external estimators and an IMU.
 *
 * This can be used to reconstruct the trajectory of the sensor at each point in time for precise
 * scan deformation/deskew. This task is done externally to this class (see mp2p_icp::FilterDeskew).
 * This class is also agnostic regarding IMU biases.
 *
 * Its state can be (de)serialized to/from YAML.
 *
 * \ingroup mp2p_icp_map_grp
 */
class LocalVelocityBuffer
{
   public:
    LocalVelocityBuffer() = default;

    using TimeStamp          = double;  // seconds in UNIX epoch
    using SO3                = mrpt::math::CMatrixDouble33;
    using LinearVelocity     = mrpt::math::TVector3D;
    using LinearAcceleration = mrpt::math::TVector3D;
    using AngularVelocity    = mrpt::math::TVector3D;

    struct Parameters
    {
        Parameters() = default;

        double max_time_window        = 1.5;  // seconds
        double tolerance_search_stamp = 10e-3;  // seconds
    };

    Parameters parameters;

    struct Sample
    {
        Sample() = default;

        std::optional<SO3>                q;
        std::optional<LinearVelocity>     v_b;  //!< linear velocity (body frame)
        std::optional<LinearAcceleration> a_b;  //!< linear acceleration (body frame)
        std::optional<AngularVelocity>    w_b;  //!< Angular velocity (body frame)
    };

    /// Each kind of sample on its own timeline.
    struct SamplesByTime
    {
        std::map<TimeStamp, SO3>                q;  //!< orientations (gravity-aligned, global)
        std::map<TimeStamp, LinearVelocity>     v_b;  //!< linear velocity (body frame)
        std::map<TimeStamp, LinearAcceleration> a_b;  //!< proper linear acceleration (body frame)
        std::map<TimeStamp, AngularVelocity>    w_b;  //!< Angular velocity (body frame)
    };

    struct SampleHistory
    {
        SampleHistory() = default;

        /// All samples, sorted by time. Each entry may have one type of observation or another.
        std::map<TimeStamp, Sample> by_time;

        /// Each kind of sample on its own timeline.
        SamplesByTime by_type;
    };

    /// Add a new global (gravity-aligned) orientation entry
    void add_orientation(const TimeStamp& time, const SO3& attitude);

    /// Add a new linear velocity entry (in the vehicle local frame of reference)
    void add_linear_velocity(const TimeStamp& time, const LinearVelocity& v_vehicle);

    /// Add a new linear acceleration entry (in the vehicle local frame of reference)
    void add_linear_acceleration(const TimeStamp& time, const LinearAcceleration& a_vehicle);

    /// Add a new angular velocity entry (in the vehicle local frame of reference)
    void add_angular_velocity(const TimeStamp& time, const AngularVelocity& w_vehicle);

    /// Stores the current state and parameters to a YAML dictionary.
    mrpt::containers::yaml toYAML() const;

    /// Restores the current state and parameters from a YAML dictionary.
    void fromYAML(const mrpt::containers::yaml& y);

    /// Get the current linear velocities map (in the vehicle frame of reference)
    const auto& get_linear_velocities() const { return linear_velocities_; }

    /// Get the current angular velocities map (in the vehicle frame of reference)
    const auto& get_angular_velocities() const { return angular_velocities_; }

    /// Get the current linear accelerations map (in the vehicle frame of reference)
    const auto& get_linear_accelerations() const { return linear_accelerations_; }

    /// Get the current orientations map (in the gravity-aligned global frame of reference)
    const auto& get_orientations() const { return orientations_; }

    /// Set the reference time for lidar scans:
    void set_reference_zero_time(const TimeStamp& time) { reference_zero_time = time; }

    /// Get the reference time for lidar scans:
    TimeStamp get_reference_zero_time() const { return reference_zero_time; }

    /** Collects the samples within the time range `ref_time ± half_time_span`.
     *
     * In the returned trajectory, t=0 is the reference time.
     * \return The generated series, or an empty one if there is no enough data to build it.
     */
    SampleHistory collect_samples_around_reference_time(double half_time_span) const;

    /// reset the buffer, clearing all entries
    void clear() { *this = {}; }

   private:
    std::map<TimeStamp, LinearVelocity>     linear_velocities_;  // in the vehicle frame
    std::map<TimeStamp, AngularVelocity>    angular_velocities_;  // in the vehicle frame
    std::map<TimeStamp, LinearAcceleration> linear_accelerations_;  // in the vehicle frame
    std::map<TimeStamp, SO3>                orientations_;  // in the global, gravity-aligned frame

    TimeStamp reference_zero_time = 0.0;  //!< Reference time for each lidar scan

    void delete_too_old_entries(const TimeStamp& now);
};

// So we can build libraries downstream in backward compatible way:
#define MP2P_ICP_HAS_VELOCITY_BUFFER 1

}  // namespace mp2p_icp
