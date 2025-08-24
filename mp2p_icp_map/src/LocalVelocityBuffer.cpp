/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/LocalVelocityBuffer.h>
#include <mrpt/containers/find_closest.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mp2p_icp;

void LocalVelocityBuffer::add_linear_velocity(
    const TimeStamp& time, const LinearVelocity& v_vehicle)
{
    linear_velocities_[time] = v_vehicle;
    delete_too_old_entries(time);
}

void LocalVelocityBuffer::add_angular_velocity(
    const TimeStamp& time, const AngularVelocity& w_vehicle)
{
    angular_velocities_[time] = w_vehicle;
    delete_too_old_entries(time);
}

void LocalVelocityBuffer::delete_too_old_entries(const TimeStamp& now)
{
    // Remove entries older than the max time window
    for (auto it = linear_velocities_.begin(); it != linear_velocities_.end();)
    {
        if (now - it->first > parameters.max_time_window)
        {
            it = linear_velocities_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (auto it = angular_velocities_.begin(); it != angular_velocities_.end();)
    {
        if (now - it->first > parameters.max_time_window)
        {
            it = angular_velocities_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

std::map<double, mrpt::poses::CPose3D> LocalVelocityBuffer::reconstruct_poses_around_reference_time(
    double half_time_span) const
{
    // Recall: In the returned trajectory, t=0 is the reference time
    std::map<double, mrpt::poses::CPose3D> trajectory;

    const auto closest_stamp_w = mrpt::containers::find_closest_with_tolerance(
        angular_velocities_, reference_zero_time, parameters.tolerance_search_stamp);

    if (!closest_stamp_w.has_value())
    {
        // We don't have any nearby IMU reading to work with!
        // Return an empty trajectory.
        return {};
    }

    const double ref_time = closest_stamp_w->first;

    auto pose = mrpt::poses::CPose3D::Identity();
    // Should never fail by preconditions above
    const auto it_ref = angular_velocities_.find(ref_time);

    // Insert ref pose (=the identity)
    trajectory[0.0 /*ref_time - ref_time*/] = pose;

    // 1/2: Forward integration
    auto   it         = it_ref;
    double prev_stamp = it->first;
    for (;;)
    {
        if (it == angular_velocities_.end())
        {
            break;
        }
        const auto& w = it->second;
        ++it;
        const double this_abs_stamp = it->first;
        const double dt             = this_abs_stamp - prev_stamp;
        prev_stamp                  = this_abs_stamp;

        const double this_rel_stamp = this_abs_stamp - ref_time;
        if (this_rel_stamp > half_time_span)
        {
            break;
        }
        // Integrate:
        const auto R =
            mrpt::poses::Lie::SO<3>::exp((w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>());
        pose.setRotationMatrix(pose.getRotationMatrix() * R);

        trajectory[this_rel_stamp] = pose;
    }

    // 2/2: Backward integration
    pose       = mrpt::poses::CPose3D::Identity();
    it         = it_ref;
    prev_stamp = it->first;
    for (;;)
    {
        if (it == angular_velocities_.end() || it == angular_velocities_.begin())
        {
            break;
        }
        const auto& w = it->second;
        --it;
        const double this_abs_stamp = it->first;
        const double dt             = this_abs_stamp - prev_stamp;
        prev_stamp                  = this_abs_stamp;

        const double this_rel_stamp = this_abs_stamp - ref_time;
        if (this_rel_stamp < -half_time_span)
        {
            break;
        }
        // Integrate:
        // NOTE: dt<0, so there is nothing else special to care about while integrating this
        //       backwards in time.
        const auto R =
            mrpt::poses::Lie::SO<3>::exp((w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>());
        pose.setRotationMatrix(pose.getRotationMatrix() * R);

        trajectory[this_rel_stamp] = pose;
    }

    return trajectory;
}
