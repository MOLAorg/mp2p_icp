/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/LocalVelocityBuffer.h>

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

mrpt::poses::CPose3DInterpolator
    LocalVelocityBuffer::build_interpolated_poses_around_reference_time() const
{
    mrpt::poses::CPose3DInterpolator trajectory;

    // xx;

    return trajectory;
}
