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

LocalVelocityBuffer::Trajectory LocalVelocityBuffer::reconstruct_poses_around_reference_time(
    double half_time_span) const
{
    // Recall: In the returned trajectory, t=0 is the reference time
    Trajectory trajectory;

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

    constexpr double INTERPOLATION_TIME_STEP_SEC = 0.1e-3;

    // 1/2: Forward integration
    auto   it         = it_ref;
    double prev_stamp = it->first;
    while (prev_stamp - ref_time < half_time_span)
    {
        if (it == angular_velocities_.end())
        {
            break;
        }
        const auto& w = it->second;
        ++it;
        const double this_abs_stamp = it->first;

        // Move forward in time in small interpolating steps:
        for (;;)
        {
            const double dt = std::min(this_abs_stamp - prev_stamp, INTERPOLATION_TIME_STEP_SEC);
            prev_stamp += dt;
            const double this_rel_stamp = prev_stamp - ref_time;

            if (std::abs(dt) < 1e-3 * INTERPOLATION_TIME_STEP_SEC)
            {
                break;
            }
            // Integrate:
            const auto R = mrpt::poses::Lie::SO<3>::exp(
                (w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>());
            pose.setRotationMatrix(pose.getRotationMatrix() * R);
            trajectory[this_rel_stamp] = pose;
        }
    }

    // 2/2: Backward integration
    pose       = mrpt::poses::CPose3D::Identity();
    it         = it_ref;
    prev_stamp = it->first;
    while (prev_stamp - ref_time > -half_time_span)
    {
        if (it == angular_velocities_.end() || it == angular_velocities_.begin())
        {
            break;
        }
        const auto& w = it->second;
        --it;
        const double this_abs_stamp = it->first;

        // NOTE: dt<0, so there is nothing else special to care about while integrating this
        //       backwards in time.
        // Move forward in time in small interpolating steps:
        for (;;)
        {
            const double dt =
                -std::min(std::abs(this_abs_stamp - prev_stamp), INTERPOLATION_TIME_STEP_SEC);
            prev_stamp += dt;
            const double this_rel_stamp = prev_stamp - ref_time;

            if (std::abs(dt) < 1e-3 * INTERPOLATION_TIME_STEP_SEC)
            {
                break;
            }
            // Integrate:
            const auto R = mrpt::poses::Lie::SO<3>::exp(
                (w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>());
            pose.setRotationMatrix(pose.getRotationMatrix() * R);
            trajectory[this_rel_stamp] = pose;
        }
    }

    return trajectory;
}

mrpt::containers::yaml LocalVelocityBuffer::toYAML() const
{
    mrpt::containers::yaml root = mrpt::containers::yaml::Map();

    {
        mrpt::containers::yaml yamlParams    = mrpt::containers::yaml::Map();
        yamlParams["max_time_window"]        = parameters.max_time_window;
        yamlParams["tolerance_search_stamp"] = parameters.tolerance_search_stamp;

        root["parameters"] = yamlParams;
    }

    {
        mrpt::containers::yaml yamlState = mrpt::containers::yaml::Map();
        yamlState["reference_zero_time"] = reference_zero_time;
        if (!linear_velocities_.empty())
        {
            yamlState["linear_velocities"] = mrpt::containers::yaml::Map();
            for (const auto& [time, vel] : linear_velocities_)
            {
                yamlState["linear_velocities"][mrpt::format("%.09lf", time)] =
                    "'" + vel.asString() + "'";
            }
        }
        if (!angular_velocities_.empty())
        {
            yamlState["angular_velocities"] = mrpt::containers::yaml::Map();
            for (const auto& [time, w] : angular_velocities_)
            {
                yamlState["angular_velocities"][mrpt::format("%.09lf", time)] =
                    "'" + w.asString() + "'";
            }
        }
        root["state"] = yamlState;
    }
    return root;
}

void LocalVelocityBuffer::fromYAML(const mrpt::containers::yaml& y)
{
    if (!y.isMap())
    {
        throw std::runtime_error("Invalid YAML format for LocalVelocityBuffer");
    }

    if (y.has("parameters"))
    {
        const auto& params                = y["parameters"];
        parameters.max_time_window        = params["max_time_window"].as<double>();
        parameters.tolerance_search_stamp = params["tolerance_search_stamp"].as<double>();
    }

    if (y.has("state"))
    {
        const auto& state   = y["state"];
        reference_zero_time = state["reference_zero_time"].as<TimeStamp>();

        linear_velocities_.clear();
        if (state.has("linear_velocities"))
        {
            for (const auto& [time_str, vel_str] : state["linear_velocities"].asMapRange())
            {
                linear_velocities_[std::stod(time_str.as<std::string>())] =
                    mrpt::math::TVector3D::FromString(vel_str.as<std::string>());
            }
        }

        angular_velocities_.clear();
        if (state.has("angular_velocities"))
        {
            for (const auto& [time_str, w_str] : state["angular_velocities"].asMapRange())
            {
                angular_velocities_[std::stod(time_str.as<std::string>())] =
                    mrpt::math::TVector3D::FromString(w_str.as<std::string>());
            }
        }
    }
}
