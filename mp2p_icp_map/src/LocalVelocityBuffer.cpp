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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mp2p_icp;

void LocalVelocityBuffer::add_linear_velocity(
    const TimeStamp& time, const LinearVelocity& v_vehicle)
{
    linear_velocities_[time] = v_vehicle;
    delete_too_old_entries(time);
}

void LocalVelocityBuffer::add_linear_acceleration(
    const TimeStamp& time, const LinearAcceleration& a_vehicle)
{
    linear_accelerations_[time] = a_vehicle;
    delete_too_old_entries(time);
}

void LocalVelocityBuffer::add_angular_velocity(
    const TimeStamp& time, const AngularVelocity& w_vehicle)
{
    angular_velocities_[time] = w_vehicle;
    delete_too_old_entries(time);
}

void LocalVelocityBuffer::add_orientation(const TimeStamp& time, const SO3& attitude)
{
    orientations_[time] = attitude;
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

auto LocalVelocityBuffer::collect_samples_around_reference_time(double half_time_span) const
    -> LocalVelocityBuffer::SampleHistory
{
    SampleHistory result;

    const double t0 = reference_zero_time - half_time_span;
    const double t1 = reference_zero_time + half_time_span;

    auto collect_from_map = [&](const auto& srcMap, auto Sample::*field)
    {
        for (const auto& [ts, val] : srcMap)
        {
            if (ts < t0 || ts > t1)
            {
                continue;
            }

            const double rel_t  = ts - reference_zero_time;
            auto&        sample = result[rel_t];
            sample.*field       = val;
        }
    };

    collect_from_map(linear_velocities_, &Sample::v_b);
    collect_from_map(angular_velocities_, &Sample::w_b);
    collect_from_map(linear_accelerations_, &Sample::a_b);
    collect_from_map(orientations_, &Sample::orientation);

    return result;
}
mrpt::containers::yaml LocalVelocityBuffer::toYAML() const
{
    mrpt::containers::yaml root = mrpt::containers::yaml::Map();

    // Parameters
    {
        mrpt::containers::yaml yamlParams    = mrpt::containers::yaml::Map();
        yamlParams["max_time_window"]        = parameters.max_time_window;
        yamlParams["tolerance_search_stamp"] = parameters.tolerance_search_stamp;

        root["parameters"] = yamlParams;
    }

    // State
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

        if (!linear_accelerations_.empty())
        {
            yamlState["linear_accelerations"] = mrpt::containers::yaml::Map();
            for (const auto& [time, acc] : linear_accelerations_)
            {
                yamlState["linear_accelerations"][mrpt::format("%.09lf", time)] =
                    "'" + acc.asString() + "'";
            }
        }

        if (!orientations_.empty())
        {
            yamlState["orientations"] = mrpt::containers::yaml::Map();
            for (const auto& [time, R] : orientations_)
            {
                yamlState["orientations"][mrpt::format("%.09lf", time)] =
                    "'" + R.inMatlabFormat() + "'";
            }
        }

        root["state"] = yamlState;
    }

    return root;
}

namespace
{

static std::string stripQuotes(const std::string& s)
{
    if (s.size() >= 2)
    {
        char first = s.front();
        char last  = s.back();
        if ((first == '\'' && last == '\'') || (first == '"' && last == '"'))
        {
            return s.substr(1, s.size() - 2);
        }
    }
    return s;
}

}  // namespace

void LocalVelocityBuffer::fromYAML(const mrpt::containers::yaml& y)
{
    if (!y.isMap())
    {
        throw std::runtime_error("Invalid YAML format for LocalVelocityBuffer");
    }

    // Parameters
    if (y.has("parameters"))
    {
        const auto& params                = y["parameters"];
        parameters.max_time_window        = params["max_time_window"].as<double>();
        parameters.tolerance_search_stamp = params["tolerance_search_stamp"].as<double>();
    }

    // State
    if (y.has("state"))
    {
        const auto& state   = y["state"];
        reference_zero_time = state["reference_zero_time"].as<TimeStamp>();

        // Linear velocities
        linear_velocities_.clear();
        if (state.has("linear_velocities"))
        {
            for (const auto& [time_str, vel_str] : state["linear_velocities"].asMapRange())
            {
                linear_velocities_[std::stod(time_str.as<std::string>())] =
                    mrpt::math::TVector3D::FromString(stripQuotes(vel_str.as<std::string>()));
            }
        }

        // Angular velocities
        angular_velocities_.clear();
        if (state.has("angular_velocities"))
        {
            for (const auto& [time_str, w_str] : state["angular_velocities"].asMapRange())
            {
                angular_velocities_[std::stod(time_str.as<std::string>())] =
                    mrpt::math::TVector3D::FromString(stripQuotes(w_str.as<std::string>()));
            }
        }

        // Linear accelerations
        linear_accelerations_.clear();
        if (state.has("linear_accelerations"))
        {
            for (const auto& [time_str, acc_str] : state["linear_accelerations"].asMapRange())
            {
                linear_accelerations_[std::stod(time_str.as<std::string>())] =
                    mrpt::math::TVector3D::FromString(stripQuotes(acc_str.as<std::string>()));
            }
        }

        // Orientations
        orientations_.clear();
        if (state.has("orientations"))
        {
            for (const auto& [time_str, R_str] : state["orientations"].asMapRange())
            {
                mrpt::math::CMatrixDouble33 R;
                R.fromMatlabStringFormat(stripQuotes(R_str.as<std::string>()));
                orientations_[std::stod(time_str.as<std::string>())] = R;
            }
        }
    }
}
