/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   GeometryClassWeights.cpp
 * @brief  Extra per-pairing weight as a function of the map surface geometry.
 * @author Jose Luis Blanco Claraco
 * @date   Aug 21, 2026
 */

#include <mp2p_icp/GeometryClassWeights.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

#include <algorithm>
#include <cmath>

using namespace mp2p_icp;

namespace
{
/// Cubic smoothstep, clamped outside [0,1].
double smoothstep(double t)
{
    if (t <= 0)
    {
        return 0.0;
    }
    if (t >= 1)
    {
        return 1.0;
    }
    return t * t * (3.0 - 2.0 * t);
}

/// Fraction of `x` that has crossed the edge at `b`, ramping over `s`.
double crossed(double x, double b, double s)
{
    if (s <= 0)
    {
        return x < b ? 0.0 : 1.0;
    }
    return smoothstep((x - b) / s + 0.5);
}

std::string to_string(GeometryClassWeights::Variable v)
{
    return v == GeometryClassWeights::Variable::Incidence ? "incidence" : "verticality";
}

std::string to_string(GeometryClassWeights::GravitySource g)
{
    return g == GeometryClassWeights::GravitySource::Imu ? "imu" : "map_frame";
}
}  // namespace

void GeometryClassWeights::load_from(const mrpt::containers::yaml& p)
{
    // An absent block leaves the defaults, i.e. disabled.
    if (p.empty())
    {
        return;
    }

    if (p.has("enabled"))
    {
        enabled = p["enabled"].as<bool>();
    }
    if (p.has("variable"))
    {
        const auto s = p["variable"].as<std::string>();
        if (s == "incidence")
        {
            variable = Variable::Incidence;
        }
        else if (s == "verticality")
        {
            variable = Variable::Verticality;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "geometry_class_weights: unknown variable '%s' (expected 'incidence' or "
                "'verticality')",
                s.c_str());
        }
    }
    if (p.has("gravity_source"))
    {
        const auto s = p["gravity_source"].as<std::string>();
        if (s == "imu")
        {
            gravitySource = GravitySource::Imu;
        }
        else if (s == "map_frame")
        {
            gravitySource = GravitySource::MapFrame;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "geometry_class_weights: unknown gravity_source '%s' (expected 'imu' or "
                "'map_frame')",
                s.c_str());
        }
    }
    if (p.has("softness"))
    {
        softness = p["softness"].as<double>();
    }

    const auto readVector = [&](const char* key, std::vector<double>& dst)
    {
        if (!p.has(key))
        {
            return;
        }
        dst.clear();
        for (const auto& v : p[key].asSequence())
        {
            dst.push_back(v.as<double>());
        }
    };
    readVector("breakpoints", breakpoints);
    readVector("weights", weights);

    validate();
}

void GeometryClassWeights::save_to(mrpt::containers::yaml& p) const
{
    p["enabled"]        = enabled;
    p["variable"]       = to_string(variable);
    p["gravity_source"] = to_string(gravitySource);
    p["softness"]       = softness;

    p["breakpoints"] = mrpt::containers::yaml::Sequence();
    for (const auto v : breakpoints)
    {
        p["breakpoints"].asSequence().push_back(v);
    }
    p["weights"] = mrpt::containers::yaml::Sequence();
    for (const auto v : weights)
    {
        p["weights"].asSequence().push_back(v);
    }
}

void GeometryClassWeights::validate() const
{
    if (!enabled)
    {
        return;
    }

    ASSERTMSG_(
        weights.size() == breakpoints.size() + 1,
        mrpt::format(
            "geometry_class_weights: 'weights' must have one more entry than "
            "'breakpoints' (%zu breakpoints, %zu weights)",
            breakpoints.size(), weights.size()));
    ASSERTMSG_(!weights.empty(), "geometry_class_weights: 'weights' cannot be empty");

    for (const auto w : weights)
    {
        ASSERTMSG_(w > 0, "geometry_class_weights: every weight must be strictly positive");
    }

    ASSERT_(softness >= 0);

    // Breakpoints closer together than the ramp width would make the memberships
    // overlap, and a "partition of unity" that does not sum to one is a silently
    // wrong weight rather than a loud error.
    for (size_t i = 0; i + 1 < breakpoints.size(); i++)
    {
        ASSERTMSG_(
            breakpoints[i + 1] - breakpoints[i] >= softness,
            mrpt::format(
                "geometry_class_weights: breakpoints must be ascending and at least "
                "'softness' (%.3f) apart; got %.3f and %.3f",
                softness, breakpoints[i], breakpoints[i + 1]));
    }
}

bool GeometryClassWeights::active() const
{
    if (!enabled || weights.empty())
    {
        return false;
    }
    // The identity must cost nothing and change nothing, so that "all weights
    // 1.0" and "enabled: false" are the same run.
    return std::any_of(weights.begin(), weights.end(), [](double w) { return w != 1.0; });
}

void GeometryClassWeights::memberships(double x, std::vector<double>& out) const
{
    out.assign(weights.size(), 0.0);
    if (out.empty())
    {
        return;
    }
    // m_0 = 1 - S_1, m_k = S_k - S_{k+1}, m_n = S_n.
    double prev = 1.0;
    for (size_t k = 0; k < breakpoints.size(); k++)
    {
        const double s = crossed(x, breakpoints[k], softness);
        out[k]         = prev - s;
        prev           = s;
    }
    out.back() = prev;
}

double GeometryClassWeights::variable_value(
    const Eigen::Vector3d& normal, const mrpt::math::TPoint3Df& local,
    const mrpt::poses::CPose3D& pose, const std::optional<Eigen::Vector3d>& upInMap) const
{
    if (variable == Variable::Verticality)
    {
        ASSERTMSG_(
            upInMap.has_value(),
            "geometry_class_weights: variable 'verticality' needs a gravity "
            "direction, and none was available. Provide a gravity observation to "
            "the solver, or set gravity_source: map_frame to accept the map "
            "frame's own +Z (which drifts with the attitude estimate).");
        return std::abs(normal.dot(upInMap->normalized()));
    }

    // Incidence. The normal is in the map frame and the scan point in the
    // vehicle frame, so the ray must be rotated into the map frame before the
    // two are contracted; skipping that gives incidence mixed with vehicle yaw.
    const Eigen::Vector3d lv(local.x, local.y, local.z);
    const double          r = lv.norm();
    if (r < 1e-6)
    {
        return 0.0;
    }
    const Eigen::Vector3d ray = (pose.getRotationMatrix().asEigen() * lv).normalized();
    return mrpt::RAD2DEG(std::acos(std::min(1.0, std::abs(normal.dot(ray)))));
}

double GeometryClassWeights::operator()(
    const Eigen::Vector3d& normal, const mrpt::math::TPoint3Df& local,
    const mrpt::poses::CPose3D& pose, const std::optional<Eigen::Vector3d>& upInMap) const
{
    if (!active())
    {
        return 1.0;
    }
    const double x = variable_value(normal, local, pose, upInMap);

    if (softness <= 0)
    {
        // Hard bins: kept so the smooth version can be A/B'd against the
        // measurements that were taken with hard bins.
        size_t k = 0;
        while (k < breakpoints.size() && x >= breakpoints[k])
        {
            k++;
        }
        return weights[k];
    }

    double       lambda = 0;
    double       prev   = 1.0;
    const size_t n      = breakpoints.size();
    for (size_t k = 0; k < n; k++)
    {
        const double s = crossed(x, breakpoints[k], softness);
        lambda += weights[k] * (prev - s);
        prev = s;
    }
    lambda += weights[n] * prev;
    return lambda;
}
