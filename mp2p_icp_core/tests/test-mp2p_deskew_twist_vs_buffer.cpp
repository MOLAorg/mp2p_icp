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
 * @file   test-mp2p_deskew_twist_vs_buffer.cpp
 * @brief  FilterDeskew with IMU-based methods: `use_twist_with_imu_methods`
 *         selects between the `twist` velocity (applied at t=0) and the
 *         LocalVelocityBuffer velocity sample (at its own stamp).
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDeskew.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
constexpr double      REF_TIME    = 100.0;  // [s] scan reference time
constexpr double      SCAN_PERIOD = 0.1;  // [s]
constexpr double      GRAVITY     = 9.81;
constexpr std::size_t N_POINTS    = 21;

// Points spread along y, with times centered at the reference time:
mrpt::maps::CGenericPointsMap::Ptr make_scan()
{
    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->registerField_float(mp2p_icp_filters::POINT_FIELD_TIMESTAMP);

    for (std::size_t i = 0; i < N_POINTS; ++i)
    {
        const double t = -0.5 * SCAN_PERIOD +
                         SCAN_PERIOD * static_cast<double>(i) / static_cast<double>(N_POINTS - 1);
        pts->insertPointFast(10.0f, -5.0f + 0.5f * static_cast<float>(i), 1.0f);
        pts->insertPointField_float(mp2p_icp_filters::POINT_FIELD_TIMESTAMP, static_cast<float>(t));
    }
    return pts;
}

struct Case
{
    std::string name;
    bool        use_twist = true;
    double      twist_vx  = 0;  // [m/s]
    double      buffer_vx = 0;  // [m/s] velocity sample in the buffer
    double      buffer_vt = 0;  // [s] its stamp, relative to REF_TIME
    double      ax        = 0;  // [m/s^2] constant coordinate acceleration along +x
    double      vx0       = 0;  // [m/s] expected velocity at t=0
    double      tol       = 0;  // [m]
};

// A vehicle with a fixed attitude, accelerating along +x: the IMU measures
// the specific force (ax, 0, g).
void fill_buffer(mp2p_icp::ParameterSource& ps, const Case& c)
{
    auto& buf = ps.localVelocityBuffer;
    for (int k = -40; k <= 40; k++)
    {
        const double t = REF_TIME + 0.005 * k;
        buf.add_angular_velocity(t, {0, 0, 0});
        buf.add_linear_acceleration(t, {c.ax, 0, GRAVITY});
    }
    buf.add_orientation(REF_TIME - 0.05, mrpt::math::CMatrixDouble33::Identity());
    buf.add_linear_velocity(REF_TIME + c.buffer_vt, {c.buffer_vx, 0, 0});
    buf.set_reference_zero_time(REF_TIME);
}

// Returns the largest error of the deskewed points w.r.t. the motion
// x(t) - x(0) = vx0 * t + ax * t^2 / 2, or a negative value on failure.
double run_case(const Case& c)
{
    using namespace mp2p_icp_filters;

    mp2p_icp::ParameterSource ps;
    fill_buffer(ps, c);

    FilterDeskew deskew;
    deskew.input_pointcloud_layer     = "raw";
    deskew.output_pointcloud_layer    = "deskewed";
    deskew.method                     = MotionCompensationMethod::IMU;
    deskew.twist                      = mrpt::math::TTwist3D(c.twist_vx, 0, 0, 0, 0, 0);
    deskew.use_twist_with_imu_methods = c.use_twist;
    deskew.attachToParameterSource(ps);

    const auto  scan = make_scan();
    const auto* Ts   = scan->getPointsBufferRef_float_field(POINT_FIELD_TIMESTAMP);

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = scan;
    deskew.filter(m);

    const auto out = m.point_layer("deskewed");
    if (!out || out->size() != N_POINTS)
    {
        return -1;
    }

    double max_err = 0;
    for (std::size_t i = 0; i < N_POINTS; ++i)
    {
        mrpt::math::TPoint3Df in;
        mrpt::math::TPoint3Df p;
        scan->getPoint(i, in.x, in.y, in.z);
        out->getPoint(i, p.x, p.y, p.z);

        const double t          = (*Ts)[i];
        const double expected_x = in.x + c.vx0 * t + 0.5 * c.ax * t * t;
        max_err                 = std::max(max_err, std::abs(p.x - expected_x));
        max_err =
            std::max(max_err, static_cast<double>(std::abs(p.y - in.y) + std::abs(p.z - in.z)));
    }
    return max_err;
}

bool check_parsing()
{
    mrpt::containers::yaml p;
    p["input_pointcloud_layer"]     = "raw";
    p["output_pointcloud_layer"]    = "deskewed";
    p["method"]                     = "MotionCompensationMethod::IMU";
    p["use_twist_with_imu_methods"] = "false";

    mp2p_icp_filters::FilterDeskew deskew;
    deskew.initialize(p);
    return !deskew.use_twist_with_imu_methods;
}
}  // namespace

int main()
{
    try
    {
        constexpr double ACC = 2.0;  // [m/s^2]

        const Case cases[] = {
            // The (zero) twist replaces the buffer velocity: no motion.
            {"twist at t=0", true, 0.0, 2.0, -0.1, 0.0, 0.0, 1e-4},
            // Buffer velocity at its own stamp, constant velocity.
            {"buffer sample", false, 0.0, 2.0, -0.1, 0.0, 2.0, 1e-4},
            // Buffer velocity 0.1 s before t=0, carried with the accelerometer:
            // v(0) = 2 + ACC * 0.1. Applying it at t=0 instead would be off by
            // 1 cm at the ends of the scan.
            {"buffer sample, accelerating", false, 0.0, 2.0, -0.1, ACC, 2.0 + ACC * 0.1, 2e-3},
            // The only velocity sample is in the buffer but outside the window
            // (twice the scan span around t=0): falls back to the twist.
            {"no sample in window", false, 1.5, 2.0, -0.25, 0.0, 1.5, 1e-4},
        };

        for (const auto& c : cases)
        {
            const double e = run_case(c);
            if (e < 0 || e > c.tol)
            {
                std::cerr << "[FAIL] " << c.name << ": max error " << e << " m\n";
                return 1;
            }
            std::cout << "[OK ] " << c.name << " (max error " << e << " m)\n";
        }

        if (!check_parsing())
        {
            std::cerr << "[FAIL] use_twist_with_imu_methods not parsed\n";
            return 1;
        }
        std::cout << "[OK ] parsing\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[FAIL] Exception: " << e.what() << "\n";
        return 1;
    }
}
