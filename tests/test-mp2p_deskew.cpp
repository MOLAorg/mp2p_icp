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
 * @file   test-mp2p_deskew.cpp
 * @brief  Unit tests for matcher
 * @author Jose Luis Blanco Claraco
 * @date   Oct 10, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDeskew.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <random>

namespace
{
using mrpt::literals::operator""_deg;

constexpr std::size_t NUM_POINTS   = 10;
constexpr double      LINEAR_SPEED = 1.0;
constexpr double      ANGULAR_VEL  = 90.0_deg;
constexpr double      SCAN_PERIOD  = 0.10;  // seconds

std::vector<mrpt::math::TPoint3D> create_gt_points()
{
    std::vector<mrpt::math::TPoint3D> pts;
    pts.reserve(NUM_POINTS);
    std::mt19937                           rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    for (size_t i = 0; i < NUM_POINTS; ++i)
    {
        pts.emplace_back(dist(rng), dist(rng), dist(rng));
    }
    return pts;
}

std::tuple<
    mrpt::poses::CPose3DInterpolator, std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>,
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>>
    create_gt_keyframes()
{
    mrpt::poses::CPose3DInterpolator                        kfs;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D> kfTwists;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D> imuReadings;

    // t=0: stopped
    kfs.insert(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg));
    kfTwists[mrpt::Clock::fromDouble(0.0)] = {0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg};

    // t=0.1: stopped
    kfs.insert(
        mrpt::Clock::fromDouble(0.1),
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg));
    kfTwists[mrpt::Clock::fromDouble(0.1)] = {0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg};

    // t=0.2: start moving in circles

    kfs.insert(
        mrpt::Clock::fromDouble(0.2),
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg));
    kfTwists[mrpt::Clock::fromDouble(0.2)] = {LINEAR_SPEED, 0, 0, 0.0_deg, 0.0_deg, ANGULAR_VEL};

    for (int i = 0; i < 40; i++)
    {
        const double t      = 0.3 + 0.1 * i;
        const auto   stamp  = mrpt::Clock::fromDouble(t);
        const double dt     = t - 0.2;
        const double radius = LINEAR_SPEED / ANGULAR_VEL;
        const double theta  = ANGULAR_VEL * dt;

        // Center of rotation at (0, radius, 0)
        const double x = radius * std::sin(theta);
        const double y = radius * (1 - std::cos(theta));
        const double z = 0.0;

        kfs.insert(
            stamp, mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, z, 0.0_deg, 0.0_deg, theta));
        kfTwists[stamp] = {LINEAR_SPEED, 0, 0, 0.0_deg, 0.0_deg, ANGULAR_VEL};
    }

    // Fill imuReadings at 100 Hz (every 0.01s) over the keyframe time range
    const double t_start = mrpt::Clock::toDouble(kfs.begin()->first);
    const double t_end   = mrpt::Clock::toDouble(kfs.rbegin()->first);

    for (double t = t_start; t <= t_end + 1e-8; t += 0.01)
    {
        mrpt::Clock::time_point stamp = mrpt::Clock::fromDouble(t);

        // Find the latest keyframe not after t
        auto it = kfTwists.upper_bound(stamp);
        if (it == kfTwists.begin())
        {
            imuReadings[stamp] = mrpt::math::TTwist3D();  // Default zero twist
        }
        else
        {
            --it;
            imuReadings[stamp] = it->second;
        }
    }

    return {kfs, kfTwists, imuReadings};
}

mrpt::maps::CPointsMapXYZIRT::Ptr simulate_skewed_points(
    const mrpt::Clock::time_point& stamp, const mrpt::poses::CPose3DInterpolator& gtKeyframes,
    const std::vector<mrpt::math::TPoint3D>& gtPoints, const double lidar_scan_period)
{
    auto pts = mrpt::maps::CPointsMapXYZIRT::Create();

    // Simulate a scan where each point is acquired at a different time during the scan period.
    // Assume points are acquired sequentially over [stamp, stamp + lidar_scan_period)
    for (std::size_t i = 0; i < gtPoints.size(); ++i)
    {
        // Time offset for this point in the scan:
        const double rel_time = lidar_scan_period * static_cast<double>(i) /
                                static_cast<double>(std::max<std::size_t>(1U, gtPoints.size() - 1));
        const auto pt_stamp = mrpt::Clock::fromDouble(mrpt::Clock::toDouble(stamp) + rel_time);

        // Interpolate pose at this time:
        mrpt::poses::CPose3D pose;
        bool                 interpolation_ok;
        gtKeyframes.interpolate(pt_stamp, pose, interpolation_ok);
        if (!interpolation_ok)
        {
            THROW_EXCEPTION("Error interpolating trajectory!");
        }

        // Transform the GT point to the sensor frame at this time:
        const auto& pt       = gtPoints[i];
        const auto  pt_local = pose.inverseComposePoint(pt).cast<float>();

        // Add to map, with time offset
        pts->insertPointFast(pt_local.x, pt_local.y, pt_local.z);
        pts->insertPointField_Timestamp(static_cast<float>(rel_time));

#if 0
        std::cout << "PT[" << i << "] x: " << pt_local.x << ", y: " << pt_local.y
                  << ", z: " << pt_local.z << ", t: " << rel_time << "\n";
#endif
    }

    return pts;
}

mrpt::maps::CSimplePointsMap simulate_gt_local_points(
    const mrpt::poses::CPose3D& pose, const std::vector<mrpt::math::TPoint3D>& gtPoints)
{
    mrpt::maps::CSimplePointsMap pts;
    for (const auto& pt : gtPoints)
    {
        // Transform the GT point to the local sensor frame at this pose
        const auto pt_local = pose.inverseComposePoint(pt).cast<float>();
        pts.insertPointFast(pt_local.x, pt_local.y, pt_local.z);
    }
    return pts;
}

void run_deskew_test()
{
    // Generate test data:
    const auto gtPoints = create_gt_points();

    const auto [gtKeyframes, gtTwist, imuReadings] = create_gt_keyframes();

#if 0
    for (const auto& [stamp, pose] : gtKeyframes)
    {
        std::cout << "Stamp: " << mrpt::Clock::toDouble(stamp) << " | Pose: " << pose.asString()
                  << " | Twist: " << gtTwist.at(stamp).asString() << "\n";
    }
#endif

    // Create deskew method:
    mp2p_icp::ParameterSource ps;

    mp2p_icp_filters::FilterDeskew deskew;
    deskew.silently_ignore_no_timestamps = false;
    deskew.input_pointcloud_layer        = "raw";
    deskew.output_pointcloud_layer       = "deskewed";
    deskew.method                        = mp2p_icp_filters::MotionCompensationMethod::Linear;

    deskew.attachToParameterSource(ps);

    // Run test:
    const auto  numKfs = gtKeyframes.size();
    std::size_t kfIdx  = 0;
    for (const auto& [stamp, pose] : gtKeyframes)
    {
        if (kfIdx == numKfs - 1)
        {
            // We cannot interpolate past the last one
            break;
        }

        const auto kfGtTwist = gtTwist.at(stamp);

        // Simulate the skewed points:
        const auto skewedPoints = simulate_skewed_points(stamp, gtKeyframes, gtPoints, SCAN_PERIOD);

        // Get the GT deskewed points:
        const auto gtLocalPoints = simulate_gt_local_points(mrpt::poses::CPose3D(pose), gtPoints);

        // Update deskew params (needed for Linear method only):
        deskew.twist = kfGtTwist;

        // Update local velocity buffer:
        ps.localVelocityBuffer.add_orientation(
            mrpt::Clock::toDouble(stamp), pose.getRotationMatrix());
        ps.localVelocityBuffer.add_linear_velocity(
            mrpt::Clock::toDouble(stamp), {kfGtTwist.vx, kfGtTwist.vy, kfGtTwist.vz});

        // For all IMU readings from "stamp" to "stamp+SCAN_PERIOD", add IMU readings:
        {
            const double t_start = mrpt::Clock::toDouble(stamp);
            const double t_end   = t_start + SCAN_PERIOD;
            for (double t = t_start; t <= t_end + 1e-8; t += 0.01)
            {
                mrpt::Clock::time_point imu_stamp = mrpt::Clock::fromDouble(t);
                // Get IMU reading at this time:
                auto it = imuReadings.find(imu_stamp);
                if (it == imuReadings.end())
                {
                    continue;
                }

                const auto& twist = it->second;
                // Add linear acceleration (assume zero for this test):
                ps.localVelocityBuffer.add_linear_acceleration(t, {0, 0, 0});
                // Add angular velocity:
                ps.localVelocityBuffer.add_angular_velocity(t, {twist.wx, twist.wy, twist.wz});
            }
        }

        // Run Deskew filter:
        mp2p_icp::metric_map_t m;
        m.layers["raw"] = skewedPoints;
        deskew.filter(m);

        // Compare points:

        ++kfIdx;
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_deskew_test();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
