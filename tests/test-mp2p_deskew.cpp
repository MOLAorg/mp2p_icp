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

struct SimulationParams
{
    std::size_t point_count  = 10;
    double      linear_speed = 1.0;
    double      angular_vel  = 0.001_deg;
    double      scan_period  = 0.10;  // seconds

    mp2p_icp_filters::MotionCompensationMethod deskew_method =
        mp2p_icp_filters::MotionCompensationMethod::None;
};

struct SimulationResult
{
    float              rmse = 0;
    std::vector<float> individual_frame_rmse;
};

std::vector<mrpt::math::TPoint3D> create_gt_points(const SimulationParams& p)
{
    std::vector<mrpt::math::TPoint3D> pts;
    pts.reserve(p.point_count);
    std::mt19937                           rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    for (size_t i = 0; i < p.point_count; ++i)
    {
        pts.emplace_back(dist(rng), dist(rng), dist(rng));
    }
    return pts;
}

std::
    tuple<
        mrpt::poses::CPose3DInterpolator, std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>,
        std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D> /* IMU ang vel */,
        std::map<mrpt::Clock::time_point, mrpt::math::TVector3D> /**acceleration*/
        >
    create_gt_keyframes(const SimulationParams& p)
{
    mrpt::poses::CPose3DInterpolator                         kfs;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>  kfTwists;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>  imuReadings;
    std::map<mrpt::Clock::time_point, mrpt::math::TVector3D> imuReadingsAcc;

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
    kfTwists[mrpt::Clock::fromDouble(0.2)] = {p.linear_speed, 0,       0,
                                              0.0_deg,        0.0_deg, p.angular_vel};

    for (int i = 0; i < 40; i++)
    {
        const double t      = 0.3 + 0.1 * i;
        const auto   stamp  = mrpt::Clock::fromDouble(t);
        const double dt     = t - 0.2;
        const double radius = p.linear_speed / p.angular_vel;
        const double theta  = p.angular_vel * dt;

        // Center of rotation at (0, radius, 0)
        const double x = radius * std::sin(theta);
        const double y = radius * (1 - std::cos(theta));
        const double z = 0.0;

        kfs.insert(
            stamp, mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, z, 0.0_deg, 0.0_deg, theta));
        kfTwists[stamp] = {p.linear_speed, 0, 0, 0.0_deg, 0.0_deg, p.angular_vel};
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
            imuReadings[stamp]    = mrpt::math::TTwist3D();  // Default zero twist
            imuReadingsAcc[stamp] = mrpt::math::TVector3D(0, 0, 9.81);  // Only gravity
        }
        else
        {
            --it;
            imuReadings[stamp] = it->second;

            // Compute linear acceleration in world frame
            // For this simulation, include centripetal acceleration if moving in a circle
            const auto& twist = it->second;
            double      vx    = twist.vx;
            double      wz    = twist.wz;

            mrpt::math::TVector3D acc(0, 0, 9.81);  // gravity

            if (std::abs(wz) > 1e-8)
            {
                // Centripetal acceleration: a_c = v^2 / r = v * w
                // Direction: towards center of rotation (negative y in local frame)
                double a_c = vx * wz;
                acc.y -= a_c;  // subtract because center is at +y
            }
            imuReadingsAcc[stamp] = acc;
        }
    }

    return {kfs, kfTwists, imuReadings, imuReadingsAcc};
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

// Returns: rmse error
[[nodiscard]] SimulationResult run_deskew_test(const SimulationParams& p)
{
    // Generate test data:
    const auto gtPoints = create_gt_points(p);

    const auto [gtKeyframes, gtTwist, imuReadings, imuReadingsAcc] = create_gt_keyframes(p);

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
    deskew.method                        = p.deskew_method;

    deskew.attachToParameterSource(ps);

    // Run test:
    SimulationResult result;

    float       sum_error_sqr = .0f;
    std::size_t error_terms   = 0;
    const auto  numKfs        = gtKeyframes.size();
    std::size_t kfIdx         = 0;
    for (const auto& [stamp, pose] : gtKeyframes)
    {
        if (kfIdx == numKfs - 1)
        {
            // We cannot interpolate past the last one
            break;
        }

        const auto kfGtTwist = gtTwist.at(stamp);

        // Simulate the skewed points:
        const auto skewedPoints =
            simulate_skewed_points(stamp, gtKeyframes, gtPoints, p.scan_period);

        // Get the GT deskewed points:
        const auto gtLocalPoints = simulate_gt_local_points(mrpt::poses::CPose3D(pose), gtPoints);

        // Update deskew params (needed for Linear method only):
        deskew.twist = kfGtTwist;

        // Update local velocity buffer:
        const double stamp_s = mrpt::Clock::toDouble(stamp);
        ps.localVelocityBuffer.add_orientation(stamp_s, pose.getRotationMatrix());
        ps.localVelocityBuffer.add_linear_velocity(
            stamp_s, {kfGtTwist.vx, kfGtTwist.vy, kfGtTwist.vz});

        ps.localVelocityBuffer.set_reference_zero_time(stamp_s);

        // For all IMU readings from "stamp" to "stamp+SCAN_PERIOD", add IMU readings:
        {
            const double t_start = stamp_s;
            const double t_end   = t_start + p.scan_period;

            for (auto it = imuReadings.lower_bound(mrpt::Clock::fromDouble(t_start));
                 it != imuReadings.end(); ++it)
            {
                const double t = mrpt::Clock::toDouble(it->first);
                if (t > t_end + 1e-8)
                {
                    break;
                }

                const auto& twist = it->second;
                // Add linear acceleration:
                ps.localVelocityBuffer.add_linear_acceleration(t, imuReadingsAcc.at(it->first));
                // Add angular velocity:
                ps.localVelocityBuffer.add_angular_velocity(t, {twist.wx, twist.wy, twist.wz});
            }
        }

        // Run Deskew filter:
        mp2p_icp::metric_map_t m;
        m.layers["raw"] = skewedPoints;
        deskew.filter(m);

        float frame_sum_error_sqr = .0f;

        // Compare points:
        const auto deskewedPts = m.point_layer("deskewed");
        ASSERT_EQUAL_(deskewedPts->size(), gtLocalPoints.size());
        for (size_t i = 0; i < deskewedPts->size(); i++)
        {
            mrpt::math::TPoint3Df pt;
            deskewedPts->getPoint(i, pt.x, pt.y, pt.z);
            mrpt::math::TPoint3Df gtPt;
            gtLocalPoints.getPoint(i, gtPt.x, gtPt.y, gtPt.z);

            const auto error = (pt - gtPt).norm();
            sum_error_sqr += mrpt::square(error);
            frame_sum_error_sqr += mrpt::square(error);
            error_terms++;
        }

        const float frame_rmse =
            std::sqrt(frame_sum_error_sqr / static_cast<float>(deskewedPts->size()));
        result.individual_frame_rmse.push_back(frame_rmse);

        ++kfIdx;
    }  // end for each timestep

    result.rmse = std::sqrt(sum_error_sqr / static_cast<float>(error_terms));
    return result;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        std::vector<mp2p_icp_filters::MotionCompensationMethod> methods = {
            mp2p_icp_filters::MotionCompensationMethod::None,
            mp2p_icp_filters::MotionCompensationMethod::Linear};
#if MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION
        methods.push_back(mp2p_icp_filters::MotionCompensationMethod::IMU);
        methods.push_back(mp2p_icp_filters::MotionCompensationMethod::IMUh);
#endif

        const std::vector<std::pair<float, float>> test_velocities = {
            {0.0f, 1e-6f},  // stationary
            {1.0f, 1e-6f},  // linear only
            {0.0f, 0.02f},  // angular only
            {1.0f, 0.02f},  // linear + angular
            {2.0f, 0.05f},  // faster linear + angular
            {0.5f, 0.10f},  // slow linear, fast angular
            {3.0f, 0.05f}  // fast linear, slow angular
        };

        for (const auto& [lin, ang] : test_velocities)
        {
            SimulationParams p;
            p.linear_speed = lin;
            p.angular_vel  = ang;
            std::cout << "\n=== Test velocities: lin=" << lin << " ang=" << ang << "\n";

            for (const auto method : methods)
            {
                p.deskew_method = method;

                const auto eval = run_deskew_test(p);

                printf(
                    " %-32s | rmse: %10.6f | errs: ", mrpt::typemeta::enum2str(method).c_str(),
                    eval.rmse);

                for (std::size_t i = 0; i < 7; i++)
                {
                    printf("%4.02f ", eval.individual_frame_rmse[i]);
                }
                printf("...\n");
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
