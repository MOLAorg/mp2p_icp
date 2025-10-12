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
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
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

struct Scenario
{
    mrpt::poses::CPose3DInterpolator                         gtKeyFrames;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>  gtTwist;
    std::map<mrpt::Clock::time_point, mrpt::math::TTwist3D>  imuAngVel;
    std::map<mrpt::Clock::time_point, mrpt::math::TVector3D> imuAccel;
};

Scenario simulate_scenario(const SimulationParams& p)
{
    Scenario s;
    auto&    kfs            = s.gtKeyFrames;
    auto&    kfTwists       = s.gtTwist;
    auto&    imuReadings    = s.imuAngVel;
    auto&    imuReadingsAcc = s.imuAccel;

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
            stamp, mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, z, theta, 0.0_deg, 0.0_deg));
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

    return s;
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

// === DESKEW TEST ===
[[nodiscard]] SimulationResult run_deskew_test(const SimulationParams& p)
{
    // Generate test data:
    const auto gtPoints = create_gt_points(p);

    const auto [gtKeyframes, gtTwist, imuReadings, imuReadingsAcc] = simulate_scenario(p);

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

// === DESKEW TEST VIA SM2MM  ===
[[nodiscard]] SimulationResult run_deskew_in_sm2mm_test(const SimulationParams& p)
{
    // Generate test data:
    const auto gtPoints = create_gt_points(p);

    const auto [gtKeyframes, gtTwist, imuReadings, imuReadingsAcc] = simulate_scenario(p);

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

    mrpt::maps::CSimpleMap sm;

    mrpt::maps::CSimplePointsMap gtGlobalPointsAggregated;

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
        gtGlobalPointsAggregated.insertAnotherMap(&gtLocalPoints, mrpt::poses::CPose3D(pose));

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

        // Store into SimpleMap:
        auto keyframe_obs = mrpt::obs::CSensoryFrame::Create();
        {
            auto pcObs         = mrpt::obs::CObservationPointCloud::Create();
            pcObs->timestamp   = stamp;
            pcObs->sensorLabel = "scan";
            pcObs->pointcloud  = skewedPoints;
            keyframe_obs->insert(pcObs);
        }
        mrpt::containers::yaml kf_metadata = mrpt::containers::yaml::Map();

        // Store local velocity buffer in the KF metadata so it is possible to deskew the scan later
        // on with precision
        kf_metadata["local_velocity_buffer"] = ps.localVelocityBuffer.toYAML();

        // convert yaml to string:
        std::stringstream ss;
        ss << kf_metadata;

        auto metadataObs         = mrpt::obs::CObservationComment::Create();
        metadataObs->timestamp   = stamp;
        metadataObs->sensorLabel = "metadata";
        metadataObs->text        = ss.str();

        // insert it:
        *keyframe_obs += metadataObs;

        sm.insert(
            // Pose: mean + covariance
            mrpt::poses::CPose3DPDFGaussian::Create(mrpt::poses::CPose3D(pose)),
            // SensoryFrame: set of observations from this KeyFrame:
            keyframe_obs,
            // twist
            kfGtTwist);

        ++kfIdx;
    }  // end for each timestep

    // Now, reconstruct the points within the SM:
    const auto sm2mmPipeline = mrpt::containers::yaml::FromText(
        mrpt::format(
            R"yaml(
# --------------------------------------------------------
# 1) Generator (observation -> local frame metric maps)
# --------------------------------------------------------
generators:
  - class_name: mp2p_icp_filters::Generator
    params: ~

# --------------------------------------------------------
# 2) Per local frame filtering
# --------------------------------------------------------
filters:
  - class_name: mp2p_icp_filters::FilterAdjustTimestamps
    params:
      pointcloud_layer: "raw"
      silently_ignore_no_timestamps: false
      #method: "TimestampAdjustMethod::MiddleIsZero"
      method: "TimestampAdjustMethod::EarliestIsZero"

  - class_name: mp2p_icp_filters::FilterDeskew
    params:
      input_pointcloud_layer: "raw"
      output_pointcloud_layer: "deskewed"
      method: %s
      silently_ignore_no_timestamps: false

      output_layer_class: "mrpt::maps::CPointsMapXYZIRT" # Keep intensity & ring channels

      # These (vx,...,wz) are variable names that must be defined via the
      # mp2p_icp::Parameterizable API to update them dynamically.
      twist: [vx, vy, vz, wx, wy, wz]

  - class_name: mp2p_icp_filters::FilterDeleteLayer
    params:
      # one or more layers to remove
      pointcloud_layer_to_remove: ["raw"]
    )yaml",
            mrpt::typemeta::enum2str(p.deskew_method).c_str()));

    mp2p_icp::metric_map_t            mm;
    mp2p_icp_filters::sm2mm_options_t sm2mm_opts;
    // sm2mm_opts.verbosity = mrpt::system::LVL_DEBUG;

    mp2p_icp_filters::simplemap_to_metricmap(sm, mm, sm2mmPipeline, sm2mm_opts);

    // And evaluate:
    // ---------------------------
    // Compare points:
    const auto deskewedPts = mm.point_layer("deskewed");
    ASSERT_EQUAL_(deskewedPts->size(), gtGlobalPointsAggregated.size());
    for (size_t i = 0; i < deskewedPts->size(); i++)
    {
        mrpt::math::TPoint3Df pt;
        deskewedPts->getPoint(i, pt.x, pt.y, pt.z);
        mrpt::math::TPoint3Df gtPt;
        gtGlobalPointsAggregated.getPoint(i, gtPt.x, gtPt.y, gtPt.z);

        const auto error = (pt - gtPt).norm();
        sum_error_sqr += mrpt::square(error);
        error_terms++;
    }

    result.rmse = std::sqrt(sum_error_sqr / static_cast<float>(error_terms));
    return result;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    int num_errors = 0;
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

        for (int use_sm2mm = 0; use_sm2mm <= 1; use_sm2mm++)
        {
            std::cout
                << (use_sm2mm != 0 ? "\n######### Using FilterDesk directly\n"
                                   : "\n######### Using ms2mm() function\n");

            for (const auto& [lin, ang] : test_velocities)
            {
                SimulationParams p;
                p.linear_speed = lin;
                p.angular_vel  = ang;
                std::cout << "\n=== Test velocities: lin=" << lin << " ang=" << ang << "\n";

                for (const auto method : methods)
                {
                    p.deskew_method = method;

                    // Run one of two possible tests:
                    const auto eval = use_sm2mm ? run_deskew_in_sm2mm_test(p) : run_deskew_test(p);

                    printf(
                        " %-32s | rmse: %10.6f | errs: ", mrpt::typemeta::enum2str(method).c_str(),
                        eval.rmse);

                    for (std::size_t i = 0;
                         i < std::min<std::size_t>(eval.individual_frame_rmse.size(), 6U); i++)
                    {
                        printf("%6.02f ", eval.individual_frame_rmse[i] * 1e3);
                    }
                    printf("... [mm] ");

                    // Check:
                    const float threshold =
                        method == mp2p_icp_filters::MotionCompensationMethod::None ? 0.25 : 0.001;
                    if (eval.rmse > threshold)
                    {
                        printf("❌ FAILED.\n");
                        num_errors++;
                    }
                    else
                    {
                        printf("✅ Passed.\n");
                    }
                }
            }
        }  // end use_sm2mm
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }

    printf("Number of test failures: %i\n", num_errors);

    return num_errors == 0 ? 0 : 1;
}
