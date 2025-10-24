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
#include <mrpt/version.h>

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

    // --- Simulation Parameters & Constants ---
    const double T_ACCEL    = 3.0;  // Time for the acceleration phase (seconds)
    const double T_CONST    = 1.0;  // Time for the constant speed phase (seconds)
    const double T_END      = T_ACCEL + T_CONST;
    const double KF_PERIOD  = 0.10;  // KeyFrame generation period (10 Hz)
    const double IMU_PERIOD = 0.01;  // IMU reading period (100 Hz)

    // Target final velocities
    const double V_FINAL = p.linear_speed;  // local +X
    const double W_FINAL = p.angular_vel;  // local +Z (yaw)

    // Calculate accelerations
    // Uniform acceleration: a = dv/dt
    const double A_LINEAR  = V_FINAL / T_ACCEL;  // linear acceleration (local +X)
    const double ALPHA_ANG = W_FINAL / T_ACCEL;  // angular acceleration (local +Z)

    // --- Phase 1: Uniformly Accelerated Motion (0.0s to 3.0s) ---
    double               t = 0.0;
    mrpt::poses::CPose3D pose_prev(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);

    // Initial state at t=0.0
    kfs.insert(mrpt::Clock::fromDouble(t), pose_prev);
    kfTwists[mrpt::Clock::fromDouble(t)] = {0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg};

    // Integrate over keyframe time steps (10 Hz)
    for (t = KF_PERIOD; t <= T_ACCEL + 1e-6; t += KF_PERIOD)
    {
        const double dt = t;  // Time elapsed since t=0

        // Linear Motion (Uniformly Accelerated: x = 0.5 * a * t^2)
        const double v_t = A_LINEAR * dt;

        // Angular Motion (Uniformly Accelerated: yaw = 0.5 * alpha * t^2)
        const double w_t = ALPHA_ANG * dt;

        // Position in World Frame (Integrate velocity)
        mrpt::poses::CPose3D new_pose = pose_prev;

        // Velocity at the start of the interval
        const double v_start = std::max(0.0, A_LINEAR * (t - KF_PERIOD));
        const double w_start = std::max(0.0, ALPHA_ANG * (t - KF_PERIOD));

        // Average velocity during the interval:
        // v_avg = 0.5 * (v_start + v_t)
        // w_avg = 0.5 * (w_start + w_t)
        const double v_avg = 0.5 * (v_start + v_t);
        const double w_avg = 0.5 * (w_start + w_t);

        // Delta pose (local frame of pose_prev)
        const double dx_local   = v_avg * KF_PERIOD;  // Simple linear distance
        const double dy_local   = 0.0;
        const double dz_local   = 0.0;
        const double dyaw_local = w_avg * KF_PERIOD;

        // Create the incremental pose:
        mrpt::poses::CPose3D delta_pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            dx_local, dy_local, dz_local, dyaw_local, 0.0_deg, 0.0_deg);

        // Compose poses:
        new_pose += delta_pose;
        pose_prev = new_pose;

        // Insert KeyFrame
        const auto stamp = mrpt::Clock::fromDouble(t);
        kfs.insert(stamp, new_pose);

        // Twist (in local frame, based on final velocity at time t)
        kfTwists[stamp] = {v_t, 0, 0, 0.0_deg, 0.0_deg, w_t};
    }

    // --- Phase 2: Constant Velocity Motion (3.0s to 4.0s) ---
    // Start from the last pose of the acceleration phase
    pose_prev = mrpt::poses::CPose3D(kfs.rbegin()->second);

    // The twist is constant at V_FINAL and W_FINAL
    const mrpt::math::TTwist3D twist_const = {V_FINAL, 0, 0, 0.0_deg, 0.0_deg, W_FINAL};

    // Integrate over keyframe time steps (10 Hz)
    for (t = T_ACCEL + KF_PERIOD; t <= T_END + 1e-6; t += KF_PERIOD)
    {
        // Delta pose (local frame of pose_prev) is constant
        const double dx_local   = V_FINAL * KF_PERIOD;
        const double dy_local   = 0.0;
        const double dz_local   = 0.0;
        const double dyaw_local = W_FINAL * KF_PERIOD;

        mrpt::poses::CPose3D delta_pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            dx_local, dy_local, dz_local, dyaw_local, 0.0_deg, 0.0_deg);

        mrpt::poses::CPose3D new_pose = pose_prev;
        new_pose += delta_pose;
        pose_prev = new_pose;

        const auto stamp = mrpt::Clock::fromDouble(t);
        kfs.insert(stamp, new_pose);
        kfTwists[stamp] = twist_const;
    }

    // --- IMU Readings Simulation (100 Hz) ---

    // Define the time range for IMU simulation
    const double t_start = 0.0;
    const double t_end   = T_END;

    for (double t_imu = t_start; t_imu <= t_end + 1e-8; t_imu += IMU_PERIOD)
    {
        const auto            stamp = mrpt::Clock::fromDouble(t_imu);
        mrpt::math::TTwist3D  twist;
        mrpt::math::TVector3D acceleration_local(0, 0, 0);

        if (t_imu <= T_ACCEL + 1e-6)  // Acceleration Phase (0.0s to 3.0s)
        {
            // Angular velocity (local +Z): w_t = alpha * t
            const double w_t = ALPHA_ANG * t_imu;
            twist            = {0, 0, 0, 0.0_deg, 0.0_deg, w_t};

            // Linear acceleration (local +X)
            acceleration_local.x = A_LINEAR;

            // Centripetal acceleration (local -Y)
            // Centripetal acceleration: a_c = v * w
            const double v_t     = A_LINEAR * t_imu;
            const double a_c     = v_t * w_t;
            acceleration_local.y = -a_c;  // Towards center of rotation (local -Y)
        }
        else  // Constant Velocity Phase (3.0s to 4.0s)
        {
            twist = twist_const;

            // Linear acceleration is zero (velocity is constant)
            acceleration_local.x = 0.0;

            // Centripetal acceleration is constant
            // a_c = V_FINAL * W_FINAL
            const double a_c     = V_FINAL * W_FINAL;
            acceleration_local.y = -a_c;
        }

        imuReadings[stamp] = twist;  // Angular velocity (w) is in the last three fields

        // IMU Reading: True Acceleration + Gravity (reaction force)
        // Gravity in local frame for yaw-only rotation
        mrpt::math::TVector3D g_local_compensated(0, 0, -9.81);

        // Apply the standard formula: $A_{measured} = A_{true} - G_{local}$
        // Since $G_{local} = (0, 0, -9.81)$, we get:
        // $A_{measured, local}.z = 0 - (-9.81) = 9.81$
        imuReadingsAcc[stamp] = acceleration_local - g_local_compensated;
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

#if MRPT_VERSION >= 0x020f00  // 2.15.0
        pts->insertPointField_float("t", static_cast<float>(rel_time));
#else
        pts->insertPointField_Timestamp(static_cast<float>(rel_time));
#endif

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

    for (const auto& [stamp, imu] : imuReadings)
    {
        std::cout << "Stamp: " << mrpt::Clock::toDouble(stamp) << " | IMU_W: " << imu.asString()
                  << " | IMU_ACC: " << imuReadingsAcc.at(stamp).asString() << "\n";
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
    const auto sm2mmPipeline = mrpt::containers::yaml::FromText(mrpt::format(
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
                                   : "\n######### Using sm2mm() function\n");

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
                        (method == mp2p_icp_filters::MotionCompensationMethod::None
                             ? 0.20f
                             : (method == mp2p_icp_filters::MotionCompensationMethod::Linear
                                    ? 0.005f
                                    : 0.001f));
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
