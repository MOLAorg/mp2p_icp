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
 * @file   FilterDeskew.cpp
 * @brief  Deskew (motion compensate) a pointcloud from a moving LIDAR
 * @author Jose Luis Blanco Claraco
 * @date   Dec 13, 2023
 */

#include <mp2p_icp_filters/FilterDeskew.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/find_closest.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>

#if defined(MP2P_HAS_IMU_PREINTEGRATION_LIB)
#include <mola_imu_preintegration/IMUIntegrationParams.h>
#include <mola_imu_preintegration/ImuTransformer.h>
#endif

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterDeskew, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterDeskew::FilterDeskew() { mrpt::system::COutputLogger::setLoggerName("FilterDeskew"); }

void FilterDeskew::initialize(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);

    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_OPT(c, silently_ignore_no_timestamps);
    MCP_LOAD_OPT(c, output_layer_class);
    MCP_LOAD_OPT(c, method);

    if (c.has("twist"))
    {
        twist.emplace();  // Define a constant twist model for deskewing
        ASSERT_(c["twist"].isSequence());
        ASSERT_EQUAL_(c["twist"].asSequence().size(), 6UL);

        const auto yamlTwist = c["twist"].asSequence();

        for (int i = 0; i < 6; i++)
        {
            Parameterizable::parseAndDeclareParameter(
                yamlTwist.at(i).as<std::string>(), twist.value()[i]);
        }
    }
}

void FilterDeskew::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // Out:
    ASSERT_(!output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, output_pointcloud_layer, false /*dont allow empty names*/, output_layer_class);

    // In:
    ASSERT_(!input_pointcloud_layer.empty());

    const mrpt::maps::CPointsMap* inPc = nullptr;
    if (auto itLy = inOut.layers.find(input_pointcloud_layer); itLy != inOut.layers.end())
    {
        inPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!inPc)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.", input_pointcloud_layer.c_str());
        }

        outPc->reserve(outPc->size() + inPc->size());
    }
    else
    {
        // Input layer doesn't exist:
        THROW_EXCEPTION_FMT(
            "Input layer '%s' not found on input map.", input_pointcloud_layer.c_str());
    }

    // If the input is empty, just move on:
    if (inPc->empty())
    {
        MRPT_LOG_DEBUG_STREAM(
            "Silently ignoring empty input layer: '" << input_pointcloud_layer << "'");
        return;
    }

    // mandatory fields:
    const auto&  xs = inPc->getPointsBufferRef_x();
    const auto&  ys = inPc->getPointsBufferRef_y();
    const auto&  zs = inPc->getPointsBufferRef_z();
    const size_t n  = xs.size();

    // optional fields:
    const auto* Is = inPc->getPointsBufferRef_intensity();
    const auto* Ts = inPc->getPointsBufferRef_timestamp();
    const auto* Rs = inPc->getPointsBufferRef_ring();

    auto* out_Is = outPc->getPointsBufferRef_intensity();
    auto* out_Rs = outPc->getPointsBufferRef_ring();
    auto* out_Ts = outPc->getPointsBufferRef_timestamp();

    // Helper lambda: copy all points (with optional attributes)
    auto copyAllPoints = [&]()
    {
        for (size_t i = 0; i < n; i++)
        {
            outPc->insertPointFrom(*inPc, i);
        }
    };

    // No timestamps available or deskewing disabled:
    const bool noTimestamps   = !Ts || Ts->empty();
    const bool deskewDisabled = (method == MotionCompensationMethod::None);

    if (noTimestamps || deskewDisabled)
    {
        if (silently_ignore_no_timestamps || deskewDisabled)
        {
            copyAllPoints();

            if (!deskewDisabled)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "Skipping de-skewing in input cloud '"
                    << input_pointcloud_layer << "' with contents: " << inPc->asString());
            }
            return;
        }

        THROW_EXCEPTION_FMT(
            "Input layer '%s' does not contain per-point timestamps, cannot do scan deskew. Set "
            "'silently_ignore_no_timestamps=true' to skip de-skew. Input map contents: '%s'",
            input_pointcloud_layer.c_str(), inPc->asString().c_str());
    }

    ASSERT_EQUAL_(Ts->size(), n);

    // Yes, we have timestamps, apply de-skew:
    const size_t n0 = outPc->size();
    outPc->resize(n0 + n);

    // Used for precise deskew-only. This contains relative poses of the vehicle frame ("base_link")
    // with t=0 being the reference time when t=0 in the point cloud timestamp field:
    std::optional<mp2p_icp::LocalVelocityBuffer::SampleHistory> sample_history_opt;

    const mrpt::math::TTwist3D* constant_twist = nullptr;

    switch (method)
    {
        case MotionCompensationMethod::IMU:
        case MotionCompensationMethod::IMU_interp:
        {
            const auto* ps = attachedSource();
            ASSERTMSG_(ps, "A ParameterSource must be attached if IMU-based methods are enabled");

            const auto [it_min, it_max] = std::minmax_element(Ts->cbegin(), Ts->cend());
            ASSERT_(it_min != Ts->cend());
            ASSERT_(it_max != Ts->cend());

            const double scan_time_span = *it_max - *it_min;

            // Recall, the reference time should have been set already by the Generator and/or
            // FilterAdjustTimestamps:
            sample_history_opt =
                ps->localVelocityBuffer.collect_samples_around_reference_time(scan_time_span);
        }
        break;

        case MotionCompensationMethod::Linear:
        {
            ASSERTMSG_(
                twist.has_value(),
                "`MotionCompensationMethod::Linear` needs defining a constant 'twist' field in "
                "this filter parameters");

            constant_twist = &twist.value();
        }
        break;

        case mp2p_icp_filters::MotionCompensationMethod::None:
        {
            // Should have been handled above!
            THROW_EXCEPTION("Shouldn't reach here!");
            break;
        }
    };

#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        static_cast<size_t>(0), n,
        [&](size_t i)
#else
    for (size_t i = 0; i < n; i++)
#endif
        {
            const auto pt = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);
            if (pt.x == 0 && pt.y == 0 && pt.z == 0)
            {
#if defined(MP2P_HAS_TBB)
                return;
#else
            continue;
#endif
            }

            mrpt::poses::CPose3D pose_increment;

#if 0

            // Use precise trajectory, if enabled and if there is enough data (otherwise,
            // fallback to constant velocity)
            if (!constant_twist)
            {
                // Translation:
                const auto v =
                    mrpt::math::TVector3D(constant_twist.vx, constant_twist.vy, constant_twist.vz);
                const mrpt::math::TVector3D v_dt = v * (*Ts)[i];

                // Interpolated rotation:
                const auto found_pose_opt =
                    mrpt::containers::find_closest(*interpolated_relative_poses, (*Ts)[i]);
                if (found_pose_opt.has_value())
                {
                    pose_increment = found_pose_opt->second;
                    pose_increment.x(v_dt.x);
                    pose_increment.y(v_dt.y);
                    pose_increment.z(v_dt.z);
                }
            }
            else
            {
                // Forward integrate constant twist:
                const auto v =
                    mrpt::math::TVector3D(constant_twist.vx, constant_twist.vy, constant_twist.vz);
                const auto w =
                    mrpt::math::TVector3D(constant_twist.wx, constant_twist.wy, constant_twist.wz);

                const mrpt::math::TVector3D v_dt = v * (*Ts)[i];
                const mrpt::math::TVector3D w_dt = w * (*Ts)[i];

                pose_increment = mrpt::poses::CPose3D::FromRotationAndTranslation(
                    // Rotation: From Lie group SO(3) exponential:
                    mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt)),
                    // Translation: simple constant velocity model:
                    v_dt);
            }
#endif
            // Correct point XYZ coordinates:
            const auto corrPt = pose_increment.composePoint(pt);

            // Set corrected XYZ:
            outPc->setPointFast(n0 + i, corrPt.x, corrPt.y, corrPt.z);
            // and copy the rest of fields:
            if (Is && out_Is) (*out_Is)[n0 + i] = (*Is)[i];
            if (Rs && out_Rs) (*out_Rs)[n0 + i] = (*Rs)[i];
            if (Ts && out_Ts) (*out_Ts)[n0 + i] = (*Ts)[i];
        }
#if defined(MP2P_HAS_TBB)
    );
#endif

    outPc->mark_as_modified();
    MRPT_END
}

#if 0
LocalVelocityBuffer::reconstruct_poses_around_reference_time(double half_time_span) const
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
#endif