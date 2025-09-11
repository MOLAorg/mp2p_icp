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

#include <Eigen/Dense>

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

namespace
{

/** Each of the recovered-trajectory key-points.
 * Some members are std::optional just to help in the process of reconstruction, to know which are
 * already computed.
 */
struct TrajectoryPoint
{
    TrajectoryPoint() = default;

    TrajectoryPoint(const mrpt::math::CMatrixDouble33& R_, const mrpt::math::TVector3D& p_)
        : pose(
              mrpt::poses::CPose3D::FromRotationAndTranslation(
                  R_, mrpt::math::CVectorFixedDouble<3>(p_)))
    {
    }

    /// SE(3) pose, relative to "t=0"
    mrpt::poses::CPose3D pose;

    /// SO(3) orientation, gravity aligned
    std::optional<mrpt::math::CMatrixDouble33> R_ga;

    /// Linear velocity (v) in the frame of reference of "t=0"
    std::optional<mrpt::math::TVector3D> v;

    // Angular velocity (ω) in the body (b) frame (directly from IMU)
    std::optional<mrpt::math::TVector3D> w_b;

    /// Linear acceleration (a) in the body (b) frame (directly from IMU, transformed)
    /// (This one still has gravity)
    std::optional<mrpt::math::TVector3D> a_b;

    /// Linear coordinate acceleration (body frame) (without gravity effects)
    std::optional<mrpt::math::TVector3D> ac_b;

    /// Angular acceleration (α)
    mrpt::math::TVector3D alpha = {0, 0, 0};

    /// Jerk = \dot{a}
    mrpt::math::TVector3D j = {0, 0, 0};

    std::string asString() const
    {
        using mrpt::format;

        auto vecToStr = [](const mrpt::math::TVector3D& x)
        { return mrpt::format("[%.3f %.3f %.3f]", x[0], x[1], x[2]); };

        auto optVecToStr = [&](const std::optional<mrpt::math::TVector3D>& x)
        { return x ? vecToStr(*x) : std::string{"<none>"}; };

        auto matToStr = [](const mrpt::math::CMatrixDouble33& M)
        {
            mrpt::poses::CPose3D pp;
            pp.setRotationMatrix(M);
            return mrpt::format(
                "(ypr)=(%.02f,%.02f,%.02f) [deg]", mrpt::RAD2DEG(pp.yaw()),
                mrpt::RAD2DEG(pp.pitch()), mrpt::RAD2DEG(pp.roll()));
        };

        auto optMatToStr = [&](const std::optional<mrpt::math::CMatrixDouble33>& M)
        { return M ? matToStr(*M) : std::string{"<none>"}; };

        std::ostringstream oss;
        oss << "TrajectoryPoint{"
            << "\n  pose = " << pose.asString()  //
            << "\n  R_ga = " << optMatToStr(R_ga)  //
            << "\n  v    = " << optVecToStr(v)  //
            << "\n  w_b  = " << optVecToStr(w_b)  //
            << "\n  a_b  = " << optVecToStr(a_b)  //
            << "\n  ac_b = " << optVecToStr(ac_b)  //
            << "\n  alpha= " << vecToStr(alpha)  //
            << "\n  j    = " << vecToStr(j)  //
            << "\n}";
        return oss.str();
    }
};

/// A recovered trajectory, indexed by relative time in seconds (t=0 is the scan reference stamp)
using Trajectory = std::map<double, TrajectoryPoint>;

auto findBeforeAfter(const Trajectory& trajectory, const double t)
    -> std::pair<Trajectory::const_iterator, Trajectory::const_iterator>
{
    using Iterator = Trajectory::const_iterator;

    // Don't check for "!trajectory.empty()", it's done in the caller.

    Iterator lower = trajectory.lower_bound(t);

    if (lower == trajectory.end())
    {
        // key is larger than all elements
        Iterator last = std::prev(trajectory.end());
        return {last, last};
    }

    if (lower->first == t)
    {
        // Exact match: before = prev, after = next (clamped)
        Iterator before = (lower == trajectory.begin()) ? lower : std::prev(lower);
        Iterator after  = std::next(lower);
        if (after == trajectory.end())
        {
            after = lower;
        }
        return {before, after};
    }

    if (lower == trajectory.begin())
    {
        // key is smaller than all elements
        return {lower, lower};
    }

    // General case: key lies between prev(lower) and lower
    Iterator before = std::prev(lower);
    return {before, lower};
}

// Optimized templated version for compile-time optimization for each method
template <MotionCompensationMethod method>
void correctPointsLoop(
    const mrpt::aligned_std_vector<float>& xs, const mrpt::aligned_std_vector<float>& ys,
    const mrpt::aligned_std_vector<float>& zs, size_t n, size_t n0, mrpt::maps::CPointsMap* outPc,
    const mrpt::aligned_std_vector<float>* Is, mrpt::aligned_std_vector<float>* out_Is,
    const mrpt::aligned_std_vector<uint16_t>* Rs, mrpt::aligned_std_vector<uint16_t>* out_Rs,
    const mrpt::aligned_std_vector<float>* Ts, mrpt::aligned_std_vector<float>* out_Ts,
    const mrpt::math::TTwist3D* constant_twist, const Trajectory& reconstructed_trajectory)
{
    MRPT_TODO("First, build a cache with times -> corrections");

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

            // Compile-time "switch":
            if constexpr (method == MotionCompensationMethod::Linear)
            {
                // Forward integrate constant twist:
                const auto v = mrpt::math::TVector3D(
                    constant_twist->vx, constant_twist->vy, constant_twist->vz);
                const auto w = mrpt::math::TVector3D(
                    constant_twist->wx, constant_twist->wy, constant_twist->wz);

                const mrpt::math::TVector3D v_dt = v * (*Ts)[i];
                const mrpt::math::TVector3D w_dt = w * (*Ts)[i];

                pose_increment = mrpt::poses::CPose3D::FromRotationAndTranslation(
                    // Rotation: From Lie group SO(3) exponential:
                    mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt)),
                    // Translation: simple constant velocity model:
                    v_dt);
            }
            else if constexpr (method == MotionCompensationMethod::IMU)
            {
                const auto t_point    = (*Ts)[i];
                const auto [it0, it1] = findBeforeAfter(reconstructed_trajectory, t_point);

                const auto   t_prev  = it0->first;
                const double dt      = t_point - t_prev;
                const auto&  tp_prev = it0->second;

                // v was already in the t=0 frame of reference:
                const mrpt::math::TVector3D v_dt = *tp_prev.v * dt;
                const mrpt::math::TVector3D w_dt = *tp_prev.w_b * dt;

                pose_increment = mrpt::poses::CPose3D::FromRotationAndTranslation(
                    // Rotation: From Lie group SO(3) exponential:
                    tp_prev.pose.getRotationMatrix() *
                        mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt)),
                    // Translation: simple constant velocity model:
                    tp_prev.pose.translation() + v_dt);
            }
            else if constexpr (method == MotionCompensationMethod::IMU_interp)
            {
                //
                THROW_EXCEPTION("to do");
            }
            else
            {
                // Should never arrive here
                THROW_EXCEPTION("Unhandled MotionCompensationMethod method (!)");
            }

            // Correct point XYZ coordinates.
            const auto corrPt = pose_increment.composePoint(pt);

            outPc->setPointFast(
                n0 + i, static_cast<float>(corrPt.x), static_cast<float>(corrPt.y),
                static_cast<float>(corrPt.z));

            // Copy additional fields
            if (Is && out_Is)
            {
                (*out_Is)[n0 + i] = (*Is)[i];
            }
            if (Rs && out_Rs)
            {
                (*out_Rs)[n0 + i] = (*Rs)[i];
            }
            if (Ts && out_Ts)
            {
                (*out_Ts)[n0 + i] = (*Ts)[i];
            }
        }
#if defined(MP2P_HAS_TBB)
    );
#endif
}

void execute_integration(
    Trajectory&                                                                           t,
    const std::function<void(const TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)>& update,
    const double start_time = 0.0)
{
    for (auto it = t.find(start_time); it != t.end(); it++)
    {
        auto it_next = std::next(it);
        if (it_next == t.end())
        {
            break;
        }
        const auto&  p_prev = it->second;
        auto&        p_this = it_next->second;
        const double dt     = it_next->first - it->first;

        update(p_prev, p_this, dt);
    }
    for (auto it = std::make_reverse_iterator(std::next(t.find(start_time))); it != t.rend(); ++it)
    {
        auto it_next = std::next(it);  // reverse_iterator trick
        if (it_next == t.rend())
        {
            break;
        }

        const auto&  p_prev = it->second;
        auto&        p_this = it_next->second;
        const double dt     = it_next->first - it->first;

        update(p_prev, p_this, dt);
    }
};

Trajectory reconstructTrajectoryFromIMU(const mp2p_icp::LocalVelocityBuffer::SampleHistory& samples)
{
    Trajectory t;

    // 1) Build the list of all timestamps that we will reconstruct:
    // {0, t_IMU}
    t[0] = TrajectoryPoint(mrpt::math::CMatrixDouble33::Identity(), {.0, .0, .0});
    for (const auto& [stamp, _] : samples.by_type.w_b)
    {
        t[stamp] = {};
    }

    // 2) Fill (ω,a) from IMU:
    for (const auto& [stamp, w] : samples.by_type.w_b)
    {
        t[stamp].w_b = w;
    }
    for (const auto& [stamp, acc] : samples.by_type.a_b)
    {
        t[stamp].a_b = acc;
    }

    // 3) Copy the closest gravity-aligned hints on global orientations:
    std::optional<double> stamp_first_R_ga;
    for (const auto& [stamp, so3] : samples.by_type.q)
    {
        t[stamp].R_ga = so3;
        if (!stamp_first_R_ga)
        {
            stamp_first_R_ga = stamp;
        }
    }
    ASSERTMSG_(
        stamp_first_R_ga.has_value(),
        "At least one entry with gravity-aligned orientation is needed for IMU integration");

    // 4) Integrate R forward / backwards in time:
    execute_integration(
        t,
        [](const TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)
        {
            const mrpt::math::TVector3D w =
                p0.w_b.has_value()
                    ? *p0.w_b
                    : (p1.w_b.has_value() ? *p1.w_b : mrpt::math::TVector3D(.0, .0, .0));

            p1.pose.setRotationMatrix(
                p0.pose.getRotationMatrix() *
                mrpt::poses::Lie::SO<3>::exp(
                    (w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>()));
        });

    // 5) Integrate R_gravity_aligned:
    execute_integration(
        t,
        [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
        {
            p1.R_ga = (*p0.R_ga) *
                      ((p0.pose.getRotationMatrix()).inverse() * (p1.pose.getRotationMatrix()));
        },
        stamp_first_R_ga.value());

    // 6) convert acceleration:
    // proper acceleration in the body frame => coordinate acceleration in the body frame
    for (auto& [stamp, p] : t)
    {
        MRPT_TODO("get acc bias");
        const mrpt::math::TVector3D bias_acc = {0, 0, 0};
        const Eigen::Vector3d       gravity(0, 0, -9.81);

        if (p.a_b)
        {
            MRPT_TODO("verify this!");
            const auto gravity_b = p.R_ga->asEigen().inverse() * gravity;

            ASSERT_(p.a_b);
            p.ac_b = *p.a_b - bias_acc +
                     mrpt::math::TVector3D(gravity_b.x(), gravity_b.y(), gravity_b.z());
        }
        else
        {
            p.ac_b = {0, 0, 0};
        }
    }

    // 7) Copy the closest velocity from the given samples:
    std::optional<double> stamp_first_v_b;
    for (const auto& [stamp, v_b] : samples.by_type.v_b)
    {
        t[stamp].v = v_b;
        if (!stamp_first_v_b)
        {
            stamp_first_v_b = stamp;
        }
    }
    ASSERTMSG_(
        stamp_first_v_b.has_value(),
        "At least one entry with velocity is needed for IMU integration");

    // 8) Integrate v:
    execute_integration(
        t,
        [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
        {
            ASSERT_(p0.v.has_value());
            p1.v = *p0.v + p0.pose.rotateVector(p0.ac_b.value() * dt);
        },
        stamp_first_v_b.value());

    // 9) Integrate p:
    execute_integration(
        t,
        [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
        {
            ASSERT_(p0.v.has_value());
            const auto t = p0.pose.translation() + p0.v.value() * dt;
            p1.pose.x(t.x);
            p1.pose.y(t.y);
            p1.pose.z(t.z);
        });

#if 0
    // Debug print
    for (auto& [stamp, p] : t)
    {
        std::cout << "t=" << stamp << " " << p.asString() << "\n";
    }
#endif

    return t;
}

Trajectory reconstructTrajectoryFromIMU_interp(
    const mp2p_icp::LocalVelocityBuffer::SampleHistory& samples)
{
    Trajectory t;

    return t;
}

}  // namespace

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
    Trajectory reconstructed_trajectory;

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
            const auto sample_history =
                ps->localVelocityBuffer.collect_samples_around_reference_time(scan_time_span);

            if (method == MotionCompensationMethod::IMU)
            {
                reconstructed_trajectory = reconstructTrajectoryFromIMU(sample_history);
            }
            else if (method == MotionCompensationMethod::IMU_interp)
            {
                reconstructed_trajectory = reconstructTrajectoryFromIMU_interp(sample_history);
            }
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

    // compile-time optimized code for each method:
    switch (method)
    {
        case MotionCompensationMethod::Linear:
            correctPointsLoop<MotionCompensationMethod::Linear>(
                xs, ys, zs, n, n0, outPc.get(), Is, out_Is, Rs, out_Rs, Ts, out_Ts, constant_twist,
                reconstructed_trajectory);
            break;

        case MotionCompensationMethod::IMU:
            correctPointsLoop<MotionCompensationMethod::IMU>(
                xs, ys, zs, n, n0, outPc.get(), Is, out_Is, Rs, out_Rs, Ts, out_Ts, constant_twist,
                reconstructed_trajectory);
            break;

        case MotionCompensationMethod::IMU_interp:
            correctPointsLoop<MotionCompensationMethod::IMU_interp>(
                xs, ys, zs, n, n0, outPc.get(), Is, out_Is, Rs, out_Rs, Ts, out_Ts, constant_twist,
                reconstructed_trajectory);
            break;

        default:
            // Should never arrive here
            throw std::runtime_error("Unknown MotionCompensationMethod method");
    }

    MRPT_END
}

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
            }

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