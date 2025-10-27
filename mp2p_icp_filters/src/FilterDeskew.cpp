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

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
#include <mola_imu_preintegration/ImuIntegrationParams.h>
#include <mola_imu_preintegration/ImuTransformer.h>
#include <mola_imu_preintegration/trajectory_from_buffer.h>
#endif

#include <mp2p_icp_filters/FilterDeskew.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/version.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterDeskew, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterDeskew::FilterDeskew() { mrpt::system::COutputLogger::setLoggerName("FilterDeskew"); }

void FilterDeskew::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);

    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, output_pointcloud_layer);

    MCP_LOAD_OPT(c, silently_ignore_no_timestamps);
    MCP_LOAD_OPT(c, output_layer_class);
    MCP_LOAD_OPT(c, method);
    MCP_LOAD_OPT(c, in_place);

    MCP_LOAD_OPT(c, points_already_global);

    auto handleVectorParam = [&](const std::string& key, auto& vec, const std::size_t dim)
    {
        if (c.has(key))
        {
            ASSERT_(c[key].isSequence() && c[key].asSequence().size() == dim);
            const auto& seq = c[key].asSequenceRange();

            for (size_t i = 0; i < dim; i++)
            {
                parseAndDeclareParameter(seq.at(i).as<std::string>(), vec[i]);
            }
        }
    };

    handleVectorParam("robot_pose", robot_pose, 6);

    handleVectorParam("bias_acc", bias_acc, 3);
    handleVectorParam("bias_gyro", bias_gyro, 3);
    handleVectorParam("gravity_vector", gravity_vector, 3);

    if (in_place)
    {
        ASSERTMSG_(
            output_pointcloud_layer.empty(),
            "If using `in_place=true`, `output_pointcloud_layer` cannot be defined.");
    }
    else
    {
        ASSERTMSG_(
            !output_pointcloud_layer.empty(),
            "If using `in_place=false`, `output_pointcloud_layer` must be defined.");
    }

    if (c.has("twist"))
    {
        twist.emplace();  // Define a constant twist model for deskewing

        handleVectorParam("twist", *twist, 6);
    }
}

// Minimal fake class for when building without IMU library:
#if !defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
namespace mola::imu
{
struct Trajectory
{
};
}  // namespace mola::imu
#endif

namespace
{
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)

auto findBeforeAfter(const mola::imu::Trajectory& trajectory, const double t)
    -> std::pair<mola::imu::Trajectory::const_iterator, mola::imu::Trajectory::const_iterator>
{
    using Iterator = mola::imu::Trajectory::const_iterator;

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
#endif  // MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION

struct CorrectPointsArguments
{
    mrpt::aligned_std_vector<float>&          xs;
    mrpt::aligned_std_vector<float>&          ys;
    mrpt::aligned_std_vector<float>&          zs;
    size_t                                    n;
    size_t                                    n0;
    mrpt::maps::CPointsMap*                   outPc;
    const mrpt::aligned_std_vector<float>*    Is;
    mrpt::aligned_std_vector<float>*          out_Is;
    const mrpt::aligned_std_vector<uint16_t>* Rs;
    mrpt::aligned_std_vector<uint16_t>*       out_Rs;
    const mrpt::aligned_std_vector<float>*    Ts;
    mrpt::aligned_std_vector<float>*          out_Ts;
    const mrpt::math::TTwist3D*               constant_twist;
    const mola::imu::Trajectory&              reconstructed_trajectory;
    bool                                      points_already_global;
    const mrpt::math::TPose3D&                global_robot_pose;
};

// Optimized templated version for compile-time optimization for each method
template <MotionCompensationMethod method>
void correctPointsLoop(const CorrectPointsArguments& args)
{
    // Capturing these bindings requires C++20.
    auto& [xs, ys, zs, n, n0, outPc, Is, out_Is, Rs, out_Rs, Ts, out_Ts, constant_twist, reconstructed_trajectory, points_already_global, global_robot_pose_t] =
        args;

    const auto global_robot_pose = mrpt::poses::CPose3D(global_robot_pose_t);

    const bool in_place = (outPc == nullptr);

    const bool has_I = Is != nullptr && out_Is != nullptr && !Is->empty();
    const bool has_R = Rs != nullptr && out_Rs != nullptr && !Rs->empty();
    const bool has_T = Ts != nullptr && out_Ts != nullptr && !Ts->empty();

    ASSERT_(Ts != nullptr);
    ASSERT_EQUAL_(Ts->size(), xs.size());

    // Is it worth to first build a cache with unique point stamps to pose corrections?
    // From initial benchmarking, it seems it doesn't...

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
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
            else if constexpr (method == MotionCompensationMethod::IMU)
            {
                const auto t_point    = (*Ts)[i];
                const auto [it0, it1] = findBeforeAfter(reconstructed_trajectory, t_point);

                const auto   t_prev  = it0->first;
                const double dt      = t_point - t_prev;
                const auto&  tp_prev = it0->second;

                if (std::abs(dt) > 0.1)
                {
                    thread_local double last_warning = mrpt::Clock::nowDouble();
                    const auto          tNow         = mrpt::Clock::nowDouble();
                    if (tNow - last_warning > 5.0)
                    {
                        last_warning = tNow;
                        fprintf(
                            stderr,
                            "[FilterDeskew|WARN]: Excessive time between point stamp and "
                            "trajectory point (dt=%.03f s)\n",
                            dt);
                    }
                }

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
            else if constexpr (method == MotionCompensationMethod::IMUt)
            {
                const auto t_point    = (*Ts)[i];
                const auto [it0, it1] = findBeforeAfter(reconstructed_trajectory, t_point);

                const auto   t_prev  = it0->first;
                const double dt      = t_point - t_prev;
                const auto&  tp_prev = it0->second;
                const auto&  tp_next = it1->second;

                // v was already in the t=0 frame of reference:
                const mrpt::math::TVector3D v_dt = (*tp_prev.v + *tp_next.v) * dt * 0.5;
                const mrpt::math::TVector3D w_dt = (*tp_prev.w_b + *tp_next.w_b) * dt * 0.5;

                pose_increment = mrpt::poses::CPose3D::FromRotationAndTranslation(
                    // Rotation: From Lie group SO(3) exponential:
                    tp_prev.pose.getRotationMatrix() *
                        mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt)),
                    // Translation: simple constant velocity model:
                    tp_prev.pose.translation() + v_dt);
            }
            else if constexpr (method == MotionCompensationMethod::IMUh)
            {
                const auto t_point    = (*Ts)[i];
                const auto [it0, it1] = findBeforeAfter(reconstructed_trajectory, t_point);

                const auto   t_prev  = it0->first;
                const double dt      = t_point - t_prev;
                const double dt2     = dt * dt;
                const double dt3     = dt2 * dt;
                const auto&  tp_prev = it0->second;

                // v was already in the t=0 frame of reference:
                const mrpt::math::TVector3D delta_t =
                    *tp_prev.v * dt +
                    tp_prev.pose.rotateVector(
                        tp_prev.ac_b.value() * 0.5 * dt2 + tp_prev.j_b * (1.0 / 6.0) * dt3);

                const mrpt::math::TVector3D delta_w =
                    *tp_prev.w_b * dt + tp_prev.alpha_b * 0.5 * dt2;

                pose_increment = mrpt::poses::CPose3D::FromRotationAndTranslation(
                    // Rotation: From Lie group SO(3) exponential:
                    tp_prev.pose.getRotationMatrix() *
                        mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(delta_w)),
                    // Translation: simple constant velocity model:
                    tp_prev.pose.translation() + delta_t);
            }
#endif
            else
            {
                // Should never arrive here
                THROW_EXCEPTION("Unhandled MotionCompensationMethod method (!)");
            }

            // Correct point XYZ coordinates.
            mrpt::math::TPoint3Df corrPt;

            if (!points_already_global)
            {
                // Simple case: points are local wrt the vehicle:
                corrPt = pose_increment.composePoint(pt).cast<float>();
            }
            else
            {
                // Points are already projected into global coordinates.
                // We must apply the deskew correction to the local point coordinates and project
                // again:
                const auto localPt = global_robot_pose.inverseComposePoint(pt.cast<double>());
                corrPt = (global_robot_pose + pose_increment).composePoint(localPt).cast<float>();
            }

            if (in_place)
            {
                xs[i] = corrPt.x;
                ys[i] = corrPt.y;
                zs[i] = corrPt.z;
            }
            else
            {
                outPc->setPointFast(n0 + i, corrPt.x, corrPt.y, corrPt.z);

                // Copy additional fields
                if (has_I)
                {
                    (*out_Is)[n0 + i] = (*Is)[i];
                }
                if (has_R)
                {
                    (*out_Rs)[n0 + i] = (*Rs)[i];
                }
                if (has_T)
                {
                    (*out_Ts)[n0 + i] = (*Ts)[i];
                }
            }
        }
#if defined(MP2P_HAS_TBB)
    );
#endif
}

}  // namespace

void FilterDeskew::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc;
    if (!in_place)
    {
        outPc = GetOrCreatePointLayer(
            inOut, output_pointcloud_layer, false /*dont allow empty names*/, output_layer_class);
    }

    // In:
    ASSERT_(!input_pointcloud_layer.empty());

    mrpt::maps::CPointsMap* inPc = nullptr;

    if (auto itLy = inOut.layers.find(input_pointcloud_layer); itLy != inOut.layers.end())
    {
        inPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!inPc)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.", input_pointcloud_layer.c_str());
        }

        // Reserve memory in output layer:
        if (!in_place)
        {
            outPc->reserve(outPc->size() + inPc->size());
        }
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

    MRPT_LOG_DEBUG_STREAM("Running on input layer: " << inPc->asString());

    // mandatory fields:
    auto&        xs = const_cast<mrpt::aligned_std_vector<float>&>(inPc->getPointsBufferRef_x());
    auto&        ys = const_cast<mrpt::aligned_std_vector<float>&>(inPc->getPointsBufferRef_y());
    auto&        zs = const_cast<mrpt::aligned_std_vector<float>&>(inPc->getPointsBufferRef_z());
    const size_t n  = xs.size();

    // optional fields:
#if MRPT_VERSION >= 0x020f00  // 2.15.0
    MRPT_TODO("Do a better generic re-factorization of this");
    const auto* Is = inPc->getPointsBufferRef_float_field("intensity");
    const auto* Ts = inPc->getPointsBufferRef_float_field("t");
    const auto* Rs = inPc->getPointsBufferRef_uint_field("ring");

    auto* out_Is = outPc ? outPc->getPointsBufferRef_float_field("intensity") : nullptr;
    auto* out_Ts = outPc ? outPc->getPointsBufferRef_float_field("t") : nullptr;
    auto* out_Rs = outPc ? outPc->getPointsBufferRef_uint_field("ring") : nullptr;

#else
    const auto* Is = inPc->getPointsBufferRef_intensity();
    const auto* Ts = inPc->getPointsBufferRef_timestamp();
    const auto* Rs = inPc->getPointsBufferRef_ring();

    auto* out_Is = outPc ? outPc->getPointsBufferRef_intensity() : nullptr;
    auto* out_Rs = outPc ? outPc->getPointsBufferRef_ring() : nullptr;
    auto* out_Ts = outPc ? outPc->getPointsBufferRef_timestamp() : nullptr;
#endif

    // No timestamps available or deskewing disabled:
    const bool noTimestamps   = !Ts || Ts->empty();
    const bool deskewDisabled = (method == MotionCompensationMethod::None);

    if (noTimestamps || deskewDisabled)
    {
        if (silently_ignore_no_timestamps || deskewDisabled)
        {
            if (!in_place)
            {
                outPc->insertAnotherMap(inPc, mrpt::poses::CPose3D::Identity());
            }

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
    const size_t n0 = outPc ? outPc->size() : 0;
    if (outPc)
    {
        outPc->resize(n0 + n);
    }

    // Used for precise deskew-only. This contains relative poses of the vehicle frame ("base_link")
    // with t=0 being the reference time when t=0 in the point cloud timestamp field:
    mola::imu::Trajectory reconstructed_trajectory;

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    mola::imu::ImuIntegrationParams imu_params;
    imu_params.bias_acc       = bias_acc;
    imu_params.bias_gyro      = bias_gyro;
    imu_params.gravity_vector = gravity_vector;
#endif

    const mrpt::math::TTwist3D* constant_twist = nullptr;

    switch (method)
    {
        case MotionCompensationMethod::IMU:
        case MotionCompensationMethod::IMUh:
        case MotionCompensationMethod::IMUt:
        {
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
            const auto* ps = attachedSource();
            ASSERTMSG_(ps, "A ParameterSource must be attached if IMU-based methods are enabled");

            const auto [it_min, it_max] = std::minmax_element(Ts->cbegin(), Ts->cend());
            ASSERT_(it_min != Ts->cend());
            ASSERT_(it_max != Ts->cend());

            const double scan_time_span = *it_max - *it_min;

            // Recall, the reference time should have been set already by the Generator and/or
            // FilterAdjustTimestamps:
            const auto sample_history =
                ps->localVelocityBuffer.collect_samples_around_reference_time(2 * scan_time_span);

            const bool use_higher_order = (method == MotionCompensationMethod::IMUh);

            reconstructed_trajectory =
                mola::imu::trajectory_from_buffer(sample_history, imu_params, use_higher_order);

#if 0  // For *really* in-depth debugging
            std::cout << "\n\n ==== INPUT BUFFER:\n" << ps->localVelocityBuffer.toYAML();
            std::cout << "\n\n ==== INPUT SAMPLES:\n" << sample_history.by_type.asString();
            std::cout << "\n\n ==== RECONSTRUCTED:\n"
                      << mola::imu::trajectory_as_string(reconstructed_trajectory);
            std::cout << "\n\n\n ------------------------------------------ \n";
#endif

#else
            THROW_EXCEPTION(
                "Only Linear deskew method is available when building mp2p_icp without "
                "mola_imu_preintegration");
#endif
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

    // Prepare call arguments:
    const CorrectPointsArguments args = {
        xs,
        ys,
        zs,
        n,
        n0,
        outPc.get(),
        Is,
        out_Is,
        Rs,
        out_Rs,
        Ts,
        out_Ts,
        constant_twist,
        reconstructed_trajectory,
        points_already_global,
        robot_pose};

    // compile-time optimized code for each method:
    switch (method)
    {
        case MotionCompensationMethod::Linear:
            correctPointsLoop<MotionCompensationMethod::Linear>(args);
            break;

        case MotionCompensationMethod::IMU:
            correctPointsLoop<MotionCompensationMethod::IMU>(args);
            break;

        case MotionCompensationMethod::IMUh:
            correctPointsLoop<MotionCompensationMethod::IMUh>(args);
            break;

        case MotionCompensationMethod::IMUt:
            correctPointsLoop<MotionCompensationMethod::IMUt>(args);
            break;

        default:
            // Should never arrive here
            throw std::runtime_error("Unknown MotionCompensationMethod method");
    }

    MRPT_END
}
