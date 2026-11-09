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
 * @file   test-mp2p_FilterAdjustTimestamps.cpp
 * @brief  Unit test for FilterAdjustTimestamps, including the time offset
 *         used to merge clouds from several sensors
 * @author Jose Luis Blanco Claraco
 * @date   Nov 2026
 */

#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterAdjustTimestamps.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace mp2p_icp_filters;

namespace
{
// Per-point times, relative to each cloud's own timestamp (one sweep):
const std::vector<float> kRawTimes = {0.0f, 0.03f, 0.05f, 0.1f};

mrpt::maps::CGenericPointsMap::Ptr makeCloud()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("t");
    for (const float t : kRawTimes)
    {
        pc->insertPointFast(1.0f, 2.0f, 3.0f);
        pc->insertPointField_float("t", t);
    }
    pc->mark_as_modified();
    return pc;
}

struct Adjusted
{
    std::vector<float> t;  //!< Adjusted per-point times
    double             refTime = 0;  //!< Reference zero time after the filter
};

// Runs the filter on a cloud stamped at `stamp`, with `offset` = stamp minus
// the common reference stamp, as a multi-sensor pipeline does.
Adjusted adjust(TimestampAdjustMethod method, double stamp, double offset)
{
    mp2p_icp::ParameterSource context;
    context.updateVariable("SENSOR_TIME_OFFSET", offset);

    FilterAdjustTimestamps filter;
    mrpt::containers::yaml p;
    p["pointcloud_layer"]              = "raw";
    p["silently_ignore_no_timestamps"] = false;
    p["method"]                        = mrpt::typemeta::enum2str(method);
    p["time_offset"]                   = "SENSOR_TIME_OFFSET";
    filter.initialize(p);
    filter.attachToParameterSource(context);
    context.realize();

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    context.localVelocityBuffer.set_reference_zero_time(stamp);
#endif

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = makeCloud();
    filter.filter(map);

    Adjusted    out;
    const auto  pc = map.layer<mrpt::maps::CPointsMap>("raw");
    const auto* Ts = pc->getPointsBufferRef_float_field("t");
    ASSERT_(Ts != nullptr);
    out.t.assign(Ts->begin(), Ts->end());
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    out.refTime = context.localVelocityBuffer.get_reference_zero_time();
#else
    out.refTime = stamp;
#endif
    return out;
}

void test_two_sensors(TimestampAdjustMethod method)
{
    const double tA    = 1000.0;
    const double delta = 0.074;  // the second sensor fires later
    const double tB    = tA + delta;

    const auto a = adjust(method, tA, 0.0);
    const auto b = adjust(method, tB, delta);

    ASSERT_EQUAL_(a.t.size(), kRawTimes.size());
    ASSERT_EQUAL_(b.t.size(), kRawTimes.size());

    for (size_t i = 0; i < kRawTimes.size(); i++)
    {
        // Same raw time within each sweep: B's point fired `delta` later.
        const double d = static_cast<double>(b.t[i]) - static_cast<double>(a.t[i]);
        ASSERTMSG_(
            std::abs(d - delta) < 1e-5, mrpt::format(
                                            "method=%s point %zu: time difference %f, expected %f",
                                            mrpt::typemeta::enum2str(method).c_str(), i, d, delta));
    }

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    // Both clouds end up relative to one common reference time, and every
    // point keeps its absolute time:
    ASSERT_NEAR_(a.refTime, b.refTime, 1e-5);
    for (size_t i = 0; i < kRawTimes.size(); i++)
    {
        ASSERT_NEAR_(a.refTime + a.t[i], tA + kRawTimes[i], 1e-5);
        ASSERT_NEAR_(b.refTime + b.t[i], tB + kRawTimes[i], 1e-5);
    }
#endif
}

void test_single_sensor_middle_is_zero()
{
    const auto a = adjust(TimestampAdjustMethod::MiddleIsZero, 0.0, 0.0);
    // Middle of [0, 0.1] is 0.05:
    for (size_t i = 0; i < kRawTimes.size(); i++)
    {
        ASSERT_NEAR_(a.t[i], kRawTimes[i] - 0.05f, 1e-6f);
    }
}

}  // namespace

int main()
{
    try
    {
        test_single_sensor_middle_is_zero();
        test_two_sensors(TimestampAdjustMethod::MiddleIsZero);
        test_two_sensors(TimestampAdjustMethod::EarliestIsZero);
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
