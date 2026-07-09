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
 * @file   test-mp2p_deskew_empty_buffer.cpp
 * @brief  Regression test: FilterDeskew must not throw when the local
 *         velocity buffer yields an empty trajectory (e.g., transient
 *         sensor data gap). It should bypass deskew and pass points
 *         through unchanged.
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDeskew.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <cstdio>
#include <iostream>

namespace
{
mrpt::maps::CGenericPointsMap::Ptr make_skewed_scan(std::size_t n_points, double scan_period)
{
    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->registerField_float(mp2p_icp_filters::POINT_FIELD_TIMESTAMP);
    pts->registerField_float(mp2p_icp_filters::POINT_FIELD_INTENSITY);

    for (std::size_t i = 0; i < n_points; ++i)
    {
        const float x        = 5.0f + 0.1f * static_cast<float>(i);
        const float y        = -3.0f + 0.05f * static_cast<float>(i);
        const float z        = 0.5f;
        const float rel_time = static_cast<float>(
            scan_period * static_cast<double>(i) /
            static_cast<double>(std::max<std::size_t>(1U, n_points - 1)));

        pts->insertPointFast(x, y, z);
        pts->insertPointField_float(mp2p_icp_filters::POINT_FIELD_TIMESTAMP, rel_time);
        pts->insertPointField_float(mp2p_icp_filters::POINT_FIELD_INTENSITY, 1.0f);
    }
    return pts;
}

[[nodiscard]] bool run_one(mp2p_icp_filters::MotionCompensationMethod method)
{
    using namespace mp2p_icp_filters;

    const std::size_t n_points    = 32;
    const double      scan_period = 0.1;
    const auto        skewed      = make_skewed_scan(n_points, scan_period);

    // Snapshot expected XYZ (must remain unchanged after deskew bypass)
    std::vector<mrpt::math::TPoint3Df> expected;
    expected.reserve(n_points);
    for (std::size_t i = 0; i < skewed->size(); ++i)
    {
        mrpt::math::TPoint3Df p;
        skewed->getPoint(i, p.x, p.y, p.z);
        expected.push_back(p);
    }

    // Attach a ParameterSource with an INTENTIONALLY EMPTY localVelocityBuffer.
    // This mimics the situation where a LiDAR scan arrives after a sensor data
    // gap: no IMU samples are available around the scan's reference time.
    mp2p_icp::ParameterSource ps;

    FilterDeskew deskew;
    deskew.silently_ignore_no_timestamps = false;
    deskew.input_pointcloud_layer        = "raw";
    deskew.output_pointcloud_layer       = "deskewed";
    deskew.method                        = method;
    deskew.output_layer_class            = "mrpt::maps::CGenericPointsMap";
    deskew.attachToParameterSource(ps);

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = skewed;

    try
    {
        deskew.filter(m);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[FAIL] Filter threw with empty velocity buffer (method="
                  << mrpt::typemeta::enum2str(method) << "): " << e.what() << "\n";
        return false;
    }

    const auto out = m.point_layer("deskewed");
    if (!out)
    {
        std::cerr << "[FAIL] No 'deskewed' layer produced.\n";
        return false;
    }
    if (out->size() != n_points)
    {
        std::cerr << "[FAIL] Output size " << out->size() << " != input size " << n_points << "\n";
        return false;
    }

    // With an empty trajectory, the filter must pass points through unchanged.
    for (std::size_t i = 0; i < out->size(); ++i)
    {
        mrpt::math::TPoint3Df p;
        out->getPoint(i, p.x, p.y, p.z);
        const auto err = (p - expected[i]).norm();
        if (err > 1e-5f)
        {
            std::cerr << "[FAIL] Point " << i << " modified (err=" << err
                      << ") despite empty velocity buffer.\n";
            return false;
        }
    }

    std::cout << "[OK ] empty-buffer bypass for method=" << mrpt::typemeta::enum2str(method)
              << "\n";
    return true;
}

}  // namespace

int main()
{
    int failures = 0;

    // Linear with no twist defined would be a misconfiguration; not tested here.
    // The IMU* methods are the ones that previously threw the hard assert.
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    if (!run_one(mp2p_icp_filters::MotionCompensationMethod::IMU))
    {
        ++failures;
    }
    if (!run_one(mp2p_icp_filters::MotionCompensationMethod::IMUh))
    {
        ++failures;
    }
    if (!run_one(mp2p_icp_filters::MotionCompensationMethod::IMUt))
    {
        ++failures;
    }
#else
    std::cout << "Skipping test: built without mola_imu_preintegration.\n";
#endif

    std::printf("Number of test failures: %i\n", failures);
    return failures == 0 ? 0 : 1;
}
