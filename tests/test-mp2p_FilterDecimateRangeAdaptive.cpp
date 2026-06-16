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
 * @file   test-mp2p_FilterDecimateRangeAdaptive.cpp
 * @brief  Unit tests for FilterDecimateRangeAdaptive
 * @author Jose Luis Blanco Claraco
 * @date   Jun 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterByRing.h>
#include <mp2p_icp_filters/FilterDecimateRangeAdaptive.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;

// Helper: count output points in shells between r_min and r_max
static size_t count_in_shell(const mrpt::maps::CPointsMap& pc, float r_min, float r_max)
{
    const auto& xs    = pc.getPointsBufferRef_x();
    const auto& ys    = pc.getPointsBufferRef_y();
    const auto& zs    = pc.getPointsBufferRef_z();
    size_t      count = 0;
    for (size_t i = 0; i < xs.size(); i++)
    {
        const float r = std::sqrt(xs[i] * xs[i] + ys[i] * ys[i] + zs[i] * zs[i]);
        if (r >= r_min && r < r_max)
        {
            count++;
        }
    }
    return count;
}

// Test 1: Near shells have higher output density (points/m^2) than far shells.
// The EllipseLIO filter uses smaller voxels near the sensor and larger ones far away,
// so the surface-area-normalized output density should be higher near than far.
static void test_near_denser_than_far()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();

    const float r_near = 5.0f;
    const float r_far  = 20.0f;

    // Seed both shells with a uniform angular grid (same angular density = same
    // physical density per unit solid angle).
    const int N_az = 200;
    const int N_el = 60;
    for (int ia = 0; ia < N_az; ia++)
    {
        for (int ie = 0; ie < N_el; ie++)
        {
            const float az = ia * (2.0f * 3.14159f / N_az);
            const float el = -0.5f + ie * (1.0f / (N_el - 1));

            const float cosEl = std::cos(el);

            // Near shell
            pc->insertPointFast(
                r_near * cosEl * std::cos(az), r_near * cosEl * std::sin(az),
                r_near * std::sin(el));

            // Far shell
            pc->insertPointFast(
                r_far * cosEl * std::cos(az), r_far * cosEl * std::sin(az), r_far * std::sin(el));
        }
    }

    std::cout << "Shell density test input cloud: " << pc->size() << " points\n";

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    FilterDecimateRangeAdaptive filter;
    mrpt::containers::yaml      p;
    p["input_pointcloud_layer"]  = "raw";
    p["output_pointcloud_layer"] = "out";
    p["vertical_fov_rad"]        = 1.0;  // ~57 deg
    p["num_scan_lines"]          = 64;
    p["bin_width"]               = 1.0;
    p["min_voxel_size"]          = 0.01;
    p["max_voxel_size"]          = 5.0;

    filter.initialize(p);
    filter.filter(map);

    auto outPtr = map.layer<mrpt::maps::CPointsMap>("out");
    ASSERT_(outPtr);

    const size_t near_count = count_in_shell(*outPtr, r_near - 1.0f, r_near + 1.0f);
    const size_t far_count  = count_in_shell(*outPtr, r_far - 1.0f, r_far + 1.0f);

    // Normalize by surface area (4*pi*r^2) to get point density:
    const double near_density = static_cast<double>(near_count) / (r_near * r_near);
    const double far_density  = static_cast<double>(far_count) / (r_far * r_far);

    std::cout << "Near shell (r=" << r_near << "m): " << near_count
              << " pts, density=" << near_density << "\n";
    std::cout << "Far  shell (r=" << r_far << "m): " << far_count << " pts, density=" << far_density
              << "\n";

    // Near voxel size = bin_i * fov / (lines-1) with bin_i ~5 => v ~ 5*1.0/63 = 0.079 m
    // Far  voxel size => bin_i ~20 => v ~ 20*1.0/63 = 0.317 m
    // Area per voxel near: ~0.079^2 = 0.006 m^2 => density ~166 pts/m^2
    // Area per voxel far:  ~0.317^2 = 0.100 m^2 => density ~10 pts/m^2
    // So near density >> far density.
    ASSERTMSG_(
        near_density > far_density * 2.0,
        "Expected significantly higher output density in near shell than far shell");

    std::cout << "[Test Passed] Near shell has higher point density than far shell\n";
}

// Test 2: auto-derive beta/theta from ring channel.
static void test_auto_derive_from_ring()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_uint16(mp2p_icp_filters::POINT_FIELD_RING_ID);

    // 32-ring sensor, FOV ~ 40 deg
    const unsigned int RINGS    = 32;
    const float        fov_half = 0.349f;  // 20 deg

    for (unsigned int ring = 0; ring < RINGS; ring++)
    {
        const float el = -fov_half + ring * (2.0f * fov_half / (RINGS - 1));
        for (int az = 0; az < 100; az++)
        {
            const float azimuth = az * 0.0628f;
            const float range   = 10.0f + ring * 0.2f;
            const float cosEl   = std::cos(el);
            const float x       = range * cosEl * std::cos(azimuth);
            const float y       = range * cosEl * std::sin(azimuth);
            const float z       = range * std::sin(el);
            pc->insertPoint(x, y, z);
            pc->insertPointField_uint16(
                mp2p_icp_filters::POINT_FIELD_RING_ID, static_cast<uint16_t>(ring));
        }
    }

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    FilterDecimateRangeAdaptive filter;
    mrpt::containers::yaml      p;
    p["input_pointcloud_layer"]  = "raw";
    p["output_pointcloud_layer"] = "out";
    p["vertical_fov_rad"]        = 0;  // auto
    p["num_scan_lines"]          = 0;  // auto
    p["bin_width"]               = 1.0;
    p["min_voxel_size"]          = 0.01;
    p["max_voxel_size"]          = 5.0;

    filter.initialize(p);
    filter.filter(map);

    auto outPtr = map.layer<mrpt::maps::CPointsMap>("out");
    ASSERT_(outPtr);

    const size_t out_size = outPtr->size();
    std::cout << "Auto-derive test: input=" << pc->size() << " output=" << out_size << "\n";

    // Must produce some output, and fewer than input
    ASSERT_GT_(out_size, 0u);
    ASSERT_LT_(out_size, pc->size());

    std::cout << "[Test Passed] Auto-derive ring/FOV decimation\n";
}

// Test 3: explicit params, output <= input
static void test_output_never_exceeds_input()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (int i = 0; i < 5000; i++)
    {
        pc->insertPointFast(
            static_cast<float>(i % 70) * 0.15f, static_cast<float>((i / 70) % 70) * 0.15f,
            static_cast<float>(i % 20) * 0.5f);
    }

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    FilterDecimateRangeAdaptive filter;
    mrpt::containers::yaml      p;
    p["input_pointcloud_layer"]  = "raw";
    p["output_pointcloud_layer"] = "out";
    p["vertical_fov_rad"]        = 0.5;
    p["num_scan_lines"]          = 32;
    p["bin_width"]               = 2.0;
    p["min_voxel_size"]          = 0.05;
    p["max_voxel_size"]          = 3.0;

    filter.initialize(p);
    filter.filter(map);

    auto outPtr = map.layer<mrpt::maps::CPointsMap>("out");
    ASSERT_(outPtr);

    ASSERT_LE_(outPtr->size(), pc->size());
    std::cout << "Output " << outPtr->size() << " <= input " << pc->size() << "\n";
    std::cout << "[Test Passed] Output never exceeds input\n";
}

// Test 4: min_input_points_per_voxel drops sparse voxels
static void test_min_input_points_per_voxel()
{
    // Build a cloud with two regions:
    //   - "dense" zone: many points packed into the same voxels at ~5 m range
    //   - "sparse" zone: one point per voxel at ~15 m range
    // With min_input_points_per_voxel=10, the sparse zone should be dropped.
    auto pc = mrpt::maps::CSimplePointsMap::Create();

    // Dense zone: 20x20 grid of points, 3 cm spacing => ~400 points, all map to
    // the same voxel neighbourhood at 5 m (voxel ~ 5*1.0/63 ~ 0.08 m, so many
    // of these will share a voxel).
    for (int ia = 0; ia < 20; ia++)
    {
        for (int ib = 0; ib < 20; ib++)
        {
            pc->insertPointFast(
                5.0f + ia * 0.01f,  // tightly clustered in x
                ib * 0.01f,  // tight y spread
                0.0f);
        }
    }

    // Sparse zone: 1 isolated point per voxel at 15 m, spread far apart so each
    // ends up in its own voxel.
    for (int i = 0; i < 30; i++)
    {
        pc->insertPointFast(15.0f + i * 1.0f, 0.0f, 0.0f);
    }

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    FilterDecimateRangeAdaptive filter;
    mrpt::containers::yaml      p;
    p["input_pointcloud_layer"]     = "raw";
    p["output_pointcloud_layer"]    = "out";
    p["vertical_fov_rad"]           = 1.0;
    p["num_scan_lines"]             = 64;
    p["bin_width"]                  = 1.0;
    p["min_voxel_size"]             = 0.05;
    p["max_voxel_size"]             = 5.0;
    p["min_input_points_per_voxel"] = 10u;

    filter.initialize(p);
    filter.filter(map);

    auto outPtr = map.layer<mrpt::maps::CPointsMap>("out");
    ASSERT_(outPtr);

    const size_t dense_out  = count_in_shell(*outPtr, 4.0f, 6.0f);
    const size_t sparse_out = count_in_shell(*outPtr, 14.0f, 50.0f);

    std::cout << "min_input_points_per_voxel=10 test: dense_out=" << dense_out
              << " sparse_out=" << sparse_out << "\n";

    ASSERTMSG_(dense_out > 0, "Dense zone must produce output points");
    ASSERTMSG_(sparse_out == 0, "Sparse zone (1 pt/voxel) must be fully suppressed");

    std::cout << "[Test Passed] min_input_points_per_voxel filters sparse voxels\n";
}

int main()
{
    try
    {
        test_near_denser_than_far();
        test_auto_derive_from_ring();
        test_output_never_exceeds_input();
        test_min_input_points_per_voxel();

        std::cout << "\nAll FilterDecimateRangeAdaptive tests passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
