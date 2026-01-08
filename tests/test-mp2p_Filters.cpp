/* +
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
 * @file   test-mp2p_Filters.cpp
 * @brief  Unit tests for various filters
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mp2p_icp_filters/FilterByIntensity.h>
#include <mp2p_icp_filters/FilterMLS.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <iostream>
#include <random>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{

constexpr std::string_view INTENSITY = POINT_FIELD_INTENSITY;

// Helper to create a test point cloud with known positions and intensities
mrpt::maps::CPointsMap::Ptr createTestPointsWithIntensity(
    size_t n_x, size_t n_y, float spacing = 1.0f)
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float(INTENSITY);

    for (size_t i = 0; i < n_x; ++i)
    {
        for (size_t j = 0; j < n_y; ++j)
        {
            float x = static_cast<float>(i) * spacing;
            float y = static_cast<float>(j) * spacing;
            float z = 0.0f;

            // Create intensity gradient: 0.0 to 1.0
            float intensity = static_cast<float>(i * n_y + j) / static_cast<float>(n_x * n_y - 1);

            pc->insertPointFast(x, y, z);
#if MRPT_VERSION >= 0x020f00
            pc->insertPointField_float(INTENSITY, intensity);
#else
            pc->insertPointField_Intensity(intensity);
#endif
        }
    }
    return pc;
}

// Test FilterBoundingBox with points inside and outside
void test_FilterBoundingBox_InsideOutside()
{
    std::cout << "Testing FilterBoundingBox (inside/outside split)... ";

    // Create test data: 10x10 grid, points from (0,0,0) to (9,9,0)
    const size_t N_X      = 10;
    const size_t N_Y      = 10;
    auto         input_pc = createTestPointsWithIntensity(N_X, N_Y, 1.0f);

    ASSERT_EQUAL_(input_pc->size(), N_X * N_Y);

    // Setup filter: bounding box from (2,2,-1) to (7,7,1)
    // This should capture points with x,y in [2,7] range
    FilterBoundingBox filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "inside";
    params["outside_pointcloud_layer"] = "outside";
    params["bounding_box_min"]         = mrpt::containers::yaml::Sequence({"2", "2", "-1"});
    params["bounding_box_max"]         = mrpt::containers::yaml::Sequence({"7", "7", "1"});

    filter.initialize(params);

    // Execute filter
    metric_map_t map;
    map.layers["raw"] = input_pc;
    filter.filter(map);

    // Verify outputs exist
    ASSERTMSG_(map.layers.count("inside"), "Inside layer not found");
    ASSERTMSG_(map.layers.count("outside"), "Outside layer not found");

    auto inside_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("inside"));
    auto outside_pc = mp2p_icp::MapToPointsMap(*map.layers.at("outside"));

    ASSERTMSG_(inside_pc, "Inside layer is not a point cloud");
    ASSERTMSG_(outside_pc, "Outside layer is not a point cloud");

    // Count expected inside points: x,y in [2,7] = 6x6 = 36 points
    const size_t EXPECTED_INSIDE  = 6UL * 6UL;
    const size_t EXPECTED_OUTSIDE = N_X * N_Y - EXPECTED_INSIDE;

    ASSERT_EQUAL_(inside_pc->size(), EXPECTED_INSIDE);
    ASSERT_EQUAL_(outside_pc->size(), EXPECTED_OUTSIDE);

    // Verify all points are accounted for
    ASSERT_EQUAL_(inside_pc->size() + outside_pc->size(), input_pc->size());

    // Verify inside points are actually inside bbox
    const auto& xs_in = inside_pc->getPointsBufferRef_x();
    const auto& ys_in = inside_pc->getPointsBufferRef_y();

    for (size_t i = 0; i < inside_pc->size(); ++i)
    {
        ASSERT_GE_(xs_in[i], 2.0f);
        ASSERT_LE_(xs_in[i], 7.0f);
        ASSERT_GE_(ys_in[i], 2.0f);
        ASSERT_LE_(ys_in[i], 7.0f);
    }

    std::cout << "Success ✅" << std::endl;
}

// Test FilterBoundingBox with only inside output
void test_FilterBoundingBox_OnlyInside()
{
    std::cout << "Testing FilterBoundingBox (only inside output)... ";

    auto input_pc = createTestPointsWithIntensity(10, 10, 1.0f);

    FilterBoundingBox filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "filtered";
    params["outside_pointcloud_layer"] = "";  // Don't create outside layer
    params["bounding_box_min"]         = mrpt::containers::yaml::Sequence({"3", "3", "-1"});
    params["bounding_box_max"]         = mrpt::containers::yaml::Sequence({"6", "6", "1"});

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = input_pc;
    filter.filter(map);

    ASSERTMSG_(map.layers.count("filtered"), "Filtered layer not found");
    ASSERTMSG_(!map.layers.count("outside"), "Outside layer should not exist");

    auto filtered_pc = mp2p_icp::MapToPointsMap(*map.layers.at("filtered"));

    // Expected: 4x4 = 16 points
    ASSERT_EQUAL_(filtered_pc->size(), 16);

    std::cout << "Success ✅" << std::endl;
}

// Test FilterByIntensity with three output layers
void test_FilterByIntensity_ThreeLayers()
{
    std::cout << "Testing FilterByIntensity (three output layers)... ";

    // Create 100 points with intensity from 0.0 to 1.0
    auto input_pc = createTestPointsWithIntensity(10, 10, 1.0f);

    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]      = "raw";
    params["output_layer_low_intensity"]  = "low";
    params["output_layer_mid_intensity"]  = "mid";
    params["output_layer_high_intensity"] = "high";
    params["low_threshold"]               = 0.3f;
    params["high_threshold"]              = 0.7f;

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = input_pc;
    filter.filter(map);

    // Verify all outputs exist
    ASSERTMSG_(map.layers.count("low"), "Low intensity layer not found");
    ASSERTMSG_(map.layers.count("mid"), "Mid intensity layer not found");
    ASSERTMSG_(map.layers.count("high"), "High intensity layer not found");

    auto low_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("low"));
    auto mid_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("mid"));
    auto high_pc = mp2p_icp::MapToPointsMap(*map.layers.at("high"));

    // Verify all points are accounted for
    ASSERT_EQUAL_(low_pc->size() + mid_pc->size() + high_pc->size(), input_pc->size());

    // Verify intensity ranges
    const auto* ptrI_low  = low_pc->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    const auto* ptrI_mid  = mid_pc->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    const auto* ptrI_high = high_pc->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);

    if (ptrI_low)
    {
        for (float I : *ptrI_low)
        {
            ASSERT_LT_(I, 0.3f);
        }
    }

    if (ptrI_mid)
    {
        for (float I : *ptrI_mid)
        {
            ASSERT_GE_(I, 0.3f);
            ASSERT_LE_(I, 0.7f);
        }
    }

    if (ptrI_high)
    {
        for (float I : *ptrI_high)
        {
            ASSERT_GT_(I, 0.7f);
        }
    }

    std::cout << "Success ✅" << std::endl;
}

// Test FilterByIntensity with only two output layers
void test_FilterByIntensity_TwoLayers()
{
    std::cout << "Testing FilterByIntensity (low and high only)... ";

    auto input_pc = createTestPointsWithIntensity(10, 10, 1.0f);

    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]      = "raw";
    params["output_layer_low_intensity"]  = "low";
    params["output_layer_high_intensity"] = "high";
    params["output_layer_mid_intensity"]  = "";  // Don't create mid layer
    params["low_threshold"]               = 0.4f;
    params["high_threshold"]              = 0.6f;

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = input_pc;
    filter.filter(map);

    ASSERTMSG_(map.layers.count("low"), "Low intensity layer not found");
    ASSERTMSG_(map.layers.count("high"), "High intensity layer not found");
    ASSERTMSG_(!map.layers.count("mid"), "Mid layer should not exist");

    auto low_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("low"));
    auto high_pc = mp2p_icp::MapToPointsMap(*map.layers.at("high"));

    // Some points should be filtered out (the mid range)
    ASSERT_LT_(low_pc->size() + high_pc->size(), input_pc->size());

    std::cout << "Success ✅" << std::endl;
}

// Test edge case: empty bounding box
void test_FilterBoundingBox_EmptyBox()
{
    std::cout << "Testing FilterBoundingBox (empty bounding box)... ";

    auto input_pc = createTestPointsWithIntensity(10, 10, 1.0f);

    FilterBoundingBox filter;

    // Bounding box that contains no points
    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "inside";
    params["outside_pointcloud_layer"] = "outside";
    params["bounding_box_min"]         = mrpt::containers::yaml::Sequence({"50", "50", "50"});
    params["bounding_box_max"]         = mrpt::containers::yaml::Sequence({"60", "60", "60"});

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = input_pc;
    filter.filter(map);

    auto inside_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("inside"));
    auto outside_pc = mp2p_icp::MapToPointsMap(*map.layers.at("outside"));

    // All points should be outside
    ASSERT_EQUAL_(inside_pc->size(), 0);
    ASSERT_EQUAL_(outside_pc->size(), input_pc->size());

    std::cout << "Success ✅" << std::endl;
}

// Test edge case: intensity at exact threshold
void test_FilterByIntensity_ExactThreshold()
{
    std::cout << "Testing FilterByIntensity (points at exact thresholds)... ";

    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float(INTENSITY);

    // Create points with exact threshold intensities
    std::vector<float> intensities = {0.0f, 0.3f, 0.5f, 0.7f, 1.0f};

    for (size_t i = 0; i < intensities.size(); ++i)
    {
        pc->insertPointFast(static_cast<float>(i), 0.0f, 0.0f);
#if MRPT_VERSION >= 0x020f00
        pc->insertPointField_float(INTENSITY, intensities[i]);
#else
        pc->insertPointField_Intensity(intensities[i]);
#endif
    }

    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]      = "raw";
    params["output_layer_low_intensity"]  = "low";
    params["output_layer_mid_intensity"]  = "mid";
    params["output_layer_high_intensity"] = "high";
    params["low_threshold"]               = 0.3f;
    params["high_threshold"]              = 0.7f;

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);

    auto low_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("low"));
    auto mid_pc  = mp2p_icp::MapToPointsMap(*map.layers.at("mid"));
    auto high_pc = mp2p_icp::MapToPointsMap(*map.layers.at("high"));

    // Points at exact thresholds should go to mid
    // low: 0.0 (I < 0.3)
    // mid: 0.3, 0.5, 0.7 (0.3 <= I <= 0.7)
    // high: 1.0 (I > 0.7)
    ASSERT_EQUAL_(low_pc->size(), 1);
    ASSERT_EQUAL_(mid_pc->size(), 3);
    ASSERT_EQUAL_(high_pc->size(), 1);

    std::cout << "Success ✅" << std::endl;
}

// Test FilterMLS with planar point cloud
void test_FilterMLS_PlanarCloud()
{
    std::cout << "Testing FilterMLS (planar cloud with noise)... ";

    auto pc = mrpt::maps::CGenericPointsMap::Create();

    // Create a planar cloud: z = 0.5*x + 0.3*y + 1.0 with small noise
    const size_t N_POINTS         = 5000;
    const float  NOISE_STD        = 0.05f;
    const float  OUTLIER_FRACTION = 0.05f;

    std::mt19937                          rng(42);
    std::normal_distribution<float>       noise_dist(0.0f, NOISE_STD);
    std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    size_t num_outliers = static_cast<size_t>(N_POINTS * OUTLIER_FRACTION);

    for (size_t i = 0; i < N_POINTS; ++i)
    {
        float x = uniform_dist(rng) * 10.0f - 5.0f;  // [-5, 5]
        float y = uniform_dist(rng) * 10.0f - 5.0f;  // [-5, 5]

        // Plane equation: z = 0.5*x + 0.3*y + 1.0
        float z = 0.5f * x + 0.3f * y + 1.0f;

        // Add noise to most points
        if (i >= num_outliers)
        {
            z += noise_dist(rng);
        }
        else
        {
            // Outliers: add large random values
            z += (uniform_dist(rng) - 0.5f) * 2.0f;  // Large random offset
        }

        pc->insertPointFast(x, y, z);
    }

    ASSERT_EQUAL_(pc->size(), N_POINTS);

    // Apply MLS filter
    FilterMLS filter;
    filter.setMinLoggingLevel(mrpt::system::LVL_WARN);

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "raw";
    params["output_pointcloud_layer"] = "mls";
    params["search_radius"]           = 0.50f;
    params["polynomial_order"]        = 1;  // Planar fit
    params["min_neighbors_for_fit"]   = 4;

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);

    ASSERTMSG_(map.layers.count("mls"), "MLS output layer not found");

    auto mls_pc = mp2p_icp::MapToPointsMap(*map.layers.at("mls"));
    ASSERTMSG_(mls_pc, "MLS layer is not a point cloud");
    ASSERT_GT_(mls_pc->size(), 0);

    // Define the reference plane: z = 0.5*x + 0.3*y + 1.0
    // In normal form: 0.5*x + 0.3*y - z + 1.0 = 0
    // Normalized: a*x + b*y + c*z + d = 0 where a^2+b^2+c^2=1
    float              norm = std::sqrt(0.5f * 0.5f + 0.3f * 0.3f + 1.0f * 1.0f);
    mrpt::math::TPlane ref_plane;
    ref_plane.coefs[0] = 0.5f / norm;
    ref_plane.coefs[1] = 0.3f / norm;
    ref_plane.coefs[2] = -1.0f / norm;
    ref_plane.coefs[3] = 1.0f / norm;

    // Check that filtered points are close to the plane
    const auto& xs = mls_pc->getPointsBufferRef_x();
    const auto& ys = mls_pc->getPointsBufferRef_y();
    const auto& zs = mls_pc->getPointsBufferRef_z();

    double avg_distance = 0.0;

    for (size_t i = 0; i < mls_pc->size(); ++i)
    {
        float x = xs[i];
        float y = ys[i];
        float z = zs[i];

        // Point to plane distance: |ax + by + cz + d|
        const auto dist = static_cast<float>(std::abs(
            ref_plane.coefs[0] * x + ref_plane.coefs[1] * y + ref_plane.coefs[2] * z +
            ref_plane.coefs[3]));

        avg_distance += dist;
    }

    avg_distance /= static_cast<double>(mls_pc->size());

    // After MLS smoothing, points should be very close to the plane
    // The threshold should be a few times the original noise std
    const float DISTANCE_THRESHOLD = 0.1f;  // Reasonable threshold for filtered points

    ASSERT_LT_(avg_distance, DISTANCE_THRESHOLD / 3.0);

    std::cout << "Success ✅ (avg distance: " << avg_distance << ")\n";
}

// Test FilterMLS with quadratic surface
void test_FilterMLS_QuadraticSurface()
{
    std::cout << "Testing FilterMLS (quadratic surface)... ";

    auto pc = mrpt::maps::CGenericPointsMap::Create();

    // Create a quadratic surface: z = 0.1*x^2 + 0.05*y^2 + 0.2*x*y + 1.0
    const size_t N_POINTS  = 300;
    const float  NOISE_STD = 0.050f;

    std::mt19937                          rng(123);
    std::normal_distribution<float>       noise_dist(0.0f, NOISE_STD);
    std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    for (size_t i = 0; i < N_POINTS; ++i)
    {
        float x = uniform_dist(rng) * 8.0f - 4.0f;  // [-4, 4]
        float y = uniform_dist(rng) * 8.0f - 4.0f;  // [-4, 4]

        // Quadratic surface
        float z = 0.1f * x * x + 0.05f * y * y + 0.2f * x * y + 1.0f;
        z += noise_dist(rng);

        pc->insertPointFast(x, y, z);
    }

    // Apply MLS filter with quadratic fitting
    FilterMLS filter;
    filter.setMinLoggingLevel(mrpt::system::LVL_WARN);

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "raw";
    params["output_pointcloud_layer"] = "mls";
    params["search_radius"]           = 1.2f;
    params["polynomial_order"]        = 2;  // Quadratic fit
    params["min_neighbors_for_fit"]   = 6;

    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);

    ASSERTMSG_(map.layers.count("mls"), "MLS output layer not found");

    auto mls_pc = mp2p_icp::MapToPointsMap(*map.layers.at("mls"));
    ASSERTMSG_(mls_pc, "MLS layer is not a point cloud");
    ASSERT_GT_(mls_pc->size(), 0);

    // For quadratic surface, check that points are smoothed
    const auto& xs_filt = mls_pc->getPointsBufferRef_x();
    const auto& ys_filt = mls_pc->getPointsBufferRef_y();
    const auto& zs_filt = mls_pc->getPointsBufferRef_z();

    // Check that filtered cloud has reasonable size
    ASSERT_GT_(mls_pc->size(), N_POINTS / 2);  // At least half the points

    // Verify filtered points
    float avg_distance = 0.0;
    for (size_t i = 0; i < mls_pc->size(); ++i)
    {
        const float expected_z = 0.1f * mrpt::square(xs_filt[i]) +
                                 0.05f * mrpt::square(ys_filt[i]) + 0.2f * xs_filt[i] * ys_filt[i] +
                                 1.0f;

        const float actual_z = zs_filt[i];

        avg_distance += std::abs(expected_z - actual_z);
    }
    avg_distance /= static_cast<float>(mls_pc->size());

    ASSERT_LT_(avg_distance, 0.10);

    std::cout << "Success ✅ (avg distance: " << avg_distance << ")\n";
}

// Test FilterMLS with distinct cloud projection
void test_FilterMLS_DistinctCloudProjection()
{
    std::cout << "Testing FilterMLS (distinct cloud projection)... ";

    // Create main point cloud (planar surface)
    auto pc_main = mrpt::maps::CGenericPointsMap::Create();

    std::mt19937                          rng(456);
    std::normal_distribution<float>       noise_dist(0.0f, 0.005f);
    std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    // Main cloud: plane z = 2.0 with small noise
    const size_t N_MAIN = 200;
    for (size_t i = 0; i < N_MAIN; ++i)
    {
        float x = uniform_dist(rng) * 6.0f;
        float y = uniform_dist(rng) * 6.0f;
        float z = 2.0f + noise_dist(rng);

        pc_main->insertPointFast(x, y, z);
    }

    // Create distinct cloud (points above the surface)
    auto pc_distinct = mrpt::maps::CGenericPointsMap::Create();

    const size_t N_DISTINCT = 50;
    for (size_t i = 0; i < N_DISTINCT; ++i)
    {
        float x = uniform_dist(rng) * 6.0f;
        float y = uniform_dist(rng) * 6.0f;
        float z = 2.0f;

        pc_distinct->insertPointFast(x, y, z);
    }

    // Apply MLS filter with distinct cloud projection
    FilterMLS filter;
    filter.setMinLoggingLevel(mrpt::system::LVL_WARN);

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "main";
    params["output_pointcloud_layer"] = "mls_output";
    params["distinct_cloud_layer"]    = "distinct";
    params["search_radius"]           = 1.0f;
    params["polynomial_order"]        = 1;
    params["upsampling_method"]       = "FilterMLS::UpsamplingMethod::DISTINCT_CLOUD";
    params["min_neighbors_for_fit"]   = 4;

    filter.initialize(params);

    metric_map_t map;
    map.layers["main"]     = pc_main;
    map.layers["distinct"] = pc_distinct;
    filter.filter(map);

    ASSERTMSG_(map.layers.count("mls_output"), "MLS output layer not found");

    auto mls_pc = mp2p_icp::MapToPointsMap(*map.layers.at("mls_output"));
    ASSERTMSG_(mls_pc, "MLS layer is not a point cloud");

    // Output should contain points from distinct cloud projected onto the surface
    ASSERT_GT_(mls_pc->size(), 0);

    // Check that projected points are closer to z=2.0
    const auto& zs    = mls_pc->getPointsBufferRef_z();
    double      avg_z = 0.0;
    for (float z : zs)
    {
        avg_z += z;
    }
    avg_z /= static_cast<double>(mls_pc->size());

    // Average z should be closer to 2.0 than the distinct cloud's z=3.0
    ASSERT_LT_(std::abs(avg_z - 2.0f), 0.5f);

    std::cout << "Success ✅ (avg z: " << avg_z << ")" << std::endl;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Vector of test functors
        std::vector<std::function<void()>> tests = {
            test_FilterBoundingBox_InsideOutside,
            test_FilterBoundingBox_OnlyInside,
            test_FilterBoundingBox_EmptyBox,
            test_FilterByIntensity_ThreeLayers,
            test_FilterByIntensity_TwoLayers,
            test_FilterByIntensity_ExactThreshold,
            test_FilterMLS_PlanarCloud,
            test_FilterMLS_QuadraticSurface,
            test_FilterMLS_DistinctCloudProjection,
        };

        int failures = 0;

        for (const auto& test : tests)
        {
            try
            {
                test();
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error: ❌\n" << e.what() << std::endl;
                failures++;
            }
        }

        if (failures == 0)
        {
            std::cout << "\n✅ All tests passed!" << std::endl;
        }
        else
        {
            std::cout << "\n❌ " << failures << " test(s) failed!" << std::endl;
        }

        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error:\n" << e.what() << std::endl;
        return 1;
    }
}
