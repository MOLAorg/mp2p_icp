/* +
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
 * @file   test-mp2p_Filters.cpp
 * @brief  Unit tests for various filters
 * @author Jose Luis Blanco Claraco
 * @date   Nov 26, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mp2p_icp_filters/FilterByIntensity.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <iostream>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

// Helper to create a test point cloud with known positions and intensities
mrpt::maps::CPointsMap::Ptr createTestPointsWithIntensity(
    size_t n_x, size_t n_y, float spacing = 1.0f)
{
    auto pc = mrpt::maps::CPointsMapXYZI::Create();

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
            pc->insertPointField_float(
                mrpt::maps::CPointsMapXYZI::POINT_FIELD_INTENSITY, intensity);
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

    filter.initialize_filter(params);

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
    const size_t EXPECTED_INSIDE  = 6 * 6;
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

    filter.initialize_filter(params);

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

    filter.initialize_filter(params);

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
#if MRPT_VERSION >= 0x020f00
    const auto* ptrI_low  = low_pc->getPointsBufferRef_float_field("intensity");
    const auto* ptrI_mid  = mid_pc->getPointsBufferRef_float_field("intensity");
    const auto* ptrI_high = high_pc->getPointsBufferRef_float_field("intensity");

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
#endif

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

    filter.initialize_filter(params);

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

    filter.initialize_filter(params);

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

    auto pc = mrpt::maps::CPointsMapXYZI::Create();

    // Create points with exact threshold intensities
    std::vector<float> intensities = {0.0f, 0.3f, 0.5f, 0.7f, 1.0f};

    for (size_t i = 0; i < intensities.size(); ++i)
    {
        pc->insertPointFast(static_cast<float>(i), 0.0f, 0.0f);
#if MRPT_VERSION >= 0x020f00
        pc->insertPointField_float(
            mrpt::maps::CPointsMapXYZI::POINT_FIELD_INTENSITY, intensities[i]);
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

    filter.initialize_filter(params);

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

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        int failures = 0;

        // Test FilterBoundingBox
        try
        {
            test_FilterBoundingBox_InsideOutside();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
        }

        try
        {
            test_FilterBoundingBox_OnlyInside();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
        }

        try
        {
            test_FilterBoundingBox_EmptyBox();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
        }

        // Test FilterByIntensity
        try
        {
            test_FilterByIntensity_ThreeLayers();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
        }

        try
        {
            test_FilterByIntensity_TwoLayers();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
        }

        try
        {
            test_FilterByIntensity_ExactThreshold();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: ❌\n" << e.what() << std::endl;
            failures++;
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