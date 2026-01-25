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
 * @file   test-mp2p_FilterByIntensity.cpp
 * @brief  Unit test for FilterByIntensity
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterByIntensity.h>
#include <mrpt/maps/CGenericPointsMap.h>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

// Helper to create point cloud with intensity values
mrpt::maps::CGenericPointsMap::Ptr createTestPointCloudWithIntensity()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();

    // Register intensity field
    pc->registerField_float(POINT_FIELD_INTENSITY);

    // Add points with various intensities
    for (int i = 0; i < 20; i++)
    {
        float x         = static_cast<float>(i);
        float y         = 0.0f;
        float z         = 0.0f;
        float intensity = static_cast<float>(i * 5);  // 0, 5, 10, ..., 95

        pc->insertPoint(x, y, z);
        pc->insertPointField_float(POINT_FIELD_INTENSITY, intensity);
    }

    return pc;
}

void test_high_threshold_filter()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]     = "raw";
    params["output_layer_mid_intensity"] = "filtered";
    params["low_threshold"]              = 0.0f;
    params["high_threshold"]             = 50.0f;

    filter.initialize(params);

    auto input = createTestPointCloudWithIntensity();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should keep only points with intensity <= 50
    // Keep: 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50 (11 points)
    ASSERT_EQUAL_(output->size(), 11);

    // Verify all remaining points have intensity <= 50
    ASSERT_(output->hasPointField(POINT_FIELD_INTENSITY));
    auto intensityField = output->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    for (size_t i = 0; i < output->size(); i++)
    {
        ASSERT_LE_(intensityField->operator[](i), 50.1f);
    }
}

void test_min_high_threshold_filter()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]     = "raw";
    params["output_layer_mid_intensity"] = "filtered";
    params["low_threshold"]              = 30.0f;
    params["high_threshold"]             = 60.0f;

    filter.initialize(params);

    auto input = createTestPointCloudWithIntensity();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should keep only points with intensity in [30, 60]
    // Keep: 30, 35, 40, 45, 50, 55, 60 (7 points)
    ASSERT_EQUAL_(output->size(), 7);

    // Verify all remaining points have intensity in [30, 60]
    ASSERT_(output->hasPointField(POINT_FIELD_INTENSITY));
    auto intensityField = output->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    for (size_t i = 0; i < output->size(); i++)
    {
        ASSERT_GE_(intensityField->operator[](i), 29.9f);
        ASSERT_LE_(intensityField->operator[](i), 60.1f);
    }
}

void test_no_intensity_field()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]     = "raw";
    params["output_layer_mid_intensity"] = "filtered";
    params["low_threshold"]              = 50.0f;
    params["high_threshold"]             = 500.0f;

    filter.initialize(params);

    // Create point cloud without intensity field
    auto input = mrpt::maps::CGenericPointsMap::Create();
    input->insertPoint(1.0f, 0.0f, 0.0f);
    input->insertPoint(2.0f, 0.0f, 0.0f);
    input->insertPoint(3.0f, 0.0f, 0.0f);

    metric_map_t map;
    map.layers["raw"] = input;

    // Should throw:
    bool have_thrown = false;
    try
    {
        filter.filter(map);
    }
    catch (const std::exception& e)
    {
        (void)e;
        have_thrown = true;
    }
    ASSERT_(have_thrown);
}

void test_inplace_filtering()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]     = "raw";
    params["output_layer_mid_intensity"] = "raw";  // Same layer
    params["low_threshold"]              = 50.0f;
    params["high_threshold"]             = 500.0f;

    filter.initialize(params);

    auto input = createTestPointCloudWithIntensity();

    metric_map_t map;
    map.layers["raw"] = input;

    // Should throw:
    bool have_thrown = false;
    try
    {
        filter.filter(map);
    }
    catch (const std::exception& e)
    {
        (void)e;
        have_thrown = true;
    }
    ASSERT_(have_thrown);
}

void test_extreme_values()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]      = "raw";
    params["output_layer_high_intensity"] = "filtered";
    params["low_threshold"]               = 100.0f;  // Higher than any point
    params["high_threshold"]              = 1000.0f;  // Higher than any point

    filter.initialize(params);

    auto input = createTestPointCloudWithIntensity();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should filter out all points
    ASSERT_EQUAL_(output->size(), 0);
}

void test_intensity_preservation()
{
    FilterByIntensity filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]     = "raw";
    params["output_layer_mid_intensity"] = "filtered";
    params["low_threshold"]              = 40.0f;
    params["high_threshold"]             = 50.0f;

    filter.initialize(params);

    auto input = createTestPointCloudWithIntensity();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Verify intensity values are preserved correctly
    ASSERT_(output->hasPointField(POINT_FIELD_INTENSITY));
    auto intensityField = output->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);

    // Should have points with intensities: 40, 45, 50
    ASSERT_EQUAL_(output->size(), 3);

    std::vector<float> expectedIntensities = {40.0f, 45.0f, 50.0f};
    for (size_t i = 0; i < output->size() && i < expectedIntensities.size(); i++)
    {
        ASSERT_NEAR_(intensityField->operator[](i), expectedIntensities[i], 0.1f);
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_high_threshold_filter();
        std::cout << "test_high_threshold_filter: Success ✅" << std::endl;

        test_min_high_threshold_filter();
        std::cout << "test_min_high_threshold_filter: Success ✅" << std::endl;

        test_no_intensity_field();
        std::cout << "test_no_intensity_field: Success ✅" << std::endl;

        test_inplace_filtering();
        std::cout << "test_inplace_filtering: Success ✅" << std::endl;

        test_extreme_values();
        std::cout << "test_extreme_values: Success ✅" << std::endl;

        test_intensity_preservation();
        std::cout << "test_intensity_preservation: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
