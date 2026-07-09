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
 * @file   test-mp2p_FilterByRange.cpp
 * @brief  Unit test for FilterByRange
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterByRange.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/geometry.h>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{

mrpt::maps::CSimplePointsMap::Ptr createTestPointCloud()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();

    // Add points at various distances from origin
    // Near points (0-5m)
    pc->insertPoint(1.0f, 0.0f, 0.0f);
    pc->insertPoint(2.0f, 0.0f, 0.0f);
    pc->insertPoint(0.0f, 3.0f, 0.0f);
    pc->insertPoint(0.0f, 4.0f, 0.0f);

    // Medium distance points (5-10m)
    pc->insertPoint(5.0f, 0.0f, 0.0f);
    pc->insertPoint(6.0f, 0.0f, 0.0f);
    pc->insertPoint(0.0f, 7.0f, 0.0f);
    pc->insertPoint(0.0f, 8.0f, 0.0f);

    // Far points (>10m)
    pc->insertPoint(12.0f, 0.0f, 0.0f);
    pc->insertPoint(15.0f, 0.0f, 0.0f);
    pc->insertPoint(0.0f, 20.0f, 0.0f);

    return pc;
}

void test_range_min_filter()
{
    FilterByRange filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_between"]   = "filtered";
    params["range_min"]              = 5.0;
    params["range_max"]              = 100.0;

    filter.initialize(params);

    auto input = createTestPointCloud();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should filter out points closer than 5m
    // We have 4 points < 5m, so output should have 11 - 4 = 7 points
    ASSERT_EQUAL_(output->size(), 7);

    // Verify all points are >= 5m from origin
    for (size_t i = 0; i < output->size(); i++)
    {
        float x, y, z;
        output->getPoint(i, x, y, z);
        float range = std::sqrt(x * x + y * y + z * z);
        ASSERT_GE_(range, 4.99f);  // Small tolerance for floating point
    }
}

void test_range_max_filter()
{
    FilterByRange filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_between"]   = "filtered";
    params["range_max"]              = 10.0;
    params["range_min"]              = 0;

    filter.initialize(params);

    auto input = createTestPointCloud();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should filter out points farther than 10m
    // We have 3 points > 10m, so output should have 11 - 3 = 8 points
    ASSERT_EQUAL_(output->size(), 8);

    // Verify all points are <= 10m from origin
    for (size_t i = 0; i < output->size(); i++)
    {
        float x, y, z;
        output->getPoint(i, x, y, z);
        float range = std::sqrt(x * x + y * y + z * z);
        ASSERT_LE_(range, 10.01f);  // Small tolerance for floating point
    }
}

void test_min_range_max_filter()
{
    FilterByRange filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_between"]   = "filtered";
    params["output_layer_outside"]   = "outliers";
    params["range_min"]              = 5.0;
    params["range_max"]              = 10.0;

    filter.initialize(params);

    auto input = createTestPointCloud();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    ASSERT_(map.layers.count("outliers") > 0);
    auto outliers =
        std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(map.layers.at("outliers"));
    ASSERT_(outliers);

    // Should only keep points in range [5, 10]m
    // We have 4 points in this range
    ASSERT_EQUAL_(output->size(), 4);

    ASSERT_EQUAL_(outliers->size(), 7);

    // Verify all points are in [5, 10]m range
    for (size_t i = 0; i < output->size(); i++)
    {
        float x, y, z;
        output->getPoint(i, x, y, z);
        float range = std::sqrt(x * x + y * y + z * z);
        ASSERT_GE_(range, 4.99f);
        ASSERT_LE_(range, 10.01f);
    }
}

void test_inplace_filtering()
{
    FilterByRange filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_between"]   = "raw";  // Same layer
    params["range_min"]              = 5.0;
    params["range_max"]              = 10.0;

    filter.initialize(params);

    auto input = createTestPointCloud();

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

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_range_min_filter();
        std::cout << "test_range_min_filter: Success ✅" << std::endl;

        test_range_max_filter();
        std::cout << "test_range_max_filter: Success ✅" << std::endl;

        test_min_range_max_filter();
        std::cout << "test_min_range_max_filter: Success ✅" << std::endl;

        test_inplace_filtering();
        std::cout << "test_inplace_filtering: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
