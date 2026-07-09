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
 * @file   test-mp2p_FilterByRing.cpp
 * @brief  Unit test for FilterByRing
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterByRing.h>
#include <mrpt/maps/CGenericPointsMap.h>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{
// Helper to create point cloud with ring information
mrpt::maps::CGenericPointsMap::Ptr createTestPointCloudWithRings()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();

    // Register ring field
    pc->registerField_uint16(POINT_FIELD_RING_ID);

    // Add points with various ring indices
    // Ring 0
    pc->insertPoint(1.0f, 0.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 0);

    pc->insertPoint(2.0f, 0.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 0);

    // Ring 1
    pc->insertPoint(1.0f, 1.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 1);

    pc->insertPoint(2.0f, 1.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 1);

    // Ring 2
    pc->insertPoint(1.0f, 2.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 2);

    pc->insertPoint(2.0f, 2.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 2);

    // Ring 5
    pc->insertPoint(1.0f, 5.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 5);

    pc->insertPoint(2.0f, 5.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 5);

    // Ring 10
    pc->insertPoint(1.0f, 10.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 10);

    pc->insertPoint(2.0f, 10.0f, 0.0f);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 10);

    return pc;
}

const auto* get_ring_channel(const mrpt::maps::CGenericPointsMap::Ptr& pts)
{
#if MRPT_VERSION >= 0x020f04  // 2.15.4
    return pts->getPointsBufferRef_uint16_field(mp2p_icp_filters::POINT_FIELD_RING_ID);
#else
    return pts->getPointsBufferRef_uint_field(mp2p_icp_filters::POINT_FIELD_RING_ID);
#endif
}

void test_keep_rings_list()
{
    FilterByRing filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_selected"]  = "filtered";
    params["selected_ring_ids"]      = mrpt::containers::yaml::Sequence({0, 2, 5});

    filter.initialize(params);

    auto input = createTestPointCloudWithRings();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should keep only rings 0, 2, 5 (6 points total)
    ASSERT_EQUAL_(output->size(), 6);

    // Verify rings
    ASSERT_(output->hasPointField(mp2p_icp_filters::POINT_FIELD_RING_ID));
    for (size_t i = 0; i < output->size(); i++)
    {
        const auto* ring = get_ring_channel(output);

        uint16_t ringValue = ring->operator[](i);
        ASSERT_(ringValue == 0 || ringValue == 2 || ringValue == 5);
    }
}

void test_keep_rings_range()
{
    FilterByRing filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_selected"]  = "filtered";
    params["selected_ring_ids"]      = mrpt::containers::yaml::Sequence({1, 2, 3, 5});

    filter.initialize(params);

    auto input = createTestPointCloudWithRings();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should keep rings 1, 2, 5 (6 points total)
    ASSERT_EQUAL_(output->size(), 6);

    // Verify rings are in range [1, 5]
    ASSERT_(output->hasPointField(POINT_FIELD_RING_ID));
    for (size_t i = 0; i < output->size(); i++)
    {
        const auto* ring   = get_ring_channel(output);
        uint16_t ringValue = ring->operator[](i);
        ASSERT_GE_(ringValue, 1);
        ASSERT_LE_(ringValue, 5);
    }
}

void test_remove_rings_list()
{
    FilterByRing filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]    = "raw";
    params["output_layer_non_selected"] = "filtered";
    params["selected_ring_ids"]         = mrpt::containers::yaml::Sequence({0, 10});

    filter.initialize(params);

    auto input = createTestPointCloudWithRings();

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    ASSERT_(map.layers.count("filtered") > 0);
    auto output =
        std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(map.layers.at("filtered"));
    ASSERT_(output);

    // Should remove rings 0 and 10 (4 points removed, 6 remaining)
    ASSERT_EQUAL_(output->size(), 6);

    // Verify no points have ring 0 or 10
    ASSERT_(output->hasPointField(POINT_FIELD_RING_ID));
    for (size_t i = 0; i < output->size(); i++)
    {
        const auto* ring   = get_ring_channel(output);
        uint16_t ringValue = ring->operator[](i);
        ASSERT_NOT_EQUAL_(ringValue, 0);
        ASSERT_NOT_EQUAL_(ringValue, 10);
    }
}

void test_no_ring_field()
{
    FilterByRing filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"] = "raw";
    params["output_layer_selected"]  = "filtered";
    params["selected_ring_ids"]      = mrpt::containers::yaml::Sequence({0, 1});

    filter.initialize(params);

    // Create point cloud without ring field
    auto input = mrpt::maps::CGenericPointsMap::Create();
    input->insertPoint(1.0f, 0.0f, 0.0f);
    input->insertPoint(2.0f, 0.0f, 0.0f);

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
    FilterByRing filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "raw";
    params["output_pointcloud_layer"] = "raw";  // Same layer
    params["selected_ring_ids"]       = mrpt::containers::yaml::Sequence({1, 2});

    filter.initialize(params);

    auto input = createTestPointCloudWithRings();

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
        test_keep_rings_list();
        std::cout << "test_keep_rings_list: Success ✅" << std::endl;

        test_keep_rings_range();
        std::cout << "test_keep_rings_range: Success ✅" << std::endl;

        test_remove_rings_list();
        std::cout << "test_remove_rings_list: Success ✅" << std::endl;

        test_no_ring_field();
        std::cout << "test_no_ring_field: Success ✅" << std::endl;

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
