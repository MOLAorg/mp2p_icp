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
 * @file   test-mp2p_FilterDeskew_in_place.cpp
 * @brief  Tests for FilterDeskew in-place vs. new-layer outputs
 * @author Jose Luis Blanco Claraco
 * @date   Oct 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDeskew.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

namespace
{
constexpr size_t NUM_POINTS = 10;

mrpt::maps::CGenericPointsMap::Ptr makeCloud(bool withTimestamps)
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("intensity");
    if (withTimestamps)
    {
        pc->registerField_float("t");
    }
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        pc->insertPointFast(1.0f + static_cast<float>(i), 2.0f, 3.0f);
        pc->insertPointField_float("intensity", static_cast<float>(i));
        if (withTimestamps)
        {
            pc->insertPointField_float("t", 0.01f * static_cast<float>(i));
        }
    }
    pc->mark_as_modified();
    return pc;
}

// Linear deskew with a constant forward velocity of 1 m/s:
FilterDeskew makeFilter(bool inPlace)
{
    FilterDeskew f;
    f.input_pointcloud_layer = "input";
    if (inPlace)
    {
        f.in_place = true;
    }
    else
    {
        f.output_pointcloud_layer = "output";
    }
    f.output_layer_class            = "mrpt::maps::CGenericPointsMap";
    f.silently_ignore_no_timestamps = true;
    f.method                        = MotionCompensationMethod::Linear;
    f.twist.emplace(1.0, 0, 0, 0, 0, 0);
    return f;
}

// Checks `pc` holds the reference cloud, with x shifted by `t` if `deskewed`:
void checkCloud(const mrpt::maps::CPointsMap& pc, bool deskewed)
{
    const auto ref = makeCloud(true);
    ASSERT_EQUAL_(pc.size(), NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        const float dx = deskewed ? ref->getPointField_float(i, "t") : 0.0f;
        ASSERT_NEAR_(pc.getPointsBufferRef_x()[i], ref->getPointsBufferRef_x()[i] + dx, 1e-5f);
        ASSERT_EQUAL_(pc.getPointsBufferRef_y()[i], ref->getPointsBufferRef_y()[i]);
        ASSERT_EQUAL_(pc.getPointsBufferRef_z()[i], ref->getPointsBufferRef_z()[i]);
        ASSERT_EQUAL_(pc.getPointField_float(i, "intensity"), static_cast<float>(i));
    }
}

void test_new_layer_without_timestamps_is_a_copy()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(false);

    makeFilter(false).filter(map);

    ASSERT_(map.layers.at("output") != map.layers.at("input"));
    checkCloud(*map.point_layer("output"), false);
    std::cout << "[Test Passed] no timestamps: output layer is a copy of the input\n";
}

void test_new_layer_with_timestamps()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(true);

    makeFilter(false).filter(map);

    checkCloud(*map.point_layer("input"), false);
    checkCloud(*map.point_layer("output"), true);
    std::cout << "[Test Passed] timestamps: new output layer is deskewed, input untouched\n";
}

void test_in_place_without_timestamps()
{
    mp2p_icp::metric_map_t map;
    const auto             input = makeCloud(false);
    map.layers["input"]          = input;

    makeFilter(true).filter(map);

    ASSERT_EQUAL_(map.layers.size(), 1UL);
    ASSERT_(map.layers.at("input") == input);
    checkCloud(*input, false);
    std::cout << "[Test Passed] in place, no timestamps: layer left untouched\n";
}

void test_in_place_with_timestamps()
{
    mp2p_icp::metric_map_t map;
    const auto             input = makeCloud(true);
    map.layers["input"]          = input;

    makeFilter(true).filter(map);

    ASSERT_EQUAL_(map.layers.size(), 1UL);
    ASSERT_(map.layers.at("input") == input);
    checkCloud(*input, true);
    std::cout << "[Test Passed] in place, timestamps: layer deskewed, fields kept\n";
}

}  // namespace

int main()
{
    try
    {
        test_new_layer_without_timestamps_is_a_copy();
        test_new_layer_with_timestamps();
        test_in_place_without_timestamps();
        test_in_place_with_timestamps();
        std::cout << "\nAll FilterDeskew in-place tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
    return 0;
}
