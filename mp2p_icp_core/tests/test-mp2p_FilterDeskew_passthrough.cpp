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
 * @file   test-mp2p_FilterDeskew_passthrough.cpp
 * @brief  Tests for FilterDeskew when points are forwarded without correction
 * @author Jose Luis Blanco Claraco
 * @date   Oct 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDeskew.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

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

FilterDeskew makeFilter(MotionCompensationMethod method)
{
    FilterDeskew f;
    f.input_pointcloud_layer        = "input";
    f.output_pointcloud_layer       = "output";
    f.output_layer_class            = "mrpt::maps::CGenericPointsMap";
    f.silently_ignore_no_timestamps = true;
    f.method                        = method;
    f.twist.emplace(1.0, 0, 0, 0, 0, 0);
    return f;
}

void checkSamePoints(const mrpt::maps::CPointsMap& a, const mrpt::maps::CPointsMap& b)
{
    ASSERT_EQUAL_(a.size(), b.size());
    for (size_t i = 0; i < a.size(); i++)
    {
        ASSERT_EQUAL_(a.getPointsBufferRef_x()[i], b.getPointsBufferRef_x()[i]);
        ASSERT_EQUAL_(a.getPointsBufferRef_y()[i], b.getPointsBufferRef_y()[i]);
        ASSERT_EQUAL_(a.getPointsBufferRef_z()[i], b.getPointsBufferRef_z()[i]);
        ASSERT_EQUAL_(a.getPointField_float(i, "intensity"), b.getPointField_float(i, "intensity"));
    }
}

void test_no_timestamps_shares_input_layer()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(false);

    makeFilter(MotionCompensationMethod::Linear).filter(map);

    ASSERT_(map.layers.at("output") == map.layers.at("input"));
    std::cout << "[Test Passed] no timestamps: output layer shares the input object\n";
}

void test_method_none_shares_input_layer()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(true);

    makeFilter(MotionCompensationMethod::None).filter(map);

    ASSERT_(map.layers.at("output") == map.layers.at("input"));
    std::cout << "[Test Passed] method None: output layer shares the input object\n";
}

void test_empty_input_shares_input_layer()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = mrpt::maps::CGenericPointsMap::Create();

    makeFilter(MotionCompensationMethod::Linear).filter(map);

    ASSERT_(map.layers.at("output") == map.layers.at("input"));
    std::cout << "[Test Passed] empty input: output layer shares the input object\n";
}

void test_different_class_is_copied()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(false);

    auto f               = makeFilter(MotionCompensationMethod::Linear);
    f.output_layer_class = "mrpt::maps::CSimplePointsMap";
    f.filter(map);

    ASSERT_(map.layers.at("output") != map.layers.at("input"));
    auto out = map.layer<mrpt::maps::CSimplePointsMap>("output");
    ASSERT_(out);
    ASSERT_EQUAL_(out->size(), NUM_POINTS);
    std::cout << "[Test Passed] different output class: input is copied\n";
}

void test_existing_output_is_appended()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"]  = makeCloud(false);
    map.layers["output"] = makeCloud(false);

    makeFilter(MotionCompensationMethod::Linear).filter(map);

    ASSERT_(map.layers.at("output") != map.layers.at("input"));
    auto in  = map.layer<mrpt::maps::CGenericPointsMap>("input");
    auto out = map.layer<mrpt::maps::CGenericPointsMap>("output");
    ASSERT_EQUAL_(in->size(), NUM_POINTS);
    ASSERT_EQUAL_(out->size(), 2 * NUM_POINTS);
    std::cout << "[Test Passed] existing output layer: input is appended\n";
}

void test_deskew_is_not_shared()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud(true);
    const auto inCopy   = makeCloud(true);

    makeFilter(MotionCompensationMethod::Linear).filter(map);

    ASSERT_(map.layers.at("output") != map.layers.at("input"));
    auto in  = map.layer<mrpt::maps::CGenericPointsMap>("input");
    auto out = map.layer<mrpt::maps::CGenericPointsMap>("output");
    ASSERT_EQUAL_(out->size(), NUM_POINTS);

    // The input must be untouched, and the output corrected (vx=1 m/s):
    checkSamePoints(*in, *inCopy);
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        const float t = in->getPointField_float(i, "t");
        ASSERT_NEAR_(out->getPointsBufferRef_x()[i], in->getPointsBufferRef_x()[i] + t, 1e-5f);
        ASSERT_EQUAL_(out->getPointField_float(i, "intensity"), static_cast<float>(i));
    }
    std::cout << "[Test Passed] actual de-skew: output is a new, corrected cloud\n";
}

}  // namespace

int main()
{
    try
    {
        test_no_timestamps_shares_input_layer();
        test_method_none_shares_input_layer();
        test_empty_input_shares_input_layer();
        test_different_class_is_copied();
        test_existing_output_is_appended();
        test_deskew_is_not_shared();
        std::cout << "\nAll FilterDeskew pass-through tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
    return 0;
}
