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
 * @file   test-mp2p_FilterMerge.cpp
 * @brief  Tests for FilterMerge point insertion and frames
 * @author Jose Luis Blanco Claraco
 * @date   Oct 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterMerge.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>

using namespace mp2p_icp_filters;

namespace
{
constexpr size_t NUM_POINTS = 5;

mrpt::maps::CGenericPointsMap::Ptr makeCloud()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("intensity");
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        pc->insertPointFast(1.0f + static_cast<float>(i), 2.0f, 3.0f);
        pc->insertPointField_float("intensity", static_cast<float>(i));
    }
    pc->mark_as_modified();
    return pc;
}

FilterMerge makeFilter(bool localCoordinates, const mrpt::math::TPose3D& robotPose)
{
    FilterMerge f;
    f.params.input_pointcloud_layer           = "input";
    f.params.target_layer                     = "target";
    f.params.input_layer_in_local_coordinates = localCoordinates;
    f.params.robot_pose                       = robotPose;
    return f;
}

// Checks the target holds the input points transformed by `tf`, and the input is untouched:
void checkMerge(mp2p_icp::metric_map_t& map, const mrpt::poses::CPose3D& tf, size_t expectedSize)
{
    const auto ref = makeCloud();
    auto       in  = map.layer<mrpt::maps::CGenericPointsMap>("input");
    auto       out = map.layer<mrpt::maps::CGenericPointsMap>("target");
    ASSERT_(in);
    ASSERT_(out);
    ASSERT_EQUAL_(in->size(), NUM_POINTS);
    ASSERT_EQUAL_(out->size(), expectedSize);

    const size_t i0 = expectedSize - NUM_POINTS;
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        ASSERT_EQUAL_(in->getPointsBufferRef_x()[i], ref->getPointsBufferRef_x()[i]);

        const auto expected = tf.composePoint(mrpt::math::TPoint3D(
            ref->getPointsBufferRef_x()[i], ref->getPointsBufferRef_y()[i],
            ref->getPointsBufferRef_z()[i]));
        ASSERT_NEAR_(out->getPointsBufferRef_x()[i0 + i], expected.x, 1e-4);
        ASSERT_NEAR_(out->getPointsBufferRef_y()[i0 + i], expected.y, 1e-4);
        ASSERT_NEAR_(out->getPointsBufferRef_z()[i0 + i], expected.z, 1e-4);
        ASSERT_EQUAL_(out->getPointField_float(i0 + i, "intensity"), static_cast<float>(i));
    }
}

void test_local_coordinates()
{
    const auto robotPose = mrpt::math::TPose3D(1.0, 2.0, 0.5, 0.3, 0.1, -0.2);

    mp2p_icp::metric_map_t map;
    map.layers["input"]  = makeCloud();
    map.layers["target"] = mrpt::maps::CGenericPointsMap::Create();

    makeFilter(true, robotPose).filter(map);
    checkMerge(map, mrpt::poses::CPose3D(robotPose), NUM_POINTS);
    std::cout << "[Test Passed] FilterMerge: input in local coordinates\n";
}

void test_global_coordinates()
{
    const auto robotPose = mrpt::math::TPose3D(1.0, 2.0, 0.5, 0.3, 0.1, -0.2);

    for (const auto& pose : {mrpt::math::TPose3D::Identity(), robotPose})
    {
        mp2p_icp::metric_map_t map;
        map.layers["input"]  = makeCloud();
        map.layers["target"] = mrpt::maps::CGenericPointsMap::Create();

        makeFilter(false, pose).filter(map);
        checkMerge(map, mrpt::poses::CPose3D::Identity(), NUM_POINTS);
    }
    std::cout << "[Test Passed] FilterMerge: input in global coordinates\n";
}

void test_merge_into_itself()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"] = makeCloud();

    auto f                = makeFilter(true, mrpt::math::TPose3D::Identity());
    f.params.target_layer = "input";
    f.filter(map);

    auto out = map.layer<mrpt::maps::CGenericPointsMap>("input");
    ASSERT_EQUAL_(out->size(), 2 * NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        ASSERT_EQUAL_(out->getPointsBufferRef_x()[i], out->getPointsBufferRef_x()[NUM_POINTS + i]);
    }
    std::cout << "[Test Passed] FilterMerge: input merged into itself\n";
}

}  // namespace

int main()
{
    try
    {
        test_local_coordinates();
        test_global_coordinates();
        test_merge_into_itself();
        std::cout << "\nAll FilterMerge tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
    return 0;
}
