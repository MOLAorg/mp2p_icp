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
 * @file   test-mp2p_FilterMerge_view.cpp
 * @brief  Tests for mp2p_icp::rotateViewDirectionFields(), the helper used by
 *         FilterMerge and mola::KeyframePointCloudMap to keep per-point
 *         view-direction unit vectors (view_x/y/z) consistent with the frame
 *         the point coordinates are expressed in, plus a FilterMerge
 *         integration check for its only well-defined view-vector contract
 *         (input_layer_in_local_coordinates=true is a no-op).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/pointcloud_field_utils.h>
#include <mp2p_icp_filters/FilterMerge.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;

namespace
{

mrpt::maps::CGenericPointsMap::Ptr makeEmptyViewLayer()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("view_x");
    pc->registerField_float("view_y");
    pc->registerField_float("view_z");
    return pc;
}

// Single point at (10,0,0) with view vector (-1,0,0).
mrpt::maps::CGenericPointsMap::Ptr makeOnePointWithView()
{
    auto pc = makeEmptyViewLayer();
    pc->insertPointFast(10.f, 0.f, 0.f);
    pc->insertPointField_float("view_x", -1.f);
    pc->insertPointField_float("view_y", 0.f);
    pc->insertPointField_float("view_z", 0.f);
    pc->mark_as_modified();
    return pc;
}

void test_rotateViewDirectionFields_rotates_correctly()
{
    for (const double yaw_deg : {0.0, 90.0, 180.0, 270.0})
    {
        auto pc = makeOnePointWithView();

        const auto tf = mrpt::poses::CPose3D(0, 0, 0, mrpt::DEG2RAD(yaw_deg), 0, 0);
        mp2p_icp::rotateViewDirectionFields(*pc, tf);

        const auto expected = tf.rotateVector(mrpt::math::TVector3D(-1, 0, 0));

        const float vx = pc->getPointField_float(0, "view_x");
        const float vy = pc->getPointField_float(0, "view_y");
        const float vz = pc->getPointField_float(0, "view_z");

        ASSERT_NEAR_(vx, static_cast<float>(expected.x), 1e-5f);
        ASSERT_NEAR_(vy, static_cast<float>(expected.y), 1e-5f);
        ASSERT_NEAR_(vz, static_cast<float>(expected.z), 1e-5f);

        // The point coordinates must be untouched.
        float px, py, pz;
        pc->getPointFast(0, px, py, pz);
        ASSERT_NEAR_(px, 10.0f, 1e-5f);
        ASSERT_NEAR_(py, 0.0f, 1e-5f);
        ASSERT_NEAR_(pz, 0.0f, 1e-5f);

        // Rotation must preserve the unit norm.
        const float norm2 = vx * vx + vy * vy + vz * vz;
        ASSERT_NEAR_(norm2, 1.0f, 1e-5f);

        std::cout << "[Test Passed] rotateViewDirectionFields yaw=" << yaw_deg << " deg\n";
    }
}

// At yaw=180deg, leaving the vector unrotated (the pre-fix bug symptom)
// would keep view=(-1,0,0); the correct rotated result is (1,0,0).
void test_rotateViewDirectionFields_not_left_unrotated_at_180deg()
{
    auto pc = makeOnePointWithView();

    const auto tf = mrpt::poses::CPose3D(0, 0, 0, mrpt::DEG2RAD(180.0), 0, 0);
    mp2p_icp::rotateViewDirectionFields(*pc, tf);

    ASSERT_NEAR_(pc->getPointField_float(0, "view_x"), 1.0f, 1e-5f);
    ASSERT_NEAR_(pc->getPointField_float(0, "view_y"), 0.0f, 1e-5f);
    ASSERT_NEAR_(pc->getPointField_float(0, "view_z"), 0.0f, 1e-5f);

    std::cout << "[Test Passed] view vector correctly rotated at yaw=180deg\n";
}

void test_rotateViewDirectionFields_noop_on_identity()
{
    auto pc = makeOnePointWithView();
    mp2p_icp::rotateViewDirectionFields(*pc, mrpt::poses::CPose3D::Identity());

    ASSERT_NEAR_(pc->getPointField_float(0, "view_x"), -1.0f, 1e-6f);
    ASSERT_NEAR_(pc->getPointField_float(0, "view_y"), 0.0f, 1e-6f);
    ASSERT_NEAR_(pc->getPointField_float(0, "view_z"), 0.0f, 1e-6f);

    std::cout << "[Test Passed] identity transform is a no-op\n";
}

void test_rotateViewDirectionFields_noop_without_view_fields()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->insertPointFast(10.f, 0.f, 0.f);
    pc->mark_as_modified();

    // Must not throw nor crash when the view fields are absent.
    mp2p_icp::rotateViewDirectionFields(*pc, mrpt::poses::CPose3D(0, 0, 0, M_PI, 0, 0));

    float px, py, pz;
    pc->getPointFast(0, px, py, pz);
    ASSERT_NEAR_(px, 10.0f, 1e-5f);

    std::cout << "[Test Passed] no-op when view fields are not registered\n";
}

mp2p_icp::metric_map_t makeFilterMergeInputMap()
{
    mp2p_icp::metric_map_t map;
    map.layers["input"]  = makeOnePointWithView();
    map.layers["target"] = makeEmptyViewLayer();
    return map;
}

// When input_layer_in_local_coordinates=true, FilterMerge applies an
// identity transform internally, so the view vector must come through
// unchanged. This is the only FilterMerge-level scenario with a frame
// contract that is well-defined independently of the target map's
// insertObservation() semantics (see FilterMerge.cpp).
void test_FilterMerge_local_coordinates_is_noop_for_view_vector()
{
    auto map = makeFilterMergeInputMap();

    FilterMerge filter;
    filter.params.input_pointcloud_layer           = "input";
    filter.params.target_layer                     = "target";
    filter.params.input_layer_in_local_coordinates = true;

    filter.filter(map);

    auto out = map.layer<mrpt::maps::CGenericPointsMap>("target");
    ASSERT_(out);
    ASSERT_EQUAL_(out->size(), 1UL);

    ASSERT_NEAR_(out->getPointField_float(0, "view_x"), -1.0f, 1e-6f);
    ASSERT_NEAR_(out->getPointField_float(0, "view_y"), 0.0f, 1e-6f);
    ASSERT_NEAR_(out->getPointField_float(0, "view_z"), 0.0f, 1e-6f);

    std::cout << "[Test Passed] FilterMerge: local-coordinates input is a no-op for view vectors\n";
}

}  // namespace

int main()
{
    try
    {
        test_rotateViewDirectionFields_rotates_correctly();
        test_rotateViewDirectionFields_not_left_unrotated_at_180deg();
        test_rotateViewDirectionFields_noop_on_identity();
        test_rotateViewDirectionFields_noop_without_view_fields();
        test_FilterMerge_local_coordinates_is_noop_for_view_vector();
        std::cout << "\nAll FilterMerge view-vector tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
    return 0;
}
