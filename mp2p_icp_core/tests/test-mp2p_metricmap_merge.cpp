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
 * @file   test-mp2p_metricmap_merge.cpp
 * @brief  Unit tests for metric_map_t::merge_with()
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>

namespace
{
void test_merge_planes_with_pose()
{
    // Plane z=1, with its centroid on it:
    mp2p_icp::metric_map_t other;
    other.planes.emplace_back(
        mrpt::math::TPlane(0, 0, 2, -2), mrpt::math::TPoint3D(3, 0, 1), 5.0, 100U);

    const auto pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.0, 2.0, 3.0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(-20.0), mrpt::DEG2RAD(10.0));

    mp2p_icp::metric_map_t m;
    m.merge_with(other, pose.asTPose());

    ASSERT_EQUAL_(m.planes.size(), 1U);
    const auto& g = m.planes.at(0);

    // Points of the original plane must lie on the transformed one:
    for (const auto& p :
         {mrpt::math::TPoint3D(0, 0, 1), mrpt::math::TPoint3D(-4, 7, 1),
          mrpt::math::TPoint3D(3, 0, 1)})
    {
        ASSERT_LT_(std::abs(g.plane.evaluatePoint(pose.composePoint(p))), 1e-9);
    }
    ASSERT_LT_((g.centroid - pose.composePoint(mrpt::math::TPoint3D(3, 0, 1))).norm(), 1e-9);
    ASSERT_EQUAL_(g.area, 5.0);
    ASSERT_EQUAL_(g.num_points, 100U);

    std::cout << "[test_merge_planes_with_pose] PASSED\n";
}

void test_self_merge_with_pose()
{
    mp2p_icp::metric_map_t m;
    for (int i = 0; i < 50; i++)  // enough to force a reallocation when appending
    {
        m.planes.emplace_back(
            mrpt::math::TPlane(0, 0, 1, -i), mrpt::math::TPoint3D(0, 0, i), 1.0, 10U);
        m.lines.emplace_back(mrpt::math::TLine3D::FromPointAndDirector({0, 0, 1.0 * i}, {1, 0, 0}));
    }
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(1, 2, 3);
    m.layers["p"] = pts;

    const auto pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 10.0, 0, 0, 0);
    m.merge_with(m, pose.asTPose());

    ASSERT_EQUAL_(m.planes.size(), 100U);
    ASSERT_EQUAL_(m.lines.size(), 100U);
    ASSERT_EQUAL_(m.layer<mrpt::maps::CPointsMap>("p")->size(), 2U);
    for (int i = 0; i < 50; i++)
    {
        ASSERT_LT_(std::abs(m.planes.at(50 + i).centroid.z - (i + 10.0)), 1e-9);
    }

    std::cout << "[test_self_merge_with_pose] PASSED\n";
}

void test_merge_mismatched_layer_types()
{
    mp2p_icp::metric_map_t m;
    m.layers["a"] = mrpt::maps::CSimplePointsMap::Create();

    mp2p_icp::metric_map_t other;
    other.layers["a"] = mrpt::maps::COccupancyGridMap2D::Create();

    bool thrown = false;
    try
    {
        m.merge_with(other);
    }
    catch (const std::exception&)
    {
        thrown = true;
    }
    ASSERT_(thrown);

    std::cout << "[test_merge_mismatched_layer_types] PASSED\n";
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_merge_planes_with_pose();
        test_self_merge_with_pose();
        test_merge_mismatched_layer_types();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
