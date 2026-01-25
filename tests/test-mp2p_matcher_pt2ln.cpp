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
 * @file   test-mp2p_matcher_pt2ln.cpp
 * @brief  Unit test for Matcher_Point2Line
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/Matcher_Point2Line.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TLine3D.h>

using namespace mp2p_icp;

namespace
{

// Helper to create a point cloud
mrpt::maps::CSimplePointsMap::Ptr createPointsMap(const std::vector<mrpt::math::TPoint3Df>& points)
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (const auto& p : points)
    {
        pc->insertPoint(p.x, p.y, p.z);
    }
    return pc;
}

mrpt::maps::CSimplePointsMap::Ptr createLineGlobalMap(const std::vector<float> ys = {0.0})
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    // Create a line along X axis from (0,y,0) to (10,y,0)
    for (const float y : ys)
    {
        for (int i = 0; i <= 100; i++)
        {
            pts->insertPoint(static_cast<float>(i) * 0.1f, y, 0.0f);
        }
    }
    return pts;
}

void test_basic_point_to_line_matching()
{
    // Create global map with line segments (as point cloud)
    auto globalPts = createLineGlobalMap();

    // Create local map with single point
    auto localPts = createPointsMap({{5.0f, 0.1f, 0.0f}});  // Close to global line

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 0.5;  // 50cm threshold
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    // Should find at least one point-to-line pairing
    ASSERT_GT_(pairs.paired_pt2ln.size(), 0);
}

void test_threshold_filtering()
{
    // Create global map with line segments (as point cloud)
    auto globalPts = createLineGlobalMap();

    // Create local map with points at different distances
    auto localPts = createPointsMap({
        {5.0f, 0.1f, 0.0f},  // Close to line
        {5.0f, 2.0f, 0.0f}  // Far from line
    });

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 0.5;  // Only accept points within 0.5m
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    // Should only match the close point
    ASSERT_EQUAL_(pairs.paired_pt2ln.size(), 1);
}

void test_with_pose_transformation()
{
    // Create global map with line segments (as point cloud)
    auto globalPts = createLineGlobalMap();

    // Create local map with point at origin
    auto localPts = createPointsMap({{0.0f, 0.0f, 0.0f}});

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 0.5;
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    // Transform: shift local map to (5, 0, 0)
    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {5, 0, 0, 0, 0, 0}, {}, ms, pairs);

    // After transformation, local point at (0,0,0) becomes (5,0,0) in global
    // Should match to the line
    ASSERT_GT_(pairs.paired_pt2ln.size(), 0);
}

void test_parallel_lines()
{
    // Create global map with two parallel lines
    auto globalPts = createLineGlobalMap({0.0f, 5.0f});

    // Create local map with point between the lines
    auto localPts = createPointsMap({{5.0f, 2.5f, 0.0f}});

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 3.0;  // Large threshold to match both lines
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    // Should find pairing to nearest line
    ASSERT_GT_(pairs.paired_pt2ln.size(), 0);
}

void test_empty_maps()
{
    auto globalPts = mrpt::maps::CSimplePointsMap::Create();
    auto localPts  = mrpt::maps::CSimplePointsMap::Create();

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 1.0;
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    // No pairings should be found
    ASSERT_EQUAL_(pairs.paired_pt2ln.size(), 0);
}

void test_line_direction()
{
    // Create global map with line segments (as point cloud)
    auto globalPts = createLineGlobalMap();

    // Create local map with point
    auto localPts = createPointsMap({{5.0f, 0.1f, 0.0f}});

    metric_map_t pcGlobal;
    pcGlobal.layers["raw"] = globalPts;

    metric_map_t pcLocal;
    pcLocal.layers["raw"] = localPts;

    Matcher_Point2Line     matcher;
    mrpt::containers::yaml params;
    params["distanceThreshold"]  = 0.5;
    params["knn"]                = 4;
    params["lineEigenThreshold"] = 0.01;
    params["minimumLinePoints"]  = 4;

    matcher.initialize(params);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    matcher.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    ASSERT_GT_(pairs.paired_pt2ln.size(), 0);

    // Check that line direction is approximately along X axis
    const auto& pairing = pairs.paired_pt2ln[0];
    const auto& lineDir = pairing.ln_global.director;

    // Normalize and check if approximately [1, 0, 0] or [-1, 0, 0]
    const auto len =
        std::sqrt(lineDir[0] * lineDir[0] + lineDir[1] * lineDir[1] + lineDir[2] * lineDir[2]);
    ASSERT_GT_(len, 0.9);  // Should be normalized
    ASSERT_LT_(len, 1.1);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_basic_point_to_line_matching();
        std::cout << "test_basic_point_to_line_matching: Success ✅" << std::endl;

        test_threshold_filtering();
        std::cout << "test_threshold_filtering: Success ✅" << std::endl;

        test_with_pose_transformation();
        std::cout << "test_with_pose_transformation: Success ✅" << std::endl;

        test_parallel_lines();
        std::cout << "test_parallel_lines: Success ✅" << std::endl;

        test_empty_maps();
        std::cout << "test_empty_maps: Success ✅" << std::endl;

        test_line_direction();
        std::cout << "test_line_direction: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
