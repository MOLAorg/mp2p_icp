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
 * @file   test-mp2p_FilterTraversability.cpp
 * @brief  Unit tests for FilterTraversability
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterTraversability.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>
#include <iostream>

namespace
{
// Ground of the synthetic scene. Strip A (y in [0,6)): flat, a 10 deg ramp up, a
// plateau, a 25 deg slope up and a higher plateau. Strip B (y in [9,13)): flat,
// then a 0.5 m step up. There are no points in between (never observed).
float groundStripA(float x)
{
    const float tan10 = std::tan(mrpt::DEG2RAD(10.0f));
    const float tan25 = std::tan(mrpt::DEG2RAD(25.0f));
    if (x < 8.0f)
    {
        return 0.0f;
    }
    if (x < 12.0f)
    {
        return (x - 8.0f) * tan10;
    }
    if (x < 16.0f)
    {
        return 4.0f * tan10;
    }
    if (x < 18.0f)
    {
        return 4.0f * tan10 + (x - 16.0f) * tan25;
    }
    return 4.0f * tan10 + 2.0f * tan25;
}

float groundStripB(float x) { return x < 8.0f ? 0.0f : 0.5f; }

bool hasPointNear(const mrpt::maps::CPointsMap& pc, float x, float y, float tol = 0.15f)
{
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    for (size_t i = 0; i < xs.size(); i++)
    {
        if (std::abs(xs[i] - x) <= tol && std::abs(ys[i] - y) <= tol)
        {
            return true;
        }
    }
    return false;
}

void buildScene(mp2p_icp::metric_map_t& map)
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (float x = 0.0f; x < 22.0f; x += 0.05f)
    {
        for (float y = 0.0f; y < 6.0f; y += 0.05f)
        {
            pc->insertPoint(x, y, groundStripA(x));
        }
        if (x < 12.0f)
        {
            for (float y = 9.0f; y < 13.0f; y += 0.05f)
            {
                pc->insertPoint(x, y, groundStripB(x));
            }
        }
    }
    // A box at the height of the vehicle body, on the flat part:
    for (float x = 4.0f; x < 4.4f; x += 0.05f)
    {
        for (float y = 1.0f; y < 1.4f; y += 0.05f)
        {
            for (float z = 0.3f; z < 1.0f; z += 0.05f)
            {
                pc->insertPoint(x, y, z);
            }
        }
    }
    map.layers["raw"] = pc;

    // Seeds: the "trajectory" on the flat part of each strip:
    auto seeds = mrpt::maps::CSimplePointsMap::Create();
    for (float x = 1.0f; x < 7.0f; x += 0.5f)
    {
        seeds->insertPoint(x, 3.0f, 0.13f);
        seeds->insertPoint(x, 11.0f, 0.13f);
    }
    map.layers["seeds"] = seeds;
}

void testWithSeeds()
{
    mp2p_icp::metric_map_t map;
    buildScene(map);

    mp2p_icp_filters::FilterTraversability filter;
    mrpt::containers::yaml                 p;
    p["input_pointcloud_layer"]   = "raw";
    p["seeds_pointcloud_layer"]   = "seeds";
    p["output_layer_obstacles"]   = "obstacles";
    p["output_layer_traversable"] = "traversable";
    filter.initialize(p);
    filter.filter(map);

    const auto obs  = map.point_layer("obstacles");
    const auto trav = map.point_layer("traversable");
    ASSERT_(obs && trav);
    std::cout << "obstacle cells: " << obs->size() << ", traversable: " << trav->size()
              << std::endl;

    // Gentle (10 deg) ramp and the plateau after it: traversable.
    ASSERT_(hasPointNear(*trav, 10.0f, 3.0f));
    ASSERT_(hasPointNear(*trav, 14.0f, 3.0f));
    ASSERT_(!hasPointNear(*obs, 10.0f, 3.0f));

    // Steep (25 deg) slope: obstacle, and the plateau above it is not reachable.
    ASSERT_(hasPointNear(*obs, 17.0f, 3.0f));
    ASSERT_(!hasPointNear(*trav, 20.0f, 3.0f));

    // Box at body height: obstacle.
    ASSERT_(hasPointNear(*obs, 4.2f, 1.2f));
    ASSERT_(!hasPointNear(*trav, 4.2f, 1.2f));

    // 0.5 m step: obstacle, and not reachable above it.
    ASSERT_(hasPointNear(*obs, 8.0f, 11.0f, 0.3f));
    ASSERT_(!hasPointNear(*trav, 10.0f, 11.0f));

    // The never observed band between strips (y in [6,9)) borders the reachable
    // region after the ground fill (ground_fill_distance=1 m) of each strip, and
    // its middle is not traversable:
    ASSERT_(hasPointNear(*obs, 3.0f, 7.1f, 0.2f));
    for (size_t i = 0; i < trav->size(); i++)
    {
        const float y = trav->getPointsBufferRef_y()[i];
        ASSERT_(y < 7.05f || y > 7.95f);
    }
}

void testWithoutSeeds()
{
    mp2p_icp::metric_map_t map;
    buildScene(map);

    mp2p_icp_filters::FilterTraversability filter;
    mrpt::containers::yaml                 p;
    p["input_pointcloud_layer"]   = "raw";
    p["output_layer_obstacles"]   = "obstacles";
    p["output_layer_traversable"] = "traversable";
    p["max_slope_deg"]            = "5*2+10";  // formulas are accepted
    filter.initialize(p);

    mp2p_icp::ParameterSource ps;
    filter.attachToParameterSource(ps);
    ps.realize();
    ASSERT_NEAR_(filter.params.max_slope_deg, 20.0f, 1e-5f);

    filter.filter(map);

    const auto obs  = map.point_layer("obstacles");
    const auto trav = map.point_layer("traversable");
    ASSERT_(obs && trav);

    // Per cell classification only: the upper plateaus are traversable, the
    // slope, the box and the step edge are not.
    ASSERT_(hasPointNear(*trav, 20.0f, 3.0f));
    ASSERT_(hasPointNear(*trav, 10.0f, 3.0f));
    ASSERT_(hasPointNear(*obs, 17.0f, 3.0f));
    ASSERT_(hasPointNear(*obs, 4.2f, 1.2f));
    ASSERT_(hasPointNear(*obs, 8.0f, 11.0f, 0.3f));
}

// Flat ground under a tree canopy: the ground was only seen every 2 m (scan
// rings), and most cells only have canopy points. It must stay traversable.
void testUnderCanopy()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (float x = 0.0f; x < 16.0f; x += 0.05f)
    {
        for (float y = 0.0f; y < 6.0f; y += 0.05f)
        {
            const bool underCanopy = x >= 5.0f;
            const bool groundRing  = std::fmod(x, 2.0f) < 0.1f;
            if (!underCanopy || groundRing)
            {
                pc->insertPoint(x, y, 0.0f);
            }
            if (underCanopy)
            {
                pc->insertPoint(x, y, 2.5f + 0.5f * std::sin(x + y));
            }
        }
    }
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;
    auto seeds        = mrpt::maps::CSimplePointsMap::Create();
    for (float x = 1.0f; x < 4.0f; x += 0.5f)
    {
        seeds->insertPoint(x, 3.0f, 0.13f);
    }
    map.layers["seeds"] = seeds;

    mp2p_icp_filters::FilterTraversability filter;
    mrpt::containers::yaml                 p;
    p["input_pointcloud_layer"]   = "raw";
    p["seeds_pointcloud_layer"]   = "seeds";
    p["output_layer_obstacles"]   = "obstacles";
    p["output_layer_traversable"] = "traversable";
    filter.initialize(p);
    filter.filter(map);

    const auto obs  = map.point_layer("obstacles");
    const auto trav = map.point_layer("traversable");
    ASSERT_(obs && trav);
    for (float x = 6.0f; x < 14.0f; x += 0.5f)
    {
        ASSERT_(hasPointNear(*trav, x, 3.0f));
        ASSERT_(!hasPointNear(*obs, x, 3.0f));
    }
}

}  // namespace

int main()
{
    try
    {
        testWithSeeds();
        testWithoutSeeds();
        testUnderCanopy();
        std::cout << "FilterTraversability Unit Test Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
