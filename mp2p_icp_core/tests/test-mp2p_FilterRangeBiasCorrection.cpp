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
 * @file   test-mp2p_FilterRangeBiasCorrection.cpp
 * @brief  Unit test for FilterRangeBiasCorrection
 * @author Jose Luis Blanco Claraco
 * @date   Sep 24, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterRangeBiasCorrection.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{
mrpt::containers::yaml makeCoefs(double a, double b, double c)
{
    return mrpt::containers::yaml::Sequence({a, b, c});
}

// Vertical wall x = 10, 4 x 4 m, 0.1 m grid, sensor at the origin.
mrpt::maps::CSimplePointsMap::Ptr makeWall()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (int iy = -20; iy <= 20; iy++)
    {
        for (int iz = -20; iz <= 20; iz++)
        {
            pc->insertPoint(10.0f, 0.1f * static_cast<float>(iy), 0.1f * static_cast<float>(iz));
        }
    }
    return pc;
}

// Horizontal ground z = -1, x in [2, 6], y in [-2, 2], 0.1 m grid.
mrpt::maps::CSimplePointsMap::Ptr makeGround()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (int ix = 20; ix <= 60; ix++)
    {
        for (int iy = -20; iy <= 20; iy++)
        {
            pc->insertPoint(0.1f * static_cast<float>(ix), 0.1f * static_cast<float>(iy), -1.0f);
        }
    }
    return pc;
}

metric_map_t runFilter(
    const mrpt::maps::CSimplePointsMap::Ptr& pc, const mrpt::containers::yaml& extraParams)
{
    FilterRangeBiasCorrection filter;

    mrpt::containers::yaml params = extraParams;
    params["pointcloud_layer"]    = "raw";
    filter.initialize(params);

    metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);
    return map;
}

// A constant wall offset of -1 cm (measured too near) pushes every point of
// the wall 1 cm away from the sensor, along the wall normal.
void test_wall_constant_offset()
{
    auto                   pc = makeWall();
    mrpt::containers::yaml p;
    p["wall"] = makeCoefs(-0.01, 0, 0);

    auto map = runFilter(pc, p);
    auto out = map.point_layer("raw");
    ASSERT_EQUAL_(out->size(), 41UL * 41UL);

    const auto& xs = out->getPointsBufferRef_x();
    for (size_t i = 0; i < xs.size(); i++)
    {
        ASSERT_NEAR_(xs[i], 10.01f, 1e-4f);
    }
}

// The incidence term on ground: d = c sin^2(theta), shift along -z (away from
// a sensor above the plane).
void test_ground_incidence_term()
{
    auto         pc = makeGround();
    const double c  = -0.02;

    mrpt::containers::yaml p;
    p["ground"] = makeCoefs(0, 0, c);

    const auto xs0 = pc->getPointsBufferRef_x();
    const auto ys0 = pc->getPointsBufferRef_y();

    auto        map = runFilter(pc, p);
    auto        out = map.point_layer("raw");
    const auto& zs  = out->getPointsBufferRef_z();

    for (size_t i = 0; i < zs.size(); i++)
    {
        const double r2    = xs0[i] * xs0[i] + ys0[i] * ys0[i] + 1.0;
        const double cos2  = 1.0 / r2;
        const double sin2  = 1.0 - cos2;
        const double zWant = -1.0 + c * sin2;
        ASSERT_NEAR_(zs[i], zWant, 1e-4);
    }
}

// All-zero coefficients (the default) must leave the cloud untouched.
void test_zero_coefficients()
{
    auto       pc  = makeWall();
    const auto xs0 = pc->getPointsBufferRef_x();

    auto        map = runFilter(pc, mrpt::containers::yaml::Map());
    const auto& xs  = map.point_layer("raw")->getPointsBufferRef_x();
    for (size_t i = 0; i < xs.size(); i++)
    {
        ASSERT_EQUAL_(xs[i], xs0[i]);
    }
}

// A 3D lattice has no planar neighborhoods: nothing is corrected.
void test_non_planar_untouched()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (int ix = 0; ix < 10; ix++)
    {
        for (int iy = 0; iy < 10; iy++)
        {
            for (int iz = 0; iz < 10; iz++)
            {
                pc->insertPoint(
                    5.0f + 0.1f * static_cast<float>(ix), 0.1f * static_cast<float>(iy),
                    0.1f * static_cast<float>(iz));
            }
        }
    }
    const auto xs0 = pc->getPointsBufferRef_x();
    const auto ys0 = pc->getPointsBufferRef_y();
    const auto zs0 = pc->getPointsBufferRef_z();

    mrpt::containers::yaml p;
    p["ground"]  = makeCoefs(0.01, 0, 0);
    p["slanted"] = makeCoefs(0.01, 0, 0);
    p["wall"]    = makeCoefs(0.01, 0, 0);

    auto        map = runFilter(pc, p);
    auto        out = map.point_layer("raw");
    const auto& xs  = out->getPointsBufferRef_x();
    const auto& ys  = out->getPointsBufferRef_y();
    const auto& zs  = out->getPointsBufferRef_z();

    // Lattice corners and edges may look planar; the interior must not move.
    size_t nInterior = 0;
    for (size_t i = 0; i < xs.size(); i++)
    {
        const bool interior = xs0[i] > 5.05f && xs0[i] < 5.85f && ys0[i] > 0.05f &&
                              ys0[i] < 0.85f && zs0[i] > 0.05f && zs0[i] < 0.85f;
        if (!interior)
        {
            continue;
        }
        nInterior++;
        ASSERT_EQUAL_(xs[i], xs0[i]);
        ASSERT_EQUAL_(ys[i], ys0[i]);
        ASSERT_EQUAL_(zs[i], zs0[i]);
    }
    ASSERT_GT_(nInterior, 0UL);
}

// Corrections are clamped to max_correction.
void test_clamp()
{
    auto                   pc = makeWall();
    mrpt::containers::yaml p;
    p["wall"]           = makeCoefs(-1.0, 0, 0);
    p["max_correction"] = 0.02;

    auto        map = runFilter(pc, p);
    const auto& xs  = map.point_layer("raw")->getPointsBufferRef_x();
    for (size_t i = 0; i < xs.size(); i++)
    {
        ASSERT_NEAR_(xs[i], 10.02f, 1e-4f);
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_wall_constant_offset();
        std::cout << "test_wall_constant_offset: Success" << std::endl;

        test_ground_incidence_term();
        std::cout << "test_ground_incidence_term: Success" << std::endl;

        test_zero_coefficients();
        std::cout << "test_zero_coefficients: Success" << std::endl;

        test_non_planar_untouched();
        std::cout << "test_non_planar_untouched: Success" << std::endl;

        test_clamp();
        std::cout << "test_clamp: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}
