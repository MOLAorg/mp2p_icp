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
 * @file   test-mp2p_FilterPlanePatches.cpp
 * @brief  Unit tests for FilterPlanePatches, on a synthetic room
 * @author Jose Luis Blanco Claraco
 * @date   Sep 1, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterPlanePatches.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
constexpr double ROOM_X = 6.0, ROOM_Y = 4.0, ROOM_Z = 2.5;

/** A closed box sampled densely on all six faces, plus a little noise, with
 *  the sensor at the middle of the room so every face is inside range. */
mp2p_icp::metric_map_t makeRoom(double tiltRad = 0.0, double noise = 0.005)
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234U);

    auto pc = mrpt::maps::CSimplePointsMap::Create();

    const double c = std::cos(tiltRad), s = std::sin(tiltRad);
    const auto   add = [&](double x, double y, double z)
    {
        x += rng.drawGaussian1D(0, noise);
        y += rng.drawGaussian1D(0, noise);
        z += rng.drawGaussian1D(0, noise);
        // rotate about +Y, i.e. a pure pitch of the whole room
        pc->insertPointFast(c * x + s * z, y, -s * x + c * z);
    };

    const double step = 0.04;
    for (double x = -ROOM_X / 2; x <= ROOM_X / 2; x += step)
        for (double y = -ROOM_Y / 2; y <= ROOM_Y / 2; y += step)
        {
            add(x, y, -ROOM_Z / 2);  // floor
            add(x, y, +ROOM_Z / 2);  // ceiling
        }
    for (double x = -ROOM_X / 2; x <= ROOM_X / 2; x += step)
        for (double z = -ROOM_Z / 2; z <= ROOM_Z / 2; z += step)
        {
            add(x, -ROOM_Y / 2, z);  // two walls normal to Y
            add(x, +ROOM_Y / 2, z);
        }
    for (double y = -ROOM_Y / 2; y <= ROOM_Y / 2; y += step)
        for (double z = -ROOM_Z / 2; z <= ROOM_Z / 2; z += step)
        {
            add(-ROOM_X / 2, y, z);  // two walls normal to X
            add(+ROOM_X / 2, y, z);
        }
    pc->mark_as_modified();

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = pc;
    return m;
}

mp2p_icp_filters::FilterPlanePatches makeFilter()
{
    mp2p_icp_filters::FilterPlanePatches f;
    // range_min has to clear nothing here: the sensor sits inside the room and
    // the nearest face is ROOM_Z/2 = 1.25 m away.
    f.initialize(mrpt::containers::yaml::FromText(R"(
      input_pointcloud_layer: 'raw'
      voxel_size: 0.10
      distance_threshold: 0.06
      normal_agreement_deg: 12.0
      min_points: 100
      min_span: 1.0
      range_min: 0.5
      range_max: 60.0
      max_patches: 12
    )"));
    return f;
}

void test_finds_the_six_faces()
{
    auto       m = makeRoom();
    const auto f = makeFilter();
    f.filter(m);

    ASSERT_EQUAL_(m.planes.size(), 6UL);

    // Every face must come out axis-aligned, and the areas must match the
    // geometry: two of ROOM_X*ROOM_Y, two of ROOM_X*ROOM_Z, two of
    // ROOM_Y*ROOM_Z, each to within the voxel quantization.
    size_t nHoriz = 0, nVert = 0;
    double totalArea = 0;
    for (const auto& p : m.planes)
    {
        const auto&  n  = p.plane.coefs;
        const double nn = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double nz = std::abs(n[2]) / nn;
        if (nz > std::cos(mrpt::DEG2RAD(2.0)))
            nHoriz++;
        else if (nz < std::sin(mrpt::DEG2RAD(2.0)))
            nVert++;
        ASSERT_GT_(p.area, 0.0);
        ASSERT_GT_(p.num_points, 100U);
        totalArea += p.area;
    }
    ASSERT_EQUAL_(nHoriz, 2UL);
    ASSERT_EQUAL_(nVert, 4UL);

    const double expected = 2 * ROOM_X * ROOM_Y + 2 * ROOM_X * ROOM_Z + 2 * ROOM_Y * ROOM_Z;
    ASSERT_LT_(std::abs(totalArea - expected) / expected, 0.15);
}

void test_normals_follow_a_tilted_room()
{
    // The whole room pitched by 5 deg: the patches must report the same tilt,
    // which is the property the structural verticality reference depends on.
    const double tilt = mrpt::DEG2RAD(5.0);
    auto         m    = makeRoom(tilt);
    const auto   f    = makeFilter();
    f.filter(m);

    ASSERT_EQUAL_(m.planes.size(), 6UL);

    size_t nAtTilt = 0;
    for (const auto& p : m.planes)
    {
        const auto&  n  = p.plane.coefs;
        const double nn = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double nz = std::abs(n[2]) / nn;
        // floors/ceilings now sit at `tilt` from vertical, the X-normal walls
        // at `tilt` from horizontal; the Y-normal walls are unmoved.
        const double a = mrpt::RAD2DEG(std::acos(std::min(1.0, nz)));
        if (std::abs(a - 5.0) < 0.5 || std::abs(a - 85.0) < 0.5) nAtTilt++;
    }
    ASSERT_EQUAL_(nAtTilt, 4UL);
}

void test_is_deterministic()
{
    const auto f = makeFilter();
    auto       a = makeRoom();
    auto       b = makeRoom();
    f.filter(a);
    f.filter(b);
    ASSERT_EQUAL_(a.planes.size(), b.planes.size());
    for (size_t i = 0; i < a.planes.size(); i++)
    {
        ASSERT_EQUAL_(a.planes[i].num_points, b.planes[i].num_points);
        for (int k = 0; k < 4; k++)
            ASSERT_LT_(std::abs(a.planes[i].plane.coefs[k] - b.planes[i].plane.coefs[k]), 1e-12);
    }
}

void test_is_independent_of_input_order()
{
    // The contract is not just "same run twice": an upstream parallel stage may
    // hand the same points over in a different order, and the patches must not
    // move because of it.
    const auto f = makeFilter();

    auto a = makeRoom();
    auto b = makeRoom();

    {
        auto& pcB      = *std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(b.layers["raw"]);
        const size_t n = pcB.size();
        auto&        rng = mrpt::random::getRandomGenerator();
        rng.randomize(7U);
        std::vector<mrpt::math::TPoint3D> pts;
        pts.reserve(n);
        for (size_t i = 0; i < n; i++)
        {
            mrpt::math::TPoint3D p;
            pcB.getPoint(i, p.x, p.y, p.z);
            pts.push_back(p);
        }
        for (size_t i = n; i > 1; i--)
        {
            const auto j = static_cast<size_t>(rng.drawUniform32bit() % i);
            std::swap(pts[i - 1], pts[j]);
        }
        pcB.clear();
        for (const auto& p : pts) pcB.insertPointFast(p.x, p.y, p.z);
        pcB.mark_as_modified();
    }

    f.filter(a);
    f.filter(b);

    ASSERT_EQUAL_(a.planes.size(), b.planes.size());
    for (size_t i = 0; i < a.planes.size(); i++)
    {
        ASSERT_EQUAL_(a.planes[i].num_points, b.planes[i].num_points);
        ASSERT_LT_(std::abs(a.planes[i].area - b.planes[i].area), 1e-9);
        for (int k = 0; k < 4; k++)
        {
            ASSERT_LT_(std::abs(a.planes[i].plane.coefs[k] - b.planes[i].plane.coefs[k]), 1e-12);
        }
    }
}

void test_min_span_rejects_a_sliver()
{
    // A 6 x 0.3 m strip: plenty of points and perfectly planar, but too narrow
    // to be evidence of a surface.
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (double x = -3.0; x <= 3.0; x += 0.02)
        for (double y = -0.15; y <= 0.15; y += 0.02) pc->insertPointFast(x, y, 2.0);
    pc->mark_as_modified();

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = pc;

    const auto f = makeFilter();
    f.filter(m);
    ASSERT_EQUAL_(m.planes.size(), 0UL);
}

void test_empty_and_missing_layer()
{
    const auto f = makeFilter();

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = mrpt::maps::CSimplePointsMap::Create();
    f.filter(m);  // must not throw on an empty cloud
    ASSERT_EQUAL_(m.planes.size(), 0UL);

    mp2p_icp::metric_map_t m2;
    bool                   threw = false;
    try
    {
        f.filter(m2);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);
}
}  // namespace

int main()
{
    try
    {
        test_finds_the_six_faces();
        std::cout << "test_finds_the_six_faces: Success" << std::endl;

        test_normals_follow_a_tilted_room();
        std::cout << "test_normals_follow_a_tilted_room: Success" << std::endl;

        test_is_deterministic();
        std::cout << "test_is_deterministic: Success" << std::endl;

        test_is_independent_of_input_order();
        std::cout << "test_is_independent_of_input_order: Success" << std::endl;

        test_min_span_rejects_a_sliver();
        std::cout << "test_min_span_rejects_a_sliver: Success" << std::endl;

        test_empty_and_missing_layer();
        std::cout << "test_empty_and_missing_layer: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
