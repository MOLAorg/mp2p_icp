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
 * @file   test-mp2p_matcher_points_blend.cpp
 * @brief  Unit tests for Matcher_Points_Blend
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_Points_Blend.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

namespace
{
mrpt::maps::CSimplePointsMap::Ptr mapOf(const std::vector<mrpt::math::TPoint3D>& pts)
{
    auto m = mrpt::maps::CSimplePointsMap::Create();
    for (const auto& p : pts)
    {
        m->insertPointFast(
            static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }
    m->mark_as_modified();
    return m;
}

/// Runs the matcher for one query point and returns the target it paired with
std::optional<mrpt::math::TPoint3Df> targetFor(
    mp2p_icp::Matcher& m, const mrpt::maps::CSimplePointsMap::Ptr& global, const double qx,
    const double qy, const double qz)
{
    mp2p_icp::metric_map_t pcGlobal;
    pcGlobal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = global;

    auto localPts = mrpt::maps::CSimplePointsMap::Create();
    localPts->insertPointFast(
        static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz));

    mp2p_icp::metric_map_t pcLocal;
    pcLocal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = localPts;

    mp2p_icp::Pairings   pairs;
    mp2p_icp::MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    if (pairs.paired_pt2pt.empty())
    {
        return {};
    }
    return pairs.paired_pt2pt.at(0).global;
}

mp2p_icp::Matcher_Points_Blend::Ptr makeBlend(
    const double temperature, const double searchRadius, const bool smoothCutoff,
    const double threshold = 10.0)
{
    auto m = mp2p_icp::Matcher_Points_Blend::Create();

    mrpt::containers::yaml p;
    p["threshold"]                            = threshold;
    p["thresholdAngularDeg"]                  = 0.0;
    p["temperature"]                          = temperature;
    p["searchRadius"]                         = searchRadius;
    p["smoothCutoff"]                         = smoothCutoff;
    p["allowMatchAlreadyMatchedGlobalPoints"] = true;
    m->initialize(p);

    return m;
}

/// Largest target jump between consecutive samples of a straight sweep in x.
double maxTargetStepAlongX(
    mp2p_icp::Matcher& m, const mrpt::maps::CSimplePointsMap::Ptr& global, const double x0,
    const double x1, const int nSteps, const double qy, const double qz)
{
    double                               worst = 0;
    std::optional<mrpt::math::TPoint3Df> prev;

    for (int i = 0; i <= nSteps; i++)
    {
        const double x = x0 + (x1 - x0) * i / static_cast<double>(nSteps);
        const auto   t = targetFor(m, global, x, qy, qz);
        if (t && prev)
        {
            const double dx = t->x - prev->x;
            const double dy = t->y - prev->y;
            const double dz = t->z - prev->z;
            worst           = std::max(worst, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        prev = t;
    }
    return worst;
}

/// Two map points separated in x. A query gliding along +x at a small offset
/// crosses their perpendicular bisector halfway, which is where the
/// nearest-neighbor target jumps by the full separation.
mrpt::maps::CSimplePointsMap::Ptr twoCompetingPoints()
{
    return mapOf({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
}

void test_zero_temperature_is_the_nearest_neighbor()
{
    auto global = twoCompetingPoints();
    auto blend  = makeBlend(0.0, 3.0, true);

    auto ref = mp2p_icp::Matcher_Points_DistanceThreshold::Create();
    {
        mrpt::containers::yaml p;
        p["threshold"]                            = 10.0;
        p["thresholdAngularDeg"]                  = 0.0;
        p["pairingsPerPoint"]                     = 1;
        p["allowMatchAlreadyMatchedGlobalPoints"] = true;
        ref->initialize(p);
    }

    for (double x = -0.5; x <= 1.5; x += 0.05)
    {
        const auto got = targetFor(*blend, global, x, 0.0, 0.05);
        const auto exp = targetFor(*ref, global, x, 0.0, 0.05);

        ASSERT_(got.has_value() == exp.has_value());
        if (!got)
        {
            continue;
        }
        ASSERT_NEAR_(got->x, exp->x, 1e-12);
        ASSERT_NEAR_(got->y, exp->y, 1e-12);
        ASSERT_NEAR_(got->z, exp->z, 1e-12);
    }
    std::cout << "test_zero_temperature_is_the_nearest_neighbor: PASSED\n";
}

void test_low_temperature_tends_to_the_nearest_neighbor()
{
    auto global = twoCompetingPoints();
    auto blend  = makeBlend(1e-3, 3.0, true);

    // Away from the bisector the softmax is saturated, so the blend and the
    // nearest neighbor have to agree.
    for (const double x : {-0.4, -0.2, 0.1, 0.9, 1.2, 1.4})
    {
        const auto got = targetFor(*blend, global, x, 0.0, 0.05);
        ASSERT_(got.has_value());

        const double nearest = (x < 0.5) ? 0.0 : 1.0;
        ASSERT_NEAR_(got->x, nearest, 1e-6);
    }
    std::cout << "test_low_temperature_tends_to_the_nearest_neighbor: PASSED\n";
}

void test_target_is_continuous_across_the_bisector()
{
    auto global = twoCompetingPoints();

    // The discrete reference: the nearest-neighbor target jumps the whole 1 m
    // separation as the query crosses x=0.5.
    auto         hard     = makeBlend(0.0, 3.0, true);
    const double jumpHard = maxTargetStepAlongX(*hard, global, 0.2, 0.8, 600, 0.0, 0.05);
    ASSERT_GT_(jumpHard, 0.9);

    // The blend crosses it continuously: with 600 samples over 0.6 m the step
    // has to stay far below the separation it replaces.
    auto         smooth     = makeBlend(0.3, 3.0, true);
    const double jumpSmooth = maxTargetStepAlongX(*smooth, global, 0.2, 0.8, 600, 0.0, 0.05);
    ASSERT_LT_(jumpSmooth, 0.02);

    std::cout << "test_target_is_continuous_across_the_bisector: PASSED (hard " << jumpHard
              << " -> smooth " << jumpSmooth << ")\n";
}

void test_target_is_continuous_as_a_point_leaves_the_radius()
{
    // One neighbor sits far enough out that a query sweeping in x pushes it
    // across searchRadius partway through the sweep.
    auto global = mapOf({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}});

    // searchRadius 1.2: the second point is inside the window at x=0 and
    // outside it once the query has moved far enough along +x.
    auto         smooth     = makeBlend(0.5, 1.2, true);
    const double jumpSmooth = maxTargetStepAlongX(*smooth, global, 0.0, 1.2, 1200, 0.0, 0.0);

    auto         hardEdge = makeBlend(0.5, 1.2, false);
    const double jumpHard = maxTargetStepAlongX(*hardEdge, global, 0.0, 1.2, 1200, 0.0, 0.0);

    // The taper is the whole reason the smooth arm is smooth here, so it must
    // beat the truncated one by a wide margin.
    ASSERT_LT_(jumpSmooth, 0.2 * jumpHard);

    std::cout << "test_target_is_continuous_as_a_point_leaves_the_radius: PASSED (hard edge "
              << jumpHard << " -> tapered " << jumpSmooth << ")\n";
}

void test_blend_shrinks_toward_the_neighborhood_interior()
{
    // The known and intrinsic cost of the formulation, asserted so that it is a
    // documented property rather than a surprise: at a temperature comparable
    // to the point spacing the target leaves the surface and moves toward the
    // local centroid.
    auto global = mapOf({{0.0, -0.5, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.5, 0.0}});

    auto       smooth = makeBlend(1.0, 3.0, true);
    const auto t      = targetFor(*smooth, global, 0.3, 0.5, 0.0);
    ASSERT_(t.has_value());

    // The nearest map point is at y=0.5; the blend pulls the target back
    // toward y=0, i.e. into the interior of the neighborhood.
    ASSERT_LT_(t->y, 0.45);

    std::cout << "test_blend_shrinks_toward_the_neighborhood_interior: PASSED (y=" << t->y << ")\n";
}
}  // namespace

int main(int argc, const char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_zero_temperature_is_the_nearest_neighbor();
        test_low_temperature_tends_to_the_nearest_neighbor();
        test_target_is_continuous_across_the_bisector();
        test_target_is_continuous_as_a_point_leaves_the_radius();
        test_blend_shrinks_toward_the_neighborhood_interior();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
