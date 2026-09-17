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
 * @file   test-mp2p_matcher_adaptive.cpp
 * @brief  Unit tests for Matcher_Adaptive
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/Matcher_Adaptive.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{
mrpt::containers::yaml basicParams()
{
    mrpt::containers::yaml p;
    p["confidenceInterval"]        = 0.9;
    p["firstToSecondDistanceMax"]  = 2.0;
    p["absoluteMaxSearchDistance"] = 1.0;
    p["enableDetectPlanes"]        = false;
    return p;
}

/** A dense grid of global points and a handful of local points, each of them
 *  very close to one grid point: all must end up paired point-to-point. */
void test_basic_pt2pt_matching()
{
    auto global = mrpt::maps::CSimplePointsMap::Create();
    for (int i = 0; i < 20; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            global->insertPoint(static_cast<float>(i) * 0.1f, static_cast<float>(j) * 0.1f, 0.0f);
        }
    }

    auto local = mrpt::maps::CSimplePointsMap::Create();
    local->insertPoint(0.501f, 0.501f, 0.0f);
    local->insertPoint(1.001f, 1.001f, 0.0f);

    metric_map_t pcGlobal, pcLocal;
    pcGlobal.layers[metric_map_t::PT_LAYER_RAW] = global;
    pcLocal.layers[metric_map_t::PT_LAYER_RAW]  = local;

    Matcher_Adaptive m;
    m.initialize(basicParams());

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, mrpt::poses::CPose3D::Identity(), {}, ms, pairs);

    ASSERT_EQUAL_(pairs.paired_pt2pt.size(), 2U);
    ASSERT_(pairs.paired_pt2pl.empty());
}

/** Two point clouds with non-overlapping bounding boxes must produce no
 *  pairings, and must not throw. */
void test_disjoint_clouds_produce_no_pairings()
{
    auto global = mrpt::maps::CSimplePointsMap::Create();
    global->insertPoint(0.0f, 0.0f, 0.0f);
    global->insertPoint(1.0f, 1.0f, 0.0f);

    auto local = mrpt::maps::CSimplePointsMap::Create();
    local->insertPoint(100.0f, 100.0f, 0.0f);

    metric_map_t pcGlobal, pcLocal;
    pcGlobal.layers[metric_map_t::PT_LAYER_RAW] = global;
    pcLocal.layers[metric_map_t::PT_LAYER_RAW]  = local;

    Matcher_Adaptive m;
    m.initialize(basicParams());

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, mrpt::poses::CPose3D::Identity(), {}, ms, pairs);

    ASSERT_(pairs.empty());
}

/** Regression test: bounding boxes may overlap (within the internal overlap
 *  epsilon) while no individual point actually has a neighbor within
 *  `absoluteMaxSearchDistance`. Before the fix, this dereferenced an empty
 *  std::optional building the adaptive-threshold histogram. */
void test_overlapping_boxes_but_no_close_neighbors_does_not_crash()
{
    // Two global points 10m apart give a bounding box that fully CONTAINS the
    // local point's bounding box below, so the bbox-overlap pre-check passes
    // and the per-point neighbor search actually runs. But absoluteMaxSearch-
    // Distance=1.0 while both global points are 5m away from it, so no
    // individual point ever finds a neighbor:
    auto global = mrpt::maps::CSimplePointsMap::Create();
    global->insertPoint(0.0f, 0.0f, 0.0f);
    global->insertPoint(10.0f, 0.0f, 0.0f);

    auto local = mrpt::maps::CSimplePointsMap::Create();
    local->insertPoint(5.0f, 0.0f, 0.0f);

    metric_map_t pcGlobal, pcLocal;
    pcGlobal.layers[metric_map_t::PT_LAYER_RAW] = global;
    pcLocal.layers[metric_map_t::PT_LAYER_RAW]  = local;

    Matcher_Adaptive m;
    m.initialize(basicParams());

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    // Must not crash/UB, and must simply find nothing:
    m.match(pcGlobal, pcLocal, mrpt::poses::CPose3D::Identity(), {}, ms, pairs);

    ASSERT_(pairs.empty());
}

/** Regression test: a single local point (or several tied at the same
 *  distance) makes the min/max of the adaptive-threshold histogram equal.
 *  Before the fix, mrpt::math::CHistogram(min,max,...) threw on max<=min. */
void test_single_local_point_does_not_throw_on_degenerate_histogram_range()
{
    auto global = mrpt::maps::CSimplePointsMap::Create();
    global->insertPoint(0.0f, 0.0f, 0.0f);
    global->insertPoint(5.0f, 5.0f, 0.0f);

    auto local = mrpt::maps::CSimplePointsMap::Create();
    local->insertPoint(0.05f, 0.0f, 0.0f);  // single local point

    metric_map_t pcGlobal, pcLocal;
    pcGlobal.layers[metric_map_t::PT_LAYER_RAW] = global;
    pcLocal.layers[metric_map_t::PT_LAYER_RAW]  = local;

    Matcher_Adaptive m;
    m.initialize(basicParams());

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, mrpt::poses::CPose3D::Identity(), {}, ms, pairs);

    ASSERT_EQUAL_(pairs.paired_pt2pt.size(), 1U);
}

/** A local point next to a well-conditioned planar patch of global points
 *  must produce a point-to-plane pairing when enableDetectPlanes=true. */
void test_plane_detection()
{
    auto global = mrpt::maps::CSimplePointsMap::Create();
    // A flat, well-sampled patch on z=0 around the origin:
    for (int i = -3; i <= 3; i++)
    {
        for (int j = -3; j <= 3; j++)
        {
            global->insertPoint(static_cast<float>(i) * 0.05f, static_cast<float>(j) * 0.05f, 0.0f);
        }
    }

    auto local = mrpt::maps::CSimplePointsMap::Create();
    // Slightly above the plane, close to its center:
    local->insertPoint(0.0f, 0.0f, 0.02f);

    metric_map_t pcGlobal, pcLocal;
    pcGlobal.layers[metric_map_t::PT_LAYER_RAW] = global;
    pcLocal.layers[metric_map_t::PT_LAYER_RAW]  = local;

    Matcher_Adaptive       m;
    mrpt::containers::yaml p     = basicParams();
    p["enableDetectPlanes"]      = true;
    p["planeSearchPoints"]       = 8;
    p["planeMinimumFoundPoints"] = 4;
    p["planeEigenThreshold"]     = 0.05;
    p["planeMinimumDistance"]    = 0.5;
    m.initialize(p);

    Pairings   pairs;
    MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, mrpt::poses::CPose3D::Identity(), {}, ms, pairs);

    ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 1U);
    // The fitted plane's normal must be close to vertical (z=0 patch):
    const auto n = pairs.paired_pt2pl.at(0).pl_global.plane.getNormalVector();
    ASSERT_GT_(std::abs(n.z), 0.9);
}

/** Parameter validation: an out-of-range confidenceInterval must throw at
 *  initialize() time. */
void test_invalid_confidence_interval_throws()
{
    mrpt::containers::yaml p = basicParams();
    p["confidenceInterval"]  = 1.5;  // out of (0,1)

    Matcher_Adaptive m;
    bool             didThrow = false;
    try
    {
        m.initialize(p);
    }
    catch (const std::exception&)
    {
        didThrow = true;
    }
    ASSERT_(didThrow);
}

/** Parameter validation: planeSearchPoints must be >= planeMinimumFoundPoints. */
void test_inconsistent_plane_points_throws()
{
    mrpt::containers::yaml p     = basicParams();
    p["enableDetectPlanes"]      = true;
    p["planeSearchPoints"]       = 3;
    p["planeMinimumFoundPoints"] = 4;  // > planeSearchPoints: invalid

    Matcher_Adaptive m;
    bool             didThrow = false;
    try
    {
        m.initialize(p);
    }
    catch (const std::exception&)
    {
        didThrow = true;
    }
    ASSERT_(didThrow);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_basic_pt2pt_matching();
        std::cout << "test_basic_pt2pt_matching: Success ✅" << std::endl;

        test_disjoint_clouds_produce_no_pairings();
        std::cout << "test_disjoint_clouds_produce_no_pairings: Success ✅" << std::endl;

        test_overlapping_boxes_but_no_close_neighbors_does_not_crash();
        std::cout << "test_overlapping_boxes_but_no_close_neighbors_does_not_crash: Success ✅"
                  << std::endl;

        test_single_local_point_does_not_throw_on_degenerate_histogram_range();
        std::cout
            << "test_single_local_point_does_not_throw_on_degenerate_histogram_range: Success ✅"
            << std::endl;

        test_plane_detection();
        std::cout << "test_plane_detection: Success ✅" << std::endl;

        test_invalid_confidence_interval_throws();
        std::cout << "test_invalid_confidence_interval_throws: Success ✅" << std::endl;

        test_inconsistent_plane_points_throws();
        std::cout << "test_inconsistent_plane_points_throws: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
