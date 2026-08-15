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
 * @file   test-mp2p_matcher_knn_plane.cpp
 * @brief  Unit tests for Matcher_Points_KnnPlane
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_Points_KnnPlane.h>
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

/// Five points spanning a patch of the plane z = z0, the last one optionally
/// lifted off it to exercise the planarity gate.
///
/// Note z0 must not be 0. The `A x = -1` form cannot represent a plane through
/// the origin at all -- its offset is 1/|x|, which diverges -- so a test patch
/// on z = 0 would exercise a degeneracy rather than the protocol. Real map
/// points are in the world frame, where a plane through the origin is a
/// measure-zero accident, and the reference implementation has the same
/// property.
std::vector<mrpt::math::TPoint3D> patch(const double z0, const double lift = 0)
{
    // The lifted point is the CENTER one, deliberately. Lifting an edge point
    // instead lets the least-squares plane tilt to absorb most of the offset,
    // so a 0.25 m displacement shows up as only ~0.06 m of deviation and the
    // planarity gate never fires; a bump at the centroid cannot be absorbed
    // that way.
    return {
        {0.0, 0.0, z0 + lift}, {0.2, 0.0, z0}, {0.0, 0.2, z0}, {-0.2, 0.0, z0}, {0.0, -0.2, z0},
    };
}

std::optional<mp2p_icp::point_plane_pair_t> pairFor(
    mp2p_icp::Matcher& m, const mrpt::maps::CSimplePointsMap::Ptr& global,
    const mrpt::math::TPoint3D& q)
{
    mp2p_icp::metric_map_t pcGlobal;
    pcGlobal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = global;

    auto localPts = mrpt::maps::CSimplePointsMap::Create();
    localPts->insertPointFast(
        static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z));

    mp2p_icp::metric_map_t pcLocal;
    pcLocal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = localPts;

    mp2p_icp::Pairings   pairs;
    mp2p_icp::MatchState ms(pcGlobal, pcLocal);
    m.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);

    if (pairs.paired_pt2pl.empty())
    {
        return {};
    }
    return pairs.paired_pt2pl.at(0);
}

mp2p_icp::Matcher_Points_KnnPlane::Ptr makeMatcher(
    const mrpt::containers::yaml& overrides = mrpt::containers::yaml::Map())
{
    auto m = mp2p_icp::Matcher_Points_KnnPlane::Create();

    mrpt::containers::yaml p;
    p["allowMatchAlreadyMatchedGlobalPoints"] = true;
    if (overrides.isMap())
    {
        for (const auto& [k, v] : overrides.asMapRange())
        {
            p[k.as<std::string>()] = v;
        }
    }
    m->initialize(p);
    return m;
}

void test_defaults_are_the_reference_protocol()
{
    auto m = makeMatcher();
    // A patch on z = -1, with the query 0.02 m above it: inside every gate.
    auto       global = mapOf(patch(-1.0));
    const auto got    = pairFor(*m, global, {0.0, 0.0, -0.98});
    ASSERT_(got.has_value());

    // The fit must recover z = -1: a vertical normal, and zero distance from
    // the plane to a point known to lie on it.
    const auto& c = got->pl_global.plane.coefs;
    ASSERT_NEAR_(std::abs(c[2]) / std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]), 1.0, 1e-4);
    ASSERT_NEAR_(std::abs(got->pl_global.plane.distance({0.1, 0.1, -1.0})), 0.0, 1e-4);

    std::cout << "test_defaults_are_the_reference_protocol: PASSED\n";
}

void test_planarity_gate_rejects_a_bad_fit()
{
    // One supporting point lifted 0.25 m off the patch: beyond the 0.1 m
    // default, so the whole fit is rejected rather than merely down-weighted.
    //
    // Both arms below open the residual gate, so that the only thing differing
    // between them is the planarity threshold. Leaving it closed would confound
    // the two: a plane fitted through the lifted point is tilted, which moves
    // the residual as well.
    auto global = mapOf(patch(-1.0, 0.25));

    mrpt::containers::yaml strict;
    strict["residualGateScale"] = 0.0;
    auto m                      = makeMatcher(strict);
    ASSERT_(!pairFor(*m, global, {0.0, 0.0, -0.98}).has_value());

    // Opening the gate accepts it again, which is what ladder arm B5 does.
    mrpt::containers::yaml open;
    open["residualGateScale"]    = 0.0;
    open["planeFitMaxDeviation"] = 10.0;
    auto m2                      = makeMatcher(open);
    ASSERT_(pairFor(*m2, global, {0.0, 0.0, -0.98}).has_value());

    std::cout << "test_planarity_gate_rejects_a_bad_fit: PASSED\n";
}

void test_neighbor_distance_gate()
{
    // The patch sits 3 m below the query, so all five neighbors are beyond the
    // 2.236 m default cap.
    auto global = mapOf(patch(-3.0));
    auto m      = makeMatcher();
    ASSERT_(!pairFor(*m, global, {0.0, 0.1, 0.0}).has_value());

    mrpt::containers::yaml o;
    o["maxNeighborDistance"] = 10.0;
    o["residualGateScale"]   = 0.0;  // isolate this gate from the residual one
    auto m2                  = makeMatcher(o);
    ASSERT_(pairFor(*m2, global, {0.0, 0.1, 0.0}).has_value());

    std::cout << "test_neighbor_distance_gate: PASSED\n";
}

void test_residual_gate_is_range_scaled()
{
    // A patch on z = -0.5. A query at (0, y, -0.5 + d) is `d` off the plane and
    // roughly `y` from the sensor origin, so range and residual are set
    // independently.
    auto global = mapOf(patch(-0.5));

    // Accept iff 1 - 0.9*|d|/sqrt(range) > 0.9, i.e. |d| < sqrt(range)/9. At
    // range ~1 m that is ~0.109 m; bracket it from both sides.
    auto m = makeMatcher();
    ASSERT_(pairFor(*m, global, {0.0, 0.866, -0.42}).has_value());
    ASSERT_(!pairFor(*m, global, {0.0, 0.866, -0.34}).has_value());

    // The range scaling is what makes that threshold grow with distance. At
    // range ~8.9 m it allows |d| < 0.33 m, where the unscaled form still only
    // allows 1/9 m. Both arms open the neighbor-distance gate so that only the
    // residual gate can differ between them.
    mrpt::containers::yaml scaled;
    scaled["maxNeighborDistance"] = 20.0;
    auto mScaled                  = makeMatcher(scaled);

    mrpt::containers::yaml fixed;
    fixed["maxNeighborDistance"] = 20.0;
    fixed["rangeNormalized"]     = false;
    auto mFixed                  = makeMatcher(fixed);

    ASSERT_(pairFor(*mScaled, global, {0.0, 8.9, -0.25}).has_value());
    ASSERT_(!pairFor(*mFixed, global, {0.0, 8.9, -0.25}).has_value());

    std::cout << "test_residual_gate_is_range_scaled: PASSED\n";
}

void test_too_few_neighbors_is_rejected()
{
    // Four map points, k=5: the query cannot be paired at all.
    auto global = mapOf({{0.0, 0.0, -1.0}, {0.2, 0.0, -1.0}, {0.0, 0.2, -1.0}, {-0.2, 0.0, -1.0}});
    auto m      = makeMatcher();
    ASSERT_(!pairFor(*m, global, {0.0, 0.0, -0.98}).has_value());

    // k=3 pairs against the same map. This is ladder arm B7's mechanism.
    mrpt::containers::yaml o;
    o["knn"] = 3;
    auto m2  = makeMatcher(o);
    ASSERT_(pairFor(*m2, global, {0.0, 0.0, -0.98}).has_value());

    std::cout << "test_too_few_neighbors_is_rejected: PASSED\n";
}
}  // namespace

int main(int argc, const char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_defaults_are_the_reference_protocol();
        test_planarity_gate_rejects_a_bad_fit();
        test_neighbor_distance_gate();
        test_residual_gate_is_range_scaled();
        test_too_few_neighbors_is_rejected();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
