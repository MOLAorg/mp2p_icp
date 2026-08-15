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
 * @file   test-mp2p_matcher_ndt_blend.cpp
 * @brief  Unit tests for Matcher_NDT_Blend
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_NDT_Blend.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
/** Minimal stand-in for a cell-based plane map.
 *
 * Deriving from CSimplePointsMap gets the whole CMetricMap interface for free;
 * the plane models are held separately so the scene is stated exactly rather
 * than fitted.
 */
class TestPlaneMap : public mrpt::maps::CSimplePointsMap, public mp2p_icp::NearestPlaneCapable
{
   public:
    struct Cell
    {
        mrpt::math::TPoint3D  centroid;
        mrpt::math::TVector3D normal;
    };

    std::vector<Cell> cells;

    void addCell(const mrpt::math::TPoint3D& c, const mrpt::math::TVector3D& n)
    {
        cells.push_back({c, n});
        // Also insert it as a point, so the map is non-empty and its bounding
        // box covers the scene:
        this->insertPointFast(
            static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
    }

    NearestPlaneResult nn_search_pt2pl(
        const mrpt::math::TPoint3Df& query, const float max_search_distance) const override
    {
        NearestPlaneResult ret;
        visitAll(
            query, max_search_distance,
            [&](const PlaneCandidate& c)
            {
                if (!ret.pairing || c.distance < ret.distance)
                {
                    ret.pairing  = c.pairing;
                    ret.distance = c.distance;
                }
            });
        return ret;
    }

    void nn_visit_pt2pl_candidates(
        const mrpt::math::TPoint3Df& query, const float max_search_distance,
        const plane_candidate_visitor_t& visitor) const override
    {
        visitAll(query, max_search_distance, visitor);
    }

   private:
    void visitAll(
        const mrpt::math::TPoint3Df&     query, const float /*max_search_distance*/,
        const plane_candidate_visitor_t& visitor) const
    {
        // Every cell is a candidate: the matcher is what applies the window,
        // and this keeps the test independent of any cell-indexing scheme.
        for (const auto& cell : cells)
        {
            const auto thePlane =
                mrpt::math::TPlane::FromPointAndNormal(cell.centroid, cell.normal);

            const mrpt::math::TPoint3D q(query.x, query.y, query.z);

            PlaneCandidate c;
            c.pairing.pt_local           = query;
            c.pairing.pl_global.centroid = cell.centroid;
            c.pairing.pl_global.plane    = thePlane;
            c.distance                   = static_cast<float>(thePlane.distance(q));
            c.centroidDistance           = static_cast<float>((cell.centroid - q).norm());

            visitor(c);
        }
    }
};

/// Runs the matcher for one query point and returns the plane it paired against
std::optional<mrpt::math::TPlane> planeFor(
    mp2p_icp::Matcher& m, const std::shared_ptr<TestPlaneMap>& global, const double qx,
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

    if (pairs.paired_pt2pl.empty())
    {
        return {};
    }
    return pairs.paired_pt2pl.at(0).pl_global.plane;
}

/// Angular disagreement between two planes' normals, ignoring their signs.
/// The sign is arbitrary and the point-to-plane cost is invariant to it.
double normalMismatch(const mrpt::math::TPlane& a, const mrpt::math::TPlane& b)
{
    const double na =
        std::sqrt(a.coefs[0] * a.coefs[0] + a.coefs[1] * a.coefs[1] + a.coefs[2] * a.coefs[2]);
    const double nb =
        std::sqrt(b.coefs[0] * b.coefs[0] + b.coefs[1] * b.coefs[1] + b.coefs[2] * b.coefs[2]);

    const double dot =
        (a.coefs[0] * b.coefs[0] + a.coefs[1] * b.coefs[1] + a.coefs[2] * b.coefs[2]) / (na * nb);

    return 1.0 - std::abs(dot);
}

std::shared_ptr<TestPlaneMap> twoCompetingPlanes()
{
    auto m = std::make_shared<TestPlaneMap>();
    // A floor (z=0) and a steeply tilted surface through (1,0,0), 65 deg apart.
    // A query gliding along +x at z=0.05 is nearer the floor at both ends of
    // the sweep and nearer the tilted surface in between, so the closest of the
    // two swaps twice along the way.
    m->addCell({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    m->addCell({1.0, 0.0, 0.0}, {std::cos(0.4363), 0.0, std::sin(0.4363)});
    return m;
}

std::shared_ptr<TestPlaneMap> orthogonalCorner()
{
    auto m = std::make_shared<TestPlaneMap>();
    // A floor and a wall at exactly 90 deg: the one configuration in which the
    // blend cannot interpolate. See test_orthogonal_corner_is_the_known_limit.
    m->addCell({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    m->addCell({1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    return m;
}

mp2p_icp::Matcher_NDT_Blend::Ptr makeMatcher(
    const double temperature, const double searchRadius, const bool smoothCutoff)
{
    auto m = mp2p_icp::Matcher_NDT_Blend::Create();

    mrpt::containers::yaml p;
    p["distanceThreshold"] = 10.0;
    p["temperature"]       = temperature;
    p["searchRadius"]      = searchRadius;
    p["smoothCutoff"]      = smoothCutoff;
    m->initialize(p);

    return m;
}

/// Largest normal jump between consecutive samples of a straight sweep in x.
double maxNormalStepAlongX(
    mp2p_icp::Matcher& m, const std::shared_ptr<TestPlaneMap>& global, const double x0,
    const double x1, const int nSteps, const double qz)
{
    double                            worst = 0;
    std::optional<mrpt::math::TPlane> prev;

    for (int i = 0; i <= nSteps; i++)
    {
        const double x  = x0 + (x1 - x0) * i / static_cast<double>(nSteps);
        const auto   pl = planeFor(m, global, x, 0.0, qz);
        if (pl && prev)
        {
            worst = std::max(worst, normalMismatch(*prev, *pl));
        }
        prev = pl;
    }
    return worst;
}

void test_zero_temperature_is_the_argmin()
{
    auto global = twoCompetingPlanes();
    auto m      = makeMatcher(0.0, 3.0, true);

    for (double x = 0.5; x <= 1.5; x += 0.1)
    {
        const auto got = planeFor(*m, global, x, 0.0, 0.05);
        ASSERT_(got.has_value());

        // Reference: the map's own argmin:
        const auto ref = global->nn_search_pt2pl({static_cast<float>(x), 0.f, 0.05f}, 10.f);
        ASSERT_(ref.pairing.has_value());

        for (int k = 0; k < 4; k++)
        {
            ASSERT_NEAR_(got->coefs[k], ref.pairing->pl_global.plane.coefs[k], 1e-12);
        }
    }
    std::cout << "test_zero_temperature_is_the_argmin: PASSED\n";
}

void test_low_temperature_tends_to_the_argmin()
{
    auto global = twoCompetingPlanes();
    auto m      = makeMatcher(1e-4, 3.0, true);

    for (double x = 0.5; x <= 1.5; x += 0.1)
    {
        const auto got = planeFor(*m, global, x, 0.0, 0.05);
        ASSERT_(got.has_value());

        const auto ref = global->nn_search_pt2pl({static_cast<float>(x), 0.f, 0.05f}, 10.f);
        ASSERT_(ref.pairing.has_value());

        // Only the direction is compared: the blended plane's centroid is a
        // weighted mean, so it need not coincide with the winner's.
        ASSERT_LT_(normalMismatch(*got, ref.pairing->pl_global.plane), 1e-6);
    }
    std::cout << "test_low_temperature_tends_to_the_argmin: PASSED\n";
}

void test_blending_removes_the_correspondence_jump()
{
    auto global = twoCompetingPlanes();

    auto hard = makeMatcher(0.0, 3.0, true);
    auto soft = makeMatcher(0.3, 3.0, true);

    const double jumpHard = maxNormalStepAlongX(*hard, global, 0.5, 1.5, 400, 0.05);
    const double jumpSoft = maxNormalStepAlongX(*soft, global, 0.5, 1.5, 400, 0.05);

    std::cout << "correspondence switch: argmin step=" << jumpHard << " blended step=" << jumpSoft
              << "\n";

    // The argmin swaps the target plane outright somewhere along the sweep:
    ASSERT_GT_(jumpHard, 0.1);
    // The blend rotates the target plane continuously instead:
    ASSERT_LT_(jumpSoft, 1e-3);

    std::cout << "test_blending_removes_the_correspondence_jump: PASSED\n";
}

void test_smooth_cutoff_removes_the_window_edge_jump()
{
    // One plane always in range, plus a strongly disagreeing one whose centroid
    // crosses the search radius during the sweep:
    auto global = std::make_shared<TestPlaneMap>();
    global->addCell({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    global->addCell({2.0, 0.0, 0.0}, {std::sin(0.5236), 0.0, std::cos(0.5236)});

    const double R = 1.5;

    auto hardEdge   = makeMatcher(1.0, R, false);
    auto smoothEdge = makeMatcher(1.0, R, true);

    // As x grows the second cell's centroid distance falls through R=1.5:
    const double jumpHard   = maxNormalStepAlongX(*hardEdge, global, 0.0, 1.0, 400, 0.05);
    const double jumpSmooth = maxNormalStepAlongX(*smoothEdge, global, 0.0, 1.0, 400, 0.05);

    std::cout << "window edge: hard step=" << jumpHard << " smooth step=" << jumpSmooth << "\n";

    ASSERT_GT_(jumpHard, 1e-3);
    ASSERT_LT_(jumpSmooth, 1e-4);

    std::cout << "test_smooth_cutoff_removes_the_window_edge_jump: PASSED\n";
}

void test_orthogonal_corner_is_the_known_limit()
{
    // Documented limitation, asserted so that it stays documented.
    //
    // Normals are combined through their outer products, which is what keeps
    // the result independent of each candidate's arbitrary sign. The price is
    // that when two candidates are EXACTLY perpendicular and carry EXACTLY
    // equal weight, the two leading eigenvalues coincide and the blended
    // direction switches instead of rotating. Any angle other than 90 degrees
    // keeps the eigenvalues apart and the blend smooth, which is why the
    // sweep above is continuous; and no single plane can describe a right
    // angle anyway, so this is a property of summarizing a corner by one
    // plane, not of the weighting.
    auto global = orthogonalCorner();
    auto soft   = makeMatcher(0.3, 3.0, true);

    const double jump = maxNormalStepAlongX(*soft, global, 0.5, 1.5, 400, 0.05);
    std::cout << "orthogonal corner: blended step=" << jump << "\n";

    ASSERT_GT_(jump, 0.5);

    std::cout << "test_orthogonal_corner_is_the_known_limit: PASSED\n";
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_zero_temperature_is_the_argmin();
        test_low_temperature_tends_to_the_argmin();
        test_blending_removes_the_correspondence_jump();
        test_smooth_cutoff_removes_the_window_edge_jump();
        test_orthogonal_corner_is_the_known_limit();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
