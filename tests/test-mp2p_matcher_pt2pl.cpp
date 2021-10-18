/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_matcher_p2pl.cpp
 * @brief  Unit tests for matcher
 * @author Jose Luis Blanco Claraco
 * @date   July 22, 2020
 */

#include <mp2p_icp/Matcher_Point2Plane.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>

static mrpt::maps::CSimplePointsMap::Ptr generateGlobalPoints()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    // Plane:
    for (int ix = 0; ix < 10; ix++)
        for (int iy = 0; iy < 10; iy++)
            pts->insertPoint(ix * 0.01f, 5.0f + iy * 0.01f, .0f);

    // Plane:
    for (int iy = 0; iy < 10; iy++)
        for (int iz = 0; iz < 10; iz++)
            pts->insertPoint(10.0f, iy * 0.01f, iz * 0.01f);

    // Not a plane:
    for (int ix = 0; ix < 10; ix++)
        for (int iy = 0; iy < 10; iy++)
            for (int iz = 0; iz < 10; iz++)
                pts->insertPoint(20.0f + ix * 0.01f, iy * 0.01f, iz * 0.01f);

    return pts;
}

static mrpt::maps::CSimplePointsMap::Ptr generateLocalPoints()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    pts->insertPointFast(0.f, 0.f, 0.f);
    pts->insertPointFast(2.f, 0.f, 0.f);

    return pts;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        mp2p_icp::metric_map_t pcGlobal;
        pcGlobal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] =
            generateGlobalPoints();

        mp2p_icp::metric_map_t pcLocal;
        pcLocal.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] =
            generateLocalPoints();

        {
            auto m = mp2p_icp::Matcher_Point2Plane::Create();

            mrpt::containers::yaml p;
            p["distanceThreshold"]   = 0.1;
            p["knn"]                 = 5;
            p["planeEigenThreshold"] = 0.1;

            m->initialize(p);

            {
                // For pose: identity
                mp2p_icp::Pairings   pairs;
                mp2p_icp::MatchState ms(pcGlobal, pcLocal);
                m->match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, ms, pairs);
                ASSERT_(pairs.empty());
            }

            {
                // For pose #1
                mp2p_icp::Pairings   pairs;
                mp2p_icp::MatchState ms(pcGlobal, pcLocal);
                m->match(pcGlobal, pcLocal, {0, 5, 0, 0, 0, 0}, {}, ms, pairs);
                ASSERT_EQUAL_(pairs.size(), 1U);
                ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 1U);
            }

            {
                // For pose #2
                mp2p_icp::Pairings   pairs;
                mp2p_icp::MatchState ms(pcGlobal, pcLocal);
                m->match(
                    pcGlobal, pcLocal, {8.04, 0, 0.0, 0, 0, 0}, {}, ms, pairs);
                ASSERT_EQUAL_(pairs.size(), 1U);
                ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 1U);

                const auto& p0 = pairs.paired_pt2pl.at(0);

                ASSERT_NEAR_(p0.pt_other.x, 2.0, 1e-3);
                ASSERT_NEAR_(p0.pt_other.y, 0.0, 1e-3);
                ASSERT_NEAR_(p0.pt_other.z, 0.0, 1e-3);

                ASSERT_NEAR_(p0.pl_this.centroid.x, 10.0, 0.01);
                ASSERT_NEAR_(p0.pl_this.centroid.y, 0.0, 0.01);
                ASSERT_NEAR_(p0.pl_this.centroid.z, 0.0, 0.01);

                // Plane equation: "x=10"  (Ax+By+Cz+D=0)
                ASSERT_NEAR_(p0.pl_this.plane.coefs[0], 1.0, 1e-3);
                ASSERT_NEAR_(p0.pl_this.plane.coefs[1], 0.0, 1e-3);
                ASSERT_NEAR_(p0.pl_this.plane.coefs[2], 0.0, 1e-3);
                ASSERT_NEAR_(p0.pl_this.plane.coefs[3], -10.0, 1e-3);
            }

            {
                // For pose #3
                mp2p_icp::Pairings   pairs;
                mp2p_icp::MatchState ms(pcGlobal, pcLocal);
                m->match(
                    pcGlobal, pcLocal, {18.053, 0.05, 0.03, 0, 0, 0}, {}, ms,
                    pairs);
                ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 0U);
            }

            {
                // For pose #2 bis, NOT avoiding duplicated matches with another
                // pt-to-pt matcher:
                auto mPt2Pt =
                    mp2p_icp::Matcher_Points_DistanceThreshold::Create();
                mrpt::containers::yaml p2;
                p2["threshold"]                      = 0.1;
                p2["allowMatchAlreadyMatchedPoints"] = true;
                mPt2Pt->initialize(p2);

                const mp2p_icp::Pairings pairs = mp2p_icp::run_matchers(
                    {m, mPt2Pt}, pcGlobal, pcLocal, {8.04, 0, 0.0, 0, 0, 0},
                    {});

                // std::cout << pairs.contents_summary() << std::endl;
                ASSERT_EQUAL_(pairs.size(), 2U);
                ASSERT_EQUAL_(pairs.paired_pt2pt.size(), 1U);
                ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 1U);
            }

            {
                // For pose #2 tris, DO avoid duplicated matches with another
                // pt-to-pt matcher:
                auto mPt2Pt =
                    mp2p_icp::Matcher_Points_DistanceThreshold::Create();
                mrpt::containers::yaml p2;
                p2["threshold"]                      = 0.1;
                p2["allowMatchAlreadyMatchedPoints"] = false;
                mPt2Pt->initialize(p2);

                const mp2p_icp::Pairings pairs = mp2p_icp::run_matchers(
                    {m, mPt2Pt}, pcGlobal, pcLocal, {8.04, 0, 0.0, 0, 0, 0},
                    {});

                // std::cout << pairs.contents_summary() << std::endl;
                ASSERT_EQUAL_(pairs.size(), 1U);
                ASSERT_EQUAL_(pairs.paired_pt2pl.size(), 1U);
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
