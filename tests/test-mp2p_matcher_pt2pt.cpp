/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_matcher_p2pt.cpp
 * @brief  Unit tests for matcher
 * @author Jose Luis Blanco Claraco
 * @date   July 22, 2020
 */

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/Matcher_Points_InlierRatio.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/maps/CSimplePointsMap.h>

static mrpt::maps::CSimplePointsMap::Ptr generateGlobalPoints()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    for (int i = 0; i < 10; i++) pts->insertPoint(i * 0.01f, 5.0f, .0f);
    for (int i = 0; i < 10; i++) pts->insertPoint(10.0f, i * 0.01f, 1.0f);

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
        mp2p_icp::pointcloud_t pcGlobal;
        pcGlobal.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] =
            generateGlobalPoints();

        mp2p_icp::pointcloud_t pcLocal;
        pcLocal.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] =
            generateLocalPoints();

        {
            mp2p_icp::Matcher_Points_DistanceThreshold m;
            mrpt::containers::yaml               p;
            p["threshold"] = 1.0;

            m.initialize(p);

            {
                // For pose: identity
                mp2p_icp::Pairings pairs;
                m.match(pcGlobal, pcLocal, {0, 0, 0, 0, 0, 0}, {}, pairs);
                ASSERT_(pairs.empty());
            }

            // note: on MRPT naming convention: "this"=global; "other"=local.

            {
                // For pose #1
                mp2p_icp::Pairings pairs;
                m.match(pcGlobal, pcLocal, {0, 5, 0, 0, 0, 0}, {}, pairs);
                ASSERT_EQUAL_(pairs.size(), 1);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).other_idx, 0);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).this_idx, 0);
            }

            {
                // For pose #2
                mp2p_icp::Pairings pairs;
                m.match(pcGlobal, pcLocal, {-2, 5, 0, 0, 0, 0}, {}, pairs);
                ASSERT_EQUAL_(pairs.size(), 1);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).this_idx, 0);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).other_idx, 1);
            }

            {
                // For pose #3
                mp2p_icp::Pairings pairs;
                m.match(
                    pcGlobal, pcLocal,
                    {8.5, -1.0, 1, mrpt::DEG2RAD(45.0f), 0, 0}, {}, pairs);
                ASSERT_EQUAL_(pairs.size(), 1);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).other_idx, 1);
                ASSERT_EQUAL_(pairs.paired_pt2pt.at(0).this_idx, 19);
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
