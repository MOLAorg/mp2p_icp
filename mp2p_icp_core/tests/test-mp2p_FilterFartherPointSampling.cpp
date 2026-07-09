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
 * @file   test_FilterFartherPointSampling.cpp
 * @brief  Unit test for FilterFartherPointSampling
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterFartherPointSampling.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        // 1. Setup cloud with 100 points
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        for (int i = 0; i < 100; ++i)
        {
            pc->insertPoint(static_cast<float>(i), 0, 0);
        }

        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;

        // ---------------------------------------------------------
        // Case A: Subsample to 10 points
        // ---------------------------------------------------------
        {
            FilterFartherPointSampling filter;
            mrpt::containers::yaml     p;
            p["input_pointcloud_layer"]     = "raw";
            p["output_pointcloud_layer"]    = "fps_out";
            p["desired_output_point_count"] = 10;

            filter.initialize(p);
            filter.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("fps_out");
            ASSERT_(out);

            std::cout << "FPS Subsample: Input=" << pc->size()
                      << " Desired=10 Output=" << out->size() << "\n";

            ASSERT_EQUAL_(out->size(), 10ULL);

            // FPS property check: The points should cover the range well.
            // With 100 points from 0 to 99, picking 10 should likely include ends.
            // (Strict geometric check is complex due to random seed, but size check is robust).
            std::cout << "[Test Passed] FPS count exact match\n";
        }

        // ---------------------------------------------------------
        // Case B: Request more points than available
        // ---------------------------------------------------------
        {
            FilterFartherPointSampling filter;
            mrpt::containers::yaml     p;
            p["input_pointcloud_layer"]     = "raw";
            p["output_pointcloud_layer"]    = "fps_copy";
            p["desired_output_point_count"] = 200;  // Request > 100

            filter.initialize(p);
            filter.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("fps_copy");
            ASSERT_(out);

            std::cout << "FPS Oversample: Input=" << pc->size()
                      << " Desired=200 Output=" << out->size() << "\n";

            // Should just copy the input
            ASSERT_EQUAL_(out->size(), 100ULL);
            std::cout << "[Test Passed] FPS oversample handling\n";
        }

        std::cout << "\nFilterFartherPointSampling Unit Tests Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}