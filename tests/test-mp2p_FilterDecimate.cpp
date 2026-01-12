/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \\ / _ \\| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\\___/|_|\\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   test_FilterDecimate.cpp
 * @brief  Unit test for FilterDecimate
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 12, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDecimate.h>
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
        // Case A: Fixed Decimation (1 out of 4 -> 25 points)
        // ---------------------------------------------------------
        {
            FilterDecimate         filter;
            mrpt::containers::yaml p;
            p["input_layer"]  = "raw";
            p["output_layer"] = "decimated_fixed";
            p["decimation"]   = 4;
            filter.initialize(p);

            filter.filter(map);
            auto out = map.layer<mrpt::maps::CPointsMap>("decimated_fixed");
            ASSERT_EQUAL_(out->size(), 25ULL);
            std::cout << "[Test Passed] Fixed decimation factor of 4" << std::endl;
        }

        // ---------------------------------------------------------
        // Case B: Target Maximum Size (Target 10 -> factor 10 -> 10 points)
        // ---------------------------------------------------------
        {
            FilterDecimate         filter;
            mrpt::containers::yaml p;
            p["input_layer"]     = "raw";
            p["output_layer"]    = "decimated_target";
            p["target_max_size"] = 10;
            filter.initialize(p);

            filter.filter(map);
            auto out = map.layer<mrpt::maps::CPointsMap>("decimated_target");
            // 100 / 10 = 10 factor. 100 / 10 points.
            ASSERT_EQUAL_(out->size(), 10ULL);
            std::cout << "[Test Passed] Target maximum size of 10" << std::endl;
        }

        // ---------------------------------------------------------
        // Case C: Target size larger than input (No-op)
        // ---------------------------------------------------------
        {
            FilterDecimate         filter;
            mrpt::containers::yaml p;
            p["input_layer"]     = "raw";
            p["output_layer"]    = "no_decimation";
            p["target_max_size"] = 500;
            filter.initialize(p);

            filter.filter(map);
            auto out = map.layer<mrpt::maps::CPointsMap>("no_decimation");
            ASSERT_EQUAL_(out->size(), 100ULL);
            std::cout << "[Test Passed] Target size larger than input" << std::endl;
        }

        std::cout << "\nFilterDecimate Unit Tests Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}