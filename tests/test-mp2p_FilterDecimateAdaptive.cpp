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
 * @file   test_FilterDecimateAdaptive.cpp
 * @brief  Unit test for FilterDecimateAdaptive
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        // 1. Setup a dense cloud with 10,000 points in a 10x10x1 volume
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        for (int i = 0; i < 10000; ++i)
        {
            pc->insertPoint(
                static_cast<float>(i % 100) * 0.1f,  // X: 0 to 10
                static_cast<float>((i / 100) % 100) * 0.1f,  // Y: 0 to 10
                0.0f  // Z: Flat
            );
        }

        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;

        // ---------------------------------------------------------
        // Test: Reduce 10k points to approx 500
        // ---------------------------------------------------------
        {
            FilterDecimateAdaptive filter;
            mrpt::containers::yaml p;
            p["input_pointcloud_layer"]         = "raw";
            p["output_pointcloud_layer"]        = "adaptive_out";
            p["desired_output_point_count"]     = 500;
            p["voxel_size"]                     = 0.5f;  // Large enough voxels to group points
            p["minimum_input_points_per_voxel"] = 1;

            filter.initialize(p);
            filter.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("adaptive_out");
            ASSERT_(out);

            const size_t outSize = out->size();
            std::cout << "Adaptive Decimation: Input=" << pc->size()
                      << " Desired=500 Output=" << outSize << "\n";

            // The adaptive filter tries to hit the target but depends on voxelization.
            // It should be reasonably close to the target if the voxel grid allows it.
            // Since we have plenty of points and voxels, it should stop exactly at or very close to
            // 500.
            ASSERT_NEAR_(outSize, 500, 20);  // Allow small tolerance

            std::cout << "[Test Passed] Adaptive decimation count check\n";
        }

        std::cout << "\nFilterDecimateAdaptive Unit Tests Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}