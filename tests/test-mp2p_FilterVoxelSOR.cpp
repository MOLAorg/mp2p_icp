/*
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
 * @file   test-mp2p_FilterVoxelSOR.cpp
 * @brief  Unit tests for FilterVoxelSOR
 * @author Jose Luis Blanco Claraco
 * @date   Jan 21, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterVoxelSOR.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        // 1. Setup cloud: A dense cluster and one distant outlier
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        // Dense cluster at (0,0,0)
        for (float x = -0.1f; x <= 0.1f; x += 0.02f)
        {
            for (float y = -0.1f; y <= 0.1f; y += 0.02f)
            {
                pc->insertPoint(x, y, 0.0f);
            }
        }

        const size_t clusterSize = pc->size();
        // Add one single point very far away
        pc->insertPoint(10.0f, 10.0f, 10.0f);

        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;

        // 2. Configure Filter
        FilterVoxelSOR         filter;
        mrpt::containers::yaml p;
        p["input_layer"]           = "raw";
        p["output_layer_inliers"]  = "clean";
        p["output_layer_outliers"] = "noise";
        p["voxel_size"]            = 15.0f;  // Large enough to contain both in the same voxel
        p["mean_k"]                = 10;
        p["std_dev_mul"]           = 1.0;
        filter.initialize(p);

        // 3. Run Filter
        filter.filter(map);

        // 4. Verify
        auto inliers  = map.layer<mrpt::maps::CPointsMap>("clean");
        auto outliers = map.layer<mrpt::maps::CPointsMap>("noise");

        std::cout << "Original points: " << clusterSize + 1 << "\n";
        std::cout << "Inliers: " << inliers->size() << "\n";
        std::cout << "Outliers: " << outliers->size() << "\n";

        // The far point should be an outlier
        ASSERT_(outliers->size() >= 1);
        ASSERT_(inliers->size() <= clusterSize);

        std::cout << "FilterVoxelSOR Unit Test Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}