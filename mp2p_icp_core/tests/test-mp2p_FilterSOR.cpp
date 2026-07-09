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
 * @file   test-mp2p_FilterSOR.cpp
 * @brief  Unit tests for FilterSOR
 * @author Jose Luis Blanco Claraco
 * @date   Dec 28, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterSOR.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/version.h>

#include <iostream>

using namespace mp2p_icp_filters;

namespace
{
#if MRPT_VERSION >= 0x020f04  // 2.15.4
void Test_SOR()
{
    // 1. Create a test point cloud
    auto pc = mrpt::maps::CSimplePointsMap::Create();

    // Create a dense cluster of 100 points around the origin (Inliers)
    // These will have very small average distances to neighbors
    for (int i = 0; i < 100; ++i)
    {
        pc->insertPoint(
            0.1f * static_cast<float>(i % 10), 0.1f * static_cast<float>(i) / 10.0f, 0.0f);
    }

    // Add one isolated point very far away (Outlier)
    // This will have a massive average distance compared to the cluster
    pc->insertPoint(100.0f, 100.0f, 100.0f);

    // 2. Setup Filter
    FilterSOR              filter;
    mrpt::containers::yaml p;
    p["mean_k"]                 = 10;
    p["std_dev_mul"]            = 1.0;
    p["input_pointcloud_layer"] = "raw";
    p["output_layer_inliers"]   = "inliers";
    p["output_layer_outliers"]  = "outliers";

    filter.initialize(p);

    // 3. Execute Filter
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    filter.filter(map);

    // 4. Verify Results using MRPT Asserts
    auto inliers  = map.layer<mrpt::maps::CPointsMap>("inliers");
    auto outliers = map.layer<mrpt::maps::CPointsMap>("outliers");

    ASSERT_(inliers != nullptr);
    ASSERT_(outliers != nullptr);

    // Check counts
    ASSERT_EQUAL_(inliers->size(), 100ULL);
    ASSERT_EQUAL_(outliers->size(), 1ULL);

    // Verify the outlier coordinate is actually the one in the outlier layer
    float ox, oy, oz;
    outliers->getPoint(0, ox, oy, oz);
    ASSERT_EQUAL_(ox, 100.0f);
}
#endif
}  // namespace

int main()
{
    try
    {
#if MRPT_VERSION >= 0x020f04  // 2.15.4
        Test_SOR();
        std::cout << "FilterSOR Unit Test Passed! ✅\n";
#else
        std::cout << "DISABLED TEST: Requires MRPT>=2.15.4" << std::endl;
#endif
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}