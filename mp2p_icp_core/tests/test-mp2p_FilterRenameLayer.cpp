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
 * @file   test-mp2p_FilterRenameLayer.cpp
 * @brief  Unit test for FilterRenameLayer
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 7, 2026
 */
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterRenameLayer.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

void test_rename_logic()
{
    // ---------------------------------------------------------
    // Case 1: Standard Rename Success
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CSimplePointsMap::Create();
        map.layers["raw_layer"]   = pc;

        FilterRenameLayer      filter;
        mrpt::containers::yaml p;
        p["input_layer"]  = "raw_layer";
        p["output_layer"] = "processed_layer";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_(map.layers.count("raw_layer") == 0);
        ASSERT_EQUAL_(map.layers.count("processed_layer"), 1ULL);
        ASSERT_(map.layers["processed_layer"] == pc);
        std::cout << "[Test Passed] Standard rename" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 2: Input == Output (No-op)
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CSimplePointsMap::Create();
        map.layers["stable"]      = pc;

        FilterRenameLayer      filter;
        mrpt::containers::yaml p;
        p["input_layer"]  = "stable";
        p["output_layer"] = "stable";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_EQUAL_(map.layers.count("stable"), 1ULL);
        ASSERT_(map.layers["stable"] == pc);
        std::cout << "[Test Passed] Identity rename (no-op)" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 3: Missing Input - Silent (default)
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;

        FilterRenameLayer      filter;
        mrpt::containers::yaml p;
        p["input_layer"]                        = "non_existent";
        p["output_layer"]                       = "whatever";
        p["fail_if_input_layer_does_not_exist"] = false;
        filter.initialize(p);

        // Should not throw
        filter.filter(map);
        ASSERT_(map.layers.empty());
        std::cout << "[Test Passed] Missing layer - Silent ignore" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 4: Missing Input - Failure Requested
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;

        FilterRenameLayer      filter;
        mrpt::containers::yaml p;
        p["input_layer"]                        = "non_existent";
        p["output_layer"]                       = "whatever";
        p["fail_if_input_layer_does_not_exist"] = true;
        filter.initialize(p);

        bool caught = false;
        try
        {
            filter.filter(map);
        }
        catch (const std::exception& e)
        {
            caught = true;
            std::cout << "Caught expected error: " << e.what() << std::endl;
        }
        ASSERT_(caught);
        std::cout << "[Test Passed] Missing layer - Exception thrown" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 5: Overwriting an existing layer
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc1 = mrpt::maps::CSimplePointsMap::Create();
        auto                   pc2 = mrpt::maps::CSimplePointsMap::Create();
        map.layers["source"]       = pc1;
        map.layers["target"]       = pc2;

        FilterRenameLayer      filter;
        mrpt::containers::yaml p;
        p["input_layer"]  = "source";
        p["output_layer"] = "target";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_(map.layers.count("source") == 0);
        ASSERT_EQUAL_(map.layers.count("target"), 1ULL);
        // Verify target now points to pc1, and pc2 was released
        ASSERT_(map.layers["target"] == pc1);
        std::cout << "[Test Passed] Target overwrite" << std::endl;
    }
}

int main()
{
    try
    {
        test_rename_logic();
        std::cout << "\nAll FilterRenameLayer coverage tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}