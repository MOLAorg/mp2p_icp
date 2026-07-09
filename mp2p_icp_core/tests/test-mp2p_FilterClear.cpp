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
 * @file   test-mp2p_FilterClear.cpp
 * @brief  Unit test for FilterClear
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterClear.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

void test_clear_layer()
{
    // ---------------------------------------------------------
    // Case 1: Clear a layer with points
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CSimplePointsMap::Create();

        // Add some points
        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPoint(4.0f, 5.0f, 6.0f);
        pc->insertPoint(7.0f, 8.0f, 9.0f);
        ASSERT_EQUAL_(pc->size(), 3U);

        map.layers["observation"] = pc;

        FilterClear            filter;
        mrpt::containers::yaml p;
        p["target_layer"] = "observation";
        filter.initialize(p);

        filter.filter(map);

        // Layer should still exist but be empty
        ASSERT_EQUAL_(map.layers.count("observation"), 1ULL);
        ASSERT_(map.layers["observation"] == pc);
        ASSERT_EQUAL_(pc->size(), 0U);
        std::cout << "[Test Passed] Clear layer with points" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 2: Clear an already empty layer
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CSimplePointsMap::Create();
        ASSERT_EQUAL_(pc->size(), 0U);

        map.layers["empty_layer"] = pc;

        FilterClear            filter;
        mrpt::containers::yaml p;
        p["target_layer"] = "empty_layer";
        filter.initialize(p);

        filter.filter(map);

        // Layer should still exist and still be empty
        ASSERT_EQUAL_(map.layers.count("empty_layer"), 1ULL);
        ASSERT_(map.layers["empty_layer"] == pc);
        ASSERT_EQUAL_(pc->size(), 0U);
        std::cout << "[Test Passed] Clear already empty layer" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 3: Try to clear non-existent layer (should throw)
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;

        FilterClear            filter;
        mrpt::containers::yaml p;
        p["target_layer"] = "non_existent";
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
        std::cout << "[Test Passed] Non-existent layer throws exception" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 4: Clear layer, verify it can be refilled
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CSimplePointsMap::Create();

        // Add initial points
        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPoint(4.0f, 5.0f, 6.0f);
        ASSERT_EQUAL_(pc->size(), 2U);

        map.layers["refillable"] = pc;

        FilterClear            filter;
        mrpt::containers::yaml p;
        p["target_layer"] = "refillable";
        filter.initialize(p);

        filter.filter(map);
        ASSERT_EQUAL_(pc->size(), 0U);

        // Refill with new points
        pc->insertPoint(10.0f, 20.0f, 30.0f);
        ASSERT_EQUAL_(pc->size(), 1U);

        std::cout << "[Test Passed] Layer can be refilled after clearing" << std::endl;
    }
}

int main()
{
    try
    {
        test_clear_layer();
        std::cout << "\nAll FilterClear tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
