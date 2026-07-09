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
 * @file   test-mp2p_FilterRemovePointCloudField.cpp
 * @brief  Unit test for FilterRemovePointCloudField
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterRemovePointCloudField.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

void test_remove_field()
{
    // ---------------------------------------------------------
    // Case 1: Remove an existing float field
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        // Add some points
        // Register and populate a custom float field
        ASSERT_(pc->registerField_float("custom_field"));

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPointField_float("custom_field", 1.5f);

        pc->insertPoint(4.0f, 5.0f, 6.0f);
        pc->insertPointField_float("custom_field", 2.5f);
        ASSERT_(pc->hasPointField("custom_field"));

        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "raw";
        p["field_names"]      = "custom_field";
        filter.initialize(p);

        filter.filter(map);

        // Field should be removed
        ASSERT_(!pc->hasPointField("custom_field"));
        std::cout << "[Test Passed] Remove existing float field" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 2: Remove an existing double field
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        ASSERT_(pc->registerField_double("timestamp_abs"));

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPointField_double("timestamp_abs", 123456.789);
        ASSERT_(pc->hasPointField("timestamp_abs"));

        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "raw";
        p["field_names"]      = "timestamp_abs";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_(!pc->hasPointField("timestamp_abs"));
        std::cout << "[Test Passed] Remove existing double field" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 3: Remove an existing uint16_t field
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        ASSERT_(pc->registerField_uint16("ring"));

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPointField_uint16("ring", 42);
        ASSERT_(pc->hasPointField("ring"));

        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "raw";
        p["field_names"]      = "ring";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_(!pc->hasPointField("ring"));
        std::cout << "[Test Passed] Remove existing uint16_t field" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 4: Remove an existing uint8_t field
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        ASSERT_(pc->registerField_uint8("intensity"));

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPointField_uint8("intensity", 200);
        ASSERT_(pc->hasPointField("intensity"));

        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "raw";
        p["field_names"]      = "intensity";
        filter.initialize(p);

        filter.filter(map);

        ASSERT_(!pc->hasPointField("intensity"));
        std::cout << "[Test Passed] Remove existing uint8_t field" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 5: Try to remove non-existent field (throw_on_missing_field=true)
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"]       = "raw";
        p["field_names"]            = "non_existent_field";
        p["throw_on_missing_field"] = true;
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
        std::cout << "[Test Passed] Non-existent field throws exception" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 6: Try to remove non-existent field (throw_on_missing_field=false)
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"]       = "raw";
        p["field_names"]            = "non_existent_field";
        p["throw_on_missing_field"] = false;
        filter.initialize(p);

        // Should not throw
        filter.filter(map);
        std::cout << "[Test Passed] Non-existent field silently ignored" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 7: Try to process non-existent layer
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "non_existent_layer";
        p["field_names"]      = "some_field";
        filter.initialize(p);

        // Should not throw, just log warning and skip
        filter.filter(map);
        std::cout << "[Test Passed] Non-existent layer handled gracefully" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 8: Try to process non-CGenericPointsMap layer
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        // CSimplePointsMap is not a CGenericPointsMap
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        pc->insertPoint(1.0f, 2.0f, 3.0f);
        map.layers["simple"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "simple";
        p["field_names"]      = "some_field";
        filter.initialize(p);

        // Should not throw, just log warning and skip
        filter.filter(map);
        std::cout << "[Test Passed] Non-CGenericPointsMap layer skipped" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 9: Remove field from specific named layer
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc1 = mrpt::maps::CGenericPointsMap::Create();
        auto                   pc2 = mrpt::maps::CGenericPointsMap::Create();

        ASSERT_(pc1->registerField_float("field_a"));
        ASSERT_(pc2->registerField_float("field_b"));

        pc1->insertPoint(1.0f, 2.0f, 3.0f);
        pc1->insertPointField_float("field_a", 1.0f);

        pc2->insertPoint(1.0f, 2.0f, 3.0f);
        pc2->insertPointField_float("field_b", 2.0f);

        map.layers["layer1"] = pc1;
        map.layers["layer2"] = pc2;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "layer1";
        p["field_names"]      = "field_a";
        filter.initialize(p);

        filter.filter(map);

        // Only layer1's field should be removed
        ASSERT_(!pc1->hasPointField("field_a"));
        ASSERT_(pc2->hasPointField("field_b"));
        std::cout << "[Test Passed] Remove field from specific layer" << std::endl;
    }

    // ---------------------------------------------------------
    // Case 10: Remove multiple fields at once
    // ---------------------------------------------------------
    {
        mp2p_icp::metric_map_t map;
        auto                   pc = mrpt::maps::CGenericPointsMap::Create();

        // Register multiple fields
        ASSERT_(pc->registerField_float("field1"));
        ASSERT_(pc->registerField_double("field2"));
        ASSERT_(pc->registerField_uint16("field3"));

        pc->insertPoint(1.0f, 2.0f, 3.0f);
        pc->insertPointField_float("field1", 1.0f);
        pc->insertPointField_double("field2", 2.0);
        pc->insertPointField_uint16("field3", 3);

        ASSERT_(pc->hasPointField("field1"));
        ASSERT_(pc->hasPointField("field2"));
        ASSERT_(pc->hasPointField("field3"));

        map.layers["raw"] = pc;

        FilterRemovePointCloudField filter;
        mrpt::containers::yaml      p;
        p["pointcloud_layer"] = "raw";
        // Pass as sequence
        p["field_names"] = mrpt::containers::yaml::Sequence();
        p["field_names"].asSequence().push_back("field1");
        p["field_names"].asSequence().push_back("field2");
        p["field_names"].asSequence().push_back("field3");
        filter.initialize(p);

        filter.filter(map);

        // All three fields should be removed
        ASSERT_(!pc->hasPointField("field1"));
        ASSERT_(!pc->hasPointField("field2"));
        ASSERT_(!pc->hasPointField("field3"));
        std::cout << "[Test Passed] Remove multiple fields at once" << std::endl;
    }
}

int main()
{
    try
    {
        test_remove_field();
        std::cout << "\nAll FilterRemovePointCloudField tests passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test suite failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
