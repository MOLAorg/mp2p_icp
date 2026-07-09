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
 * @file   test-mp2p_FilterEdgesPlanes
 * @brief  Unit tests for FilterEdgesPlanes
 * @author Jose Luis Blanco Claraco
 * @date   Jan 6, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterEdgesPlanes.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        // 1. Setup a point cloud with a plane and an edge
        auto pc = mrpt::maps::CSimplePointsMap::Create();

        // Add a horizontal Plane at Z=0 (Voxel 0,0,0)
        constexpr float POINTS_DENSITY = 0.025f;

        for (float x = 0.1f; x < 0.9f; x += POINTS_DENSITY)
        {
            for (float y = 0.1f; y < 0.9f; y += POINTS_DENSITY)
            {
                pc->insertPoint(x, y, 0.0f);
            }
        }

        // Add a vertical Edge at X=2, Y=0
        for (float z = 0.1f; z < 0.9f; z += POINTS_DENSITY)
        {
            pc->insertPoint(2.0f, 0.0f, z);
        }

        // 2. Configure Filter
        FilterEdgesPlanes      filter;
        mrpt::containers::yaml p;
        p["input_pointcloud_layer"]     = "raw";
        p["voxel_filter_resolution"]    = 0.25f;
        p["decimation"]                 = 1;
        p["full_pointcloud_decimation"] = 0;

        // Typical thresholds for classification
        p["edge_min_e2_e0"]  = 30.0f;
        p["edge_min_e2_e1"]  = 30.0f;
        p["plane_min_e2_e0"] = 100.0f;
        p["plane_min_e1_e0"] = 100.0f;

        filter.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
        filter.initialize(p);

        // 3. Run Filter
        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;
        filter.filter(map);

        // 4. Verify Results
        const auto& outPlanes = map.planes;
        auto        outEdges  = map.layer<mrpt::maps::CPointsMap>("edge_points");

        ASSERT_(outEdges != nullptr);

        std::cout << "Planes found: " << outPlanes.size() << std::endl;
        std::cout << "Edges found: " << outEdges->size() << std::endl;

        // The horizontal points should be in planes, the vertical column in edges
        ASSERT_(outPlanes.size() > 15);
        ASSERT_(outEdges->size() > 30);

        std::cout << "FilterEdgesPlanes Unit Test Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}