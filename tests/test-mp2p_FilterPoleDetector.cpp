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
 * @file   test-mp2p_FilterPoleDetector
 * @brief  Unit tests for FilterPoleDetector
 * @author Jose Luis Blanco Claraco
 * @date   Jan 6, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterPoleDetector.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        // 1. Setup a point cloud with ground and one pole
        auto pc = mrpt::maps::CSimplePointsMap::Create();

        // Ground points: a 5x5 grid at Z=0
        for (float x = -2.0f; x <= 2.0f; x += 0.5f)
        {
            for (float y = -2.0f; y <= 2.0f; y += 0.5f)
            {
                pc->insertPoint(x, y, 0.0f);
            }
        }

        // A Pole: tall stack of points at (0,0)
        // Note: This will make cell (0,0) have a high mean Z compared to neighbors
        for (float z = 1.0f; z <= 10.0f; z += 0.5f)
        {
            pc->insertPoint(0.0f, 0.0f, z);
        }

        // 2. Configure Filter
        FilterPoleDetector     filter;
        mrpt::containers::yaml p;
        p["input_pointcloud_layer"]  = "raw";
        p["output_layer_poles"]      = "poles";
        p["output_layer_no_poles"]   = "not_poles";
        p["grid_size"]               = 1.0f;
        p["minimum_relative_height"] = 1.5f;
        p["maximum_relative_height"] = 25.0f;
        p["minimum_pole_points"]     = 3;
        filter.initialize(p);

        // 3. Run Filter
        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;
        filter.filter(map);

        // 4. Verify Results
        auto outPoles = map.layer<mrpt::maps::CPointsMap>("poles");
        auto outOther = map.layer<mrpt::maps::CPointsMap>("not_poles");

        ASSERT_(outPoles != nullptr);
        ASSERT_(outOther != nullptr);

        std::cout << "Pole points detected: " << outPoles->size() << std::endl;

        // The tall vertical stack should be identified as a pole
        ASSERT_(outPoles->size() >= 3);
        // Ground points should be in the other layer
        ASSERT_(outOther->size() > 0);

        std::cout << "FilterPoleDetector Unit Test Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}