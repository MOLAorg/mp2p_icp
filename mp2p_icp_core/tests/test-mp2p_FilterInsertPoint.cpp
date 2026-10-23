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
 * @file   test-mp2p_FilterInsertPoint.cpp
 * @brief  Unit tests for FilterInsertPoint
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterInsertPoint.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

int main()
{
    try
    {
        mp2p_icp_filters::FilterInsertPoint filter;

        mrpt::containers::yaml p;
        p["target_layer"] = "trajectory";
        p["point"]        = mrpt::containers::yaml::Sequence({"robot_x", "robot_y", "robot_z + 1"});
        filter.initialize(p);

        mp2p_icp::ParameterSource ps;
        filter.attachToParameterSource(ps);

        mp2p_icp::metric_map_t map;

        // One point per "keyframe", with the variables of each:
        ps.updateVariables({{"robot_x", 1.0}, {"robot_y", 2.0}, {"robot_z", 3.0}});
        ps.realize();
        filter.filter(map);

        ps.updateVariables({{"robot_x", -1.0}, {"robot_y", 5.0}, {"robot_z", 0.0}});
        ps.realize();
        filter.filter(map);

        const auto out = map.point_layer("trajectory");
        ASSERT_(out);
        ASSERT_EQUAL_(out->size(), 2U);

        const auto& xs = out->getPointsBufferRef_x();
        const auto& ys = out->getPointsBufferRef_y();
        const auto& zs = out->getPointsBufferRef_z();
        ASSERT_NEAR_(xs[0], 1.0f, 1e-6f);
        ASSERT_NEAR_(ys[0], 2.0f, 1e-6f);
        ASSERT_NEAR_(zs[0], 4.0f, 1e-6f);
        ASSERT_NEAR_(xs[1], -1.0f, 1e-6f);
        ASSERT_NEAR_(ys[1], 5.0f, 1e-6f);
        ASSERT_NEAR_(zs[1], 1.0f, 1e-6f);

        std::cout << "FilterInsertPoint Unit Test Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
