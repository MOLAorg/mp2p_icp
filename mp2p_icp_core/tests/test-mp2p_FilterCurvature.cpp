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
 * @file   test-mp2p_FilterCurvature.cpp
 * @brief  Unit tests for FilterCurvature
 * @author Jose Luis Blanco Claraco
 * @date   Aug 11, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterCurvature.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <iostream>

namespace
{
mp2p_icp::metric_map_t makeInputMap()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();

    // This filter works per LiDAR "ring", read as a float point field:
    pc->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_RING_ID);

    const auto lambdaAddPoint = [&](const float x, const float y)
    {
        pc->insertPoint(x, y, 0.0f);
        pc->insertPointField_float(mrpt::maps::CPointsMap::POINT_FIELD_RING_ID, 0.0f);
    };

    // A straight scan line plus a corner, so that both the "larger" and the
    // "smaller" curvature output layers can get points:
    for (int i = 0; i < 20; i++)
    {
        lambdaAddPoint(static_cast<float>(i) * 0.1f, 0.0f);
    }
    for (int i = 1; i < 20; i++)
    {
        lambdaAddPoint(1.9f, static_cast<float>(i) * 0.1f);
    }

    mp2p_icp::metric_map_t m;
    m.layers["raw"] = pc;
    return m;
}

void test_basic_filtering()
{
    mp2p_icp_filters::FilterCurvature filter;

    mrpt::containers::yaml p;
    p["input_pointcloud_layer"]         = "raw";
    p["output_layer_larger_curvature"]  = "edges";
    p["output_layer_smaller_curvature"] = "planes";
    p["max_cosine"]                     = 0.5;
    p["min_clearance"]                  = 0.02;
    p["max_gap"]                        = 1.0;

    filter.initialize(p);

    auto m = makeInputMap();
    filter.filter(m);

    ASSERT_(m.layers.count("edges") == 1);
    ASSERT_(m.layers.count("planes") == 1);
}

// The three numeric parameters must accept formulas that are re-evaluated on
// every ParameterSource::realize(), not only parsed once at load time.
void test_dynamic_parameters()
{
    mp2p_icp_filters::FilterCurvature filter;

    mrpt::containers::yaml p;
    p["input_pointcloud_layer"]         = "raw";
    p["output_layer_larger_curvature"]  = "edges";
    p["output_layer_smaller_curvature"] = "planes";
    p["max_cosine"]                     = "0.5*CURV_SCALE";
    p["min_clearance"]                  = "0.02*CURV_SCALE";
    p["max_gap"]                        = "1.0*CURV_SCALE";

    filter.initialize(p);

    mp2p_icp::ParameterSource ps;
    filter.attachToParameterSource(ps);

    // Note: the values below are all different from the defaults of these
    // fields, so that the assertions cannot pass by accident:
    ps.updateVariable("CURV_SCALE", 3.0);
    ps.realize();
    ASSERT_NEAR_(filter.params.max_cosine, 1.5, 1e-5);
    ASSERT_NEAR_(filter.params.min_clearance, 0.06, 1e-5);
    ASSERT_NEAR_(filter.params.max_gap, 3.0, 1e-5);

    // Re-evaluation with new variable values is what a static YAML load, which
    // silently truncates a formula at its first non-numeric character, cannot
    // do:
    ps.updateVariable("CURV_SCALE", 2.0);
    ps.realize();
    ASSERT_NEAR_(filter.params.max_cosine, 1.0, 1e-5);
    ASSERT_NEAR_(filter.params.min_clearance, 0.04, 1e-5);
    ASSERT_NEAR_(filter.params.max_gap, 2.0, 1e-5);

    // And it must still run:
    auto m = makeInputMap();
    filter.filter(m);
}
}  // namespace

int main()
{
    try
    {
        test_basic_filtering();
        std::cout << "test_basic_filtering: Success" << std::endl;

        test_dynamic_parameters();
        std::cout << "test_dynamic_parameters: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
