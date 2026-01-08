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
 * @file   test-mp2p_FilterByExpression.cpp
 * @brief  Unit tests for FilterByExpression
 * @author Jose Luis Blanco Claraco
 * @date   Dec 26, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterByExpression.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/version.h>

using namespace mp2p_icp_filters;

namespace
{
#if MRPT_VERSION >= 0x020f04  // 2.15.4

void FilterByExpression_SpatialFiltering()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->insertPoint(1.0f, 0.0f, 0.0f);  // Range 1.0 (Inside)
    pc->insertPoint(5.0f, 0.0f, 0.0f);  // Range 5.0 (Outside)

    FilterByExpression     filter;
    mrpt::containers::yaml p;
    p["input_pointcloud_layer"]  = "raw";
    p["expression"]              = "(x^2+y^2+z^2) < 3.0^2";
    p["output_layer_passed"]     = "near";
    p["output_layer_not_passed"] = "far";
    filter.initialize(p);

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);

    ASSERT_EQUAL_(map.layer<mrpt::maps::CPointsMap>("near")->size(), 1UL);
    ASSERT_EQUAL_(map.layer<mrpt::maps::CPointsMap>("far")->size(), 1UL);
}

void FilterByExpression_IntensityAndLogic()
{
    constexpr auto INTENSITY = POINT_FIELD_INTENSITY;

    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float(INTENSITY);
    pc->insertPoint(0, 0, 0);  // Low intensity
    pc->insertPointField_float(INTENSITY, 0.1f);

    pc->insertPoint(0, 0, 0);  // High intensity
    pc->insertPointField_float(INTENSITY, 0.9f);

    pc->insertPoint(1.0, 0, 0);  // High intensity, bad "x"
    pc->insertPointField_float(INTENSITY, 0.9f);

    // Instantiate as class factory:
    auto filter = std::dynamic_pointer_cast<FilterBase>(
        mrpt::rtti::classFactory("mp2p_icp_filters::FilterByExpression"));
    ASSERT_(filter);

    mrpt::containers::yaml p;
    p["input_pointcloud_layer"] = "raw";
    p["expression"]             = "(intensity > 0.5) and (abs(x)<1e-3)";
    p["output_layer_passed"]    = "high_i";
    filter->initialize(p);

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;
    filter->filter(map);

    ASSERT_EQUAL_(map.layer<mrpt::maps::CPointsMap>("high_i")->size(), 1UL);
}

void FilterByExpression_CustomFields()
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_uint16(POINT_FIELD_RING_ID);
    pc->registerField_uint8("SEMANTICS");
    pc->registerField_double("latitude");

    pc->insertPoint(0, 0, 0);
    pc->insertPointField_uint16(POINT_FIELD_RING_ID, 32);
    pc->insertPointField_uint8("SEMANTICS", 12);
    pc->insertPointField_double("latitude", 3.14);

    FilterByExpression     filter;
    mrpt::containers::yaml p;
    p["input_pointcloud_layer"] = "raw";
    p["expression"]             = "(ring == 32) and (latitude>0)";
    p["output_layer_passed"]    = "ring32";
    filter.initialize(p);

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;
    filter.filter(map);

    ASSERT_EQUAL_(map.layer<mrpt::maps::CPointsMap>("ring32")->size(), 1UL);

    // Test running twice:
    map.layers.erase("ring32");

    filter.filter(map);

    ASSERT_EQUAL_(map.layer<mrpt::maps::CPointsMap>("ring32")->size(), 1UL);
}
#endif
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
#if MRPT_VERSION >= 0x020f04  // 2.15.4
        FilterByExpression_SpatialFiltering();
        FilterByExpression_IntensityAndLogic();
        FilterByExpression_CustomFields();
        std::cout << "Success ✅." << std::endl;
#else
        std::cout << "DISABLED TEST: Requires MRPT>=2.15.4" << std::endl;
#endif
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
