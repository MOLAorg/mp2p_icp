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
 * @file   test-mp2p_FilterPolygon2D.cpp
 * @brief  Unit test for FilterPolygon2D
 * @author Jose Luis Blanco Claraco
 * @date   May 26, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterPolygon2D.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{

// Helper: build a YAML sequence of [x, y] vertices from a vector of pairs.
mrpt::containers::yaml makePolygonYaml(const std::vector<std::pair<double, double>>& vertices)
{
    mrpt::containers::yaml seq = mrpt::containers::yaml::Sequence();
    for (const auto& [x, y] : vertices)
    {
        seq.push_back(mrpt::containers::yaml::Sequence({x, y}));
    }
    return seq;
}

mrpt::maps::CSimplePointsMap::Ptr asPoints(const metric_map_t& map, const std::string& layer)
{
    return std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(map.layers.at(layer));
}

// A 10x10 axis-aligned square: [0,0]-[10,10]
void test_square_inside_outside()
{
    FilterPolygon2D filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "inside";
    params["outside_pointcloud_layer"] = "outside";
    params["polygon"] = makePolygonYaml({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});

    filter.initialize(params);

    auto input = mrpt::maps::CSimplePointsMap::Create();
    // Inside (3):
    input->insertPoint(5.0f, 5.0f, 0.0f);
    input->insertPoint(2.0f, 8.0f, 0.0f);
    input->insertPoint(9.0f, 1.0f, 0.0f);
    // Outside (4):
    input->insertPoint(15.0f, 5.0f, 0.0f);
    input->insertPoint(-2.0f, 5.0f, 0.0f);
    input->insertPoint(5.0f, -3.0f, 0.0f);
    input->insertPoint(11.0f, 11.0f, 0.0f);

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto inside  = asPoints(map, "inside");
    auto outside = asPoints(map, "outside");
    ASSERT_(inside);
    ASSERT_(outside);

    ASSERT_EQUAL_(inside->size(), 3UL);
    ASSERT_EQUAL_(outside->size(), 4UL);

    // Total is preserved.
    ASSERT_EQUAL_(inside->size() + outside->size(), input->size());
}

// Same square but with a Z crop; verify points outside the [z_min,z_max]
// range fall into the "outside" layer even when their XY is inside.
void test_z_range()
{
    FilterPolygon2D filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "inside";
    params["outside_pointcloud_layer"] = "outside";
    params["z_min"]                    = 0.5;
    params["z_max"]                    = 2.0;
    params["polygon"] = makePolygonYaml({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});

    filter.initialize(params);

    auto input = mrpt::maps::CSimplePointsMap::Create();
    input->insertPoint(5.0f, 5.0f, 1.0f);  // inside (z ok)
    input->insertPoint(5.0f, 5.0f, 2.0f);  // inside (z == z_max, inclusive)
    input->insertPoint(5.0f, 5.0f, 0.0f);  // outside (z below)
    input->insertPoint(5.0f, 5.0f, 3.0f);  // outside (z above)
    input->insertPoint(20.0f, 1.0f, 1.0f);  // outside (xy outside)

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto inside  = asPoints(map, "inside");
    auto outside = asPoints(map, "outside");
    ASSERT_(inside);
    ASSERT_(outside);

    ASSERT_EQUAL_(inside->size(), 2UL);
    ASSERT_EQUAL_(outside->size(), 3UL);
}

// Non-convex (L-shaped) polygon: points in the missing corner must be excluded.
void test_non_convex_polygon()
{
    FilterPolygon2D filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]   = "raw";
    params["inside_pointcloud_layer"]  = "inside";
    params["outside_pointcloud_layer"] = "outside";
    // L-shape: full 10x10 square with the (4..10, 4..10) corner removed.
    params["polygon"] = makePolygonYaml(
        {{0.0, 0.0}, {10.0, 0.0}, {10.0, 4.0}, {4.0, 4.0}, {4.0, 10.0}, {0.0, 10.0}});

    filter.initialize(params);

    auto input = mrpt::maps::CSimplePointsMap::Create();
    // Inside the L (3):
    input->insertPoint(2.0f, 7.0f, 0.0f);
    input->insertPoint(7.0f, 2.0f, 0.0f);
    input->insertPoint(1.0f, 1.0f, 0.0f);
    // Outside (2): notch + far away
    input->insertPoint(7.0f, 7.0f, 0.0f);  // in the removed corner
    input->insertPoint(12.0f, 12.0f, 0.0f);  // far away

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto inside  = asPoints(map, "inside");
    auto outside = asPoints(map, "outside");
    ASSERT_(inside);
    ASSERT_(outside);

    ASSERT_EQUAL_(inside->size(), 3UL);
    ASSERT_EQUAL_(outside->size(), 2UL);
}

// Only the inside layer requested: the outside layer must not be created.
void test_only_inside_layer()
{
    FilterPolygon2D filter;

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "raw";
    params["inside_pointcloud_layer"] = "inside";
    params["polygon"] = makePolygonYaml({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});

    filter.initialize(params);

    auto input = mrpt::maps::CSimplePointsMap::Create();
    input->insertPoint(5.0f, 5.0f, 0.0f);  // inside
    input->insertPoint(50.0f, 5.0f, 0.0f);  // outside (dropped)

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto inside = asPoints(map, "inside");
    ASSERT_(inside);
    ASSERT_EQUAL_(inside->size(), 1UL);
    ASSERT_(map.layers.count("outside") == 0);
}

// Parse the filter from a literal YAML text block, exactly as it would appear
// inside an sm2mm / ICP pipeline file. This documents the expected formatting
// of the 'polygon' parameter (a sequence of [x, y] vertices).
void test_parse_from_yaml_text()
{
    const char* yaml_text = R"yaml(
input_pointcloud_layer: 'raw'
inside_pointcloud_layer: 'inside'
outside_pointcloud_layer: 'outside'
z_min: 0.0
z_max: 5.0
polygon:
  - [0.0, 0.0]
  - [10.0, 0.0]
  - [10.0, 10.0]
  - [0.0, 10.0]
)yaml";

    FilterPolygon2D filter;
    filter.initialize(mrpt::containers::yaml::FromText(yaml_text));

    auto input = mrpt::maps::CSimplePointsMap::Create();
    input->insertPoint(5.0f, 5.0f, 1.0f);  // inside (xy + z ok)
    input->insertPoint(5.0f, 5.0f, 9.0f);  // outside (z above z_max)
    input->insertPoint(20.0f, 5.0f, 1.0f);  // outside (xy outside)

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto inside  = asPoints(map, "inside");
    auto outside = asPoints(map, "outside");
    ASSERT_(inside);
    ASSERT_(outside);

    ASSERT_EQUAL_(inside->size(), 1UL);
    ASSERT_EQUAL_(outside->size(), 2UL);
}

// Bad configurations must throw at initialize().
void test_invalid_configuration()
{
    {  // No output layer at all.
        FilterPolygon2D        filter;
        mrpt::containers::yaml params;
        params["input_pointcloud_layer"] = "raw";
        params["polygon"]                = makePolygonYaml({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});

        bool thrown = false;
        try
        {
            filter.initialize(params);
        }
        catch (const std::exception&)
        {
            thrown = true;
        }
        ASSERT_(thrown);
    }

    {  // Too few vertices.
        FilterPolygon2D        filter;
        mrpt::containers::yaml params;
        params["input_pointcloud_layer"]  = "raw";
        params["inside_pointcloud_layer"] = "inside";
        params["polygon"]                 = makePolygonYaml({{0.0, 0.0}, {1.0, 0.0}});

        bool thrown = false;
        try
        {
            filter.initialize(params);
        }
        catch (const std::exception&)
        {
            thrown = true;
        }
        ASSERT_(thrown);
    }

    {  // Missing polygon.
        FilterPolygon2D        filter;
        mrpt::containers::yaml params;
        params["input_pointcloud_layer"]  = "raw";
        params["inside_pointcloud_layer"] = "inside";

        bool thrown = false;
        try
        {
            filter.initialize(params);
        }
        catch (const std::exception&)
        {
            thrown = true;
        }
        ASSERT_(thrown);
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_square_inside_outside();
        std::cout << "test_square_inside_outside: Success" << std::endl;

        test_z_range();
        std::cout << "test_z_range: Success" << std::endl;

        test_non_convex_polygon();
        std::cout << "test_non_convex_polygon: Success" << std::endl;

        test_only_inside_layer();
        std::cout << "test_only_inside_layer: Success" << std::endl;

        test_parse_from_yaml_text();
        std::cout << "test_parse_from_yaml_text: Success" << std::endl;

        test_invalid_configuration();
        std::cout << "test_invalid_configuration: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}
