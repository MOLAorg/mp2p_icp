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
 * @file   test-mp2p_FilterNormals.cpp
 * @brief  Unit test for FilterNormals
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterNormals.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{
FilterNormals makeFilter(uint32_t knn = 8)
{
    FilterNormals          filter;
    mrpt::containers::yaml p;
    p["pointcloud_layer"]    = "raw";
    p["knn"]                 = knn;
    p["max_search_distance"] = 1.0;
    filter.initialize(p);
    return filter;
}

/** A 10x10 grid on the z=0 plane, 0.1 m spacing. Optionally, with a constant
 *  view vector for all points. */
mrpt::maps::CGenericPointsMap::Ptr createPlane(const float* viewZ = nullptr)
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    if (viewZ)
    {
        for (const char* f : {"view_x", "view_y", "view_z"})
        {
            pc->registerField_float(f);
        }
    }
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            pc->insertPoint(0.1f * static_cast<float>(i), 0.1f * static_cast<float>(j), 0.0f);
            if (viewZ)
            {
                pc->insertPointField_float("view_x", 0.0f);
                pc->insertPointField_float("view_y", 0.0f);
                pc->insertPointField_float("view_z", *viewZ);
            }
        }
    }
    return pc;
}

struct NormalFields
{
    const mrpt::aligned_std_vector<float>* nx = nullptr;
    const mrpt::aligned_std_vector<float>* ny = nullptr;
    const mrpt::aligned_std_vector<float>* nz = nullptr;
};

NormalFields getNormals(const mrpt::maps::CGenericPointsMap& pc)
{
    NormalFields n;
    n.nx = pc.getPointsBufferRef_float_field("normal_x");
    n.ny = pc.getPointsBufferRef_float_field("normal_y");
    n.nz = pc.getPointsBufferRef_float_field("normal_z");
    ASSERT_(n.nx && n.ny && n.nz);
    ASSERT_EQUAL_(n.nx->size(), pc.size());
    return n;
}

/** Normals of a planar cloud must be perpendicular to the plane. */
void test_plane_normals()
{
    const auto filter = makeFilter();
    auto       input  = createPlane();

    metric_map_t map;
    map.layers["raw"] = input;
    filter.filter(map);

    const auto n = getNormals(*input);
    for (size_t i = 0; i < input->size(); i++)
    {
        ASSERT_NEAR_(std::abs((*n.nz)[i]), 1.0f, 1e-4f);
        ASSERT_NEAR_((*n.nx)[i], 0.0f, 1e-4f);
        ASSERT_NEAR_((*n.ny)[i], 0.0f, 1e-4f);
    }
}

/** With view vectors, normals must be oriented towards them. */
void test_view_vector_orientation()
{
    for (const float vz : {1.0f, -1.0f})
    {
        const auto filter = makeFilter();
        auto       input  = createPlane(&vz);

        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);

        const auto n = getNormals(*input);
        for (size_t i = 0; i < input->size(); i++)
        {
            ASSERT_NEAR_((*n.nz)[i], vz, 1e-4f);
        }
    }
}

/** Collinear neighborhoods, or isolated points, must get a zero normal. */
void test_degenerate_cases_give_zero_normal()
{
    const auto filter = makeFilter(5);

    // Collinear points:
    {
        auto input = mrpt::maps::CGenericPointsMap::Create();
        for (int i = 0; i < 10; i++)
        {
            input->insertPoint(0.1f * static_cast<float>(i), 0.0f, 0.0f);
        }
        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);

        const auto n = getNormals(*input);
        for (size_t i = 0; i < input->size(); i++)
        {
            ASSERT_EQUAL_((*n.nx)[i], 0.0f);
            ASSERT_EQUAL_((*n.ny)[i], 0.0f);
            ASSERT_EQUAL_((*n.nz)[i], 0.0f);
        }
    }

    // Isolated points, farther apart than max_search_distance:
    {
        auto input = mrpt::maps::CGenericPointsMap::Create();
        for (int i = 0; i < 5; i++)
        {
            input->insertPoint(10.0f * static_cast<float>(i), 5.0f * static_cast<float>(i), 0.0f);
        }
        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);

        const auto n = getNormals(*input);
        for (size_t i = 0; i < input->size(); i++)
        {
            ASSERT_EQUAL_((*n.nz)[i], 0.0f);
        }
    }
}

/** Applying the filter twice must reuse the existing normal fields. */
void test_rerun_on_existing_fields()
{
    const auto filter = makeFilter();
    auto       input  = createPlane();

    metric_map_t map;
    map.layers["raw"] = input;
    filter.filter(map);
    filter.filter(map);

    const auto n = getNormals(*input);
    ASSERT_NEAR_(std::abs((*n.nz)[0]), 1.0f, 1e-4f);
}

/** Empty clouds are a no-op, and a missing layer must throw. */
void test_empty_and_missing_layer()
{
    const auto filter = makeFilter();
    {
        auto         input = mrpt::maps::CGenericPointsMap::Create();
        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);
        ASSERT_EQUAL_(input->size(), 0U);
    }
    {
        metric_map_t map;
        bool         didThrow = false;
        try
        {
            filter.filter(map);
        }
        catch (const std::exception&)
        {
            didThrow = true;
        }
        ASSERT_(didThrow);
    }
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_plane_normals();
        std::cout << "test_plane_normals: Success ✅" << std::endl;

        test_view_vector_orientation();
        std::cout << "test_view_vector_orientation: Success ✅" << std::endl;

        test_degenerate_cases_give_zero_normal();
        std::cout << "test_degenerate_cases_give_zero_normal: Success ✅" << std::endl;

        test_rerun_on_existing_fields();
        std::cout << "test_rerun_on_existing_fields: Success ✅" << std::endl;

        test_empty_and_missing_layer();
        std::cout << "test_empty_and_missing_layer: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
