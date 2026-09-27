/*
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
 * @file   test-mp2p_FilterMLS.cpp
 * @brief  Unit tests for FilterMLS: per-point fields must follow their point
 * @author Jose Luis Blanco Claraco
 * @date   Sep 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterMLS.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;

namespace
{
mrpt::maps::CPointsMap::Ptr run_mls(const mrpt::maps::CGenericPointsMap::Ptr& pc)
{
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pc;

    FilterMLS              filter;
    mrpt::containers::yaml p        = mrpt::containers::yaml::Map();
    p["input_pointcloud_layer"]     = "raw";
    p["output_pointcloud_layer"]    = "mls";
    p["search_radius"]              = 0.05;
    p["polynomial_order"]           = 1;
    p["min_neighbors_for_fit"]      = 5;
    p["parallelization_grain_size"] = 8;  // many small tasks, to force interleaving
    filter.initialize(p);
    filter.filter(map);

    return map.layer<mrpt::maps::CPointsMap>("mls");
}
}  // namespace

int main()
{
    try
    {
        // A flat grid, with each point tagging its own original (x,y) in two
        // extra fields. Projecting onto the z=0 plane must not change x,y, so
        // every output point must still carry its own tags.
        auto pc = mrpt::maps::CGenericPointsMap::Create();
        pc->registerField_float("src_x");
        pc->registerField_float("src_y");
        for (int ix = 0; ix < 300; ix++)
        {
            for (int iy = 0; iy < 300; iy++)
            {
                const float x = 0.01f * static_cast<float>(ix);
                const float y = 0.01f * static_cast<float>(iy);
                pc->insertPointFast(x, y, 0.0f);
                pc->insertPointField_float("src_x", x);
                pc->insertPointField_float("src_y", y);
            }
        }

        const auto out1 = run_mls(pc);
        ASSERT_(out1);
        ASSERT_(!out1->empty());

        const auto& xs    = out1->getPointsBufferRef_x();
        const auto& ys    = out1->getPointsBufferRef_y();
        const auto* src_x = out1->getPointsBufferRef_float_field("src_x");
        const auto* src_y = out1->getPointsBufferRef_float_field("src_y");
        ASSERT_(src_x && src_y);

        size_t mismatches = 0;
        for (size_t i = 0; i < out1->size(); i++)
        {
            if (std::abs(xs[i] - (*src_x)[i]) > 1e-3f || std::abs(ys[i] - (*src_y)[i]) > 1e-3f)
            {
                mismatches++;
            }
        }
        std::cout << "Output points: " << out1->size() << ", field mismatches: " << mismatches
                  << "\n";
        ASSERT_EQUAL_(mismatches, 0U);

        // The output must not depend on thread scheduling:
        const auto out2 = run_mls(pc);
        ASSERT_EQUAL_(out1->size(), out2->size());
        const auto& xs2 = out2->getPointsBufferRef_x();
        const auto& ys2 = out2->getPointsBufferRef_y();
        for (size_t i = 0; i < out1->size(); i++)
        {
            ASSERT_(xs[i] == xs2[i] && ys[i] == ys2[i]);
        }

        std::cout << "FilterMLS Unit Test Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
