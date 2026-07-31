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
 * @file   test_FilterDecimateAdaptive.cpp
 * @brief  Unit test for FilterDecimateAdaptive
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/typemeta/TEnumType.h>

#include <iostream>
#include <optional>

using namespace mp2p_icp_filters;

namespace
{
// Finds the output point whose x coordinate is nearest to `x`, and asserts it
// is within `tol`. Used to check per-voxel representative points without
// depending on output ordering.
float closestX(const mrpt::maps::CPointsMap& pc, float x)
{
    const auto&          xs = pc.getPointsBufferRef_x();
    std::optional<float> best;
    for (const auto v : xs)
    {
        if (!best.has_value() || std::abs(v - x) < std::abs(*best - x))
        {
            best = v;
        }
    }
    ASSERT_(best.has_value());
    return *best;
}
}  // namespace

int main()
{
    try
    {
        // 1. Setup a dense cloud with 10,000 points in a 10x10x1 volume
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        for (int i = 0; i < 10000; ++i)
        {
            pc->insertPoint(
                static_cast<float>(i % 100) * 0.1f,  // X: 0 to 10
                static_cast<float>((i / 100) % 100) * 0.1f,  // Y: 0 to 10
                0.0f  // Z: Flat
            );
        }

        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;

        // ---------------------------------------------------------
        // Test: Reduce 10k points to approx 500
        // ---------------------------------------------------------
        {
            FilterDecimateAdaptive filter;
            mrpt::containers::yaml p;
            p["input_pointcloud_layer"]         = "raw";
            p["output_pointcloud_layer"]        = "adaptive_out";
            p["desired_output_point_count"]     = 500;
            p["voxel_size"]                     = 0.5f;  // Large enough voxels to group points
            p["minimum_input_points_per_voxel"] = 1;

            filter.initialize(p);
            filter.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("adaptive_out");
            ASSERT_(out);

            const size_t outSize = out->size();
            std::cout << "Adaptive Decimation: Input=" << pc->size()
                      << " Desired=500 Output=" << outSize << "\n";

            // The adaptive filter tries to hit the target but depends on voxelization.
            // It should be reasonably close to the target if the voxel grid allows it.
            // Since we have plenty of points and voxels, it should stop exactly at or very close to
            // 500.
            ASSERT_NEAR_(outSize, 500, 20);  // Allow small tolerance

            std::cout << "[Test Passed] Adaptive decimation count check\n";
        }

        // ---------------------------------------------------------
        // Test: decimate_method VoxelAverage and ClosestToAverage, one
        // point per voxel by construction (4 voxels, exactly 4 desired
        // output points), so each output point must correspond to a known
        // per-voxel centroid (VoxelAverage) or a known real point
        // (ClosestToAverage), independently of FirstPoint's behavior.
        // ---------------------------------------------------------
        {
            auto pc2 = mrpt::maps::CSimplePointsMap::Create();
            // 4 voxels along X ([0,1), [1,2), [2,3), [3,4)), 3 points each,
            // asymmetric so the closest-to-mean point is unambiguous.
            const float offsets[3]   = {0.1f, 0.2f, 0.6f};
            const float voxelBase[4] = {0.0f, 1.0f, 2.0f, 3.0f};
            for (const float base : voxelBase)
            {
                for (const float off : offsets)
                {
                    pc2->insertPoint(base + off, 0.0f, 0.0f);
                }
            }

            const float expectedMean[4]    = {0.3f, 1.3f, 2.3f, 3.3f};
            const float expectedClosest[4] = {0.2f, 1.2f, 2.2f, 3.2f};

            for (const auto method :
                 {DecimateMethod::VoxelAverage, DecimateMethod::ClosestToAverage})
            {
                mp2p_icp::metric_map_t map2;
                map2.layers["raw"] = pc2;

                FilterDecimateAdaptive filter;
                mrpt::containers::yaml p;
                p["input_pointcloud_layer"]         = "raw";
                p["output_pointcloud_layer"]        = "out";
                p["desired_output_point_count"]     = 4;
                p["voxel_size"]                     = 1.0f;
                p["minimum_input_points_per_voxel"] = 1;
                p["decimate_method"]                = mrpt::typemeta::enum2str(method);

                filter.initialize(p);
                filter.filter(map2);

                auto out = map2.layer<mrpt::maps::CPointsMap>("out");
                ASSERT_(out);
                ASSERT_EQUAL_(out->size(), 4U);

                const float* expected =
                    (method == DecimateMethod::VoxelAverage) ? expectedMean : expectedClosest;
                const float tol = (method == DecimateMethod::VoxelAverage) ? 1e-4f : 1e-6f;

                for (int i = 0; i < 4; i++)
                {
                    const float got = closestX(*out, expected[i]);
                    ASSERT_NEAR_(got, expected[i], tol);
                }

                std::cout << "[Test Passed] FilterDecimateAdaptive decimate_method="
                          << mrpt::typemeta::enum2str(method) << "\n";
            }
        }

        std::cout << "\nFilterDecimateAdaptive Unit Tests Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}