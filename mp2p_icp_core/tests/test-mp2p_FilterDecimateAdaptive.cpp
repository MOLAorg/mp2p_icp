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

#include <cmath>
#include <iostream>

using namespace mp2p_icp_filters;

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
        // Test: two output layers from one single voxelization pass
        // ---------------------------------------------------------
        {
            FilterDecimateAdaptive filter;
            mrpt::containers::yaml p;
            p["input_pointcloud_layer"]         = "raw";
            p["voxel_size"]                     = 0.5f;
            p["minimum_input_points_per_voxel"] = 1;

            p["outputs"] = mrpt::containers::yaml::Sequence();
            p["outputs"].push_back(mrpt::containers::yaml::Map(
                {{"output_pointcloud_layer", "multi_out_map"},
                 {"desired_output_point_count", 500}}));
            p["outputs"].push_back(mrpt::containers::yaml::Map(
                {{"output_pointcloud_layer", "multi_out_icp"},
                 {"desired_output_point_count", 150}}));

            filter.initialize(p);
            filter.filter(map);

            auto outMap = map.layer<mrpt::maps::CPointsMap>("multi_out_map");
            auto outIcp = map.layer<mrpt::maps::CPointsMap>("multi_out_icp");
            ASSERT_(outMap);
            ASSERT_(outIcp);

            std::cout << "Multi-output decimation: map=" << outMap->size()
                      << " (desired 500), icp=" << outIcp->size() << " (desired 150)\n";

            ASSERT_NEAR_(outMap->size(), 500, 20);
            ASSERT_NEAR_(outIcp->size(), 150, 20);

            std::cout << "[Test Passed] Multi-output decimation count check\n";
        }

        // ---------------------------------------------------------
        // Test: 'outputs' and the single-output keys are exclusive
        // ---------------------------------------------------------
        {
            FilterDecimateAdaptive filter;
            mrpt::containers::yaml p;
            p["input_pointcloud_layer"]     = "raw";
            p["output_pointcloud_layer"]    = "some_layer";
            p["desired_output_point_count"] = 100;
            p["outputs"]                    = mrpt::containers::yaml::Sequence();
            p["outputs"].push_back(mrpt::containers::yaml::Map(
                {{"output_pointcloud_layer", "other_layer"}, {"desired_output_point_count", 100}}));

            bool didThrow = false;
            try
            {
                filter.initialize(p);
            }
            catch (const std::exception&)
            {
                didThrow = true;
            }
            ASSERT_(didThrow);

            std::cout << "[Test Passed] Mutually-exclusive output parameters check\n";
        }

        // ---------------------------------------------------------
        // Test: the decimate_method options
        // ---------------------------------------------------------
        {
            // A cloud of tight clusters of 4 points each, each one well inside
            // its own voxel, so the expected output of every method is known:
            const auto      clustered      = mrpt::maps::CSimplePointsMap::Create();
            constexpr int   kClusters      = 200;
            constexpr float kVoxelSize     = 0.5f;
            constexpr float kSpread        = 0.1f;
            const auto      clusterCenterX = [](int i) { return static_cast<float>(i) + 0.25f; };

            for (int i = 0; i < kClusters; ++i)
            {
                const float cx = clusterCenterX(i);  // one cluster per voxel
                const float cy = 0.25f;
                clustered->insertPoint(cx - kSpread, cy, 0.25f);
                clustered->insertPoint(cx + kSpread, cy, 0.25f);
                clustered->insertPoint(cx, cy - kSpread, 0.25f);
                clustered->insertPoint(cx, cy + kSpread, 0.25f);
            }

            for (const auto& method :
                 {"DecimateMethod::FirstPoint", "DecimateMethod::ClosestToAverage",
                  "DecimateMethod::VoxelAverage", "DecimateMethod::RandomPoint"})
            {
                mp2p_icp::metric_map_t m;
                m.layers["raw"] = clustered;

                FilterDecimateAdaptive filter;
                mrpt::containers::yaml p;
                p["input_pointcloud_layer"]         = "raw";
                p["output_pointcloud_layer"]        = "out";
                p["desired_output_point_count"]     = kClusters;
                p["voxel_size"]                     = kVoxelSize;
                p["minimum_input_points_per_voxel"] = 1;
                p["decimate_method"]                = method;

                filter.initialize(p);
                filter.filter(m);

                auto out = m.layer<mrpt::maps::CPointsMap>("out");
                ASSERT_(out);

                // One point per cluster, whatever the method:
                ASSERT_EQUAL_(out->size(), static_cast<size_t>(kClusters));

                // Every output point must lie within its own cluster's extent:
                for (size_t i = 0; i < out->size(); i++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    out->getPointFast(i, x, y, z);

                    const float nearestCenter = std::round(x - 0.25f) + 0.25f;
                    ASSERT_LT_(std::abs(x - nearestCenter), kSpread + 1e-3f);
                    ASSERT_LT_(std::abs(y - 0.25f), kSpread + 1e-3f);
                }

                std::cout << "[Test Passed] decimate_method=" << method << " (" << out->size()
                          << " points)\n";
            }
        }

        // ---------------------------------------------------------
        // Test: parallel binning must not fragment voxels
        //
        // A rotating lidar revisits the same voxel at input indices that are
        // far apart, so a voxel's points end up in different parallel blocks.
        // The cloud below reproduces that: kSweeps passes over the same
        // kColumns positions, so voxel j holds exactly kSweeps points, at
        // indices j, kColumns+j, 2*kColumns+j, ...
        // ---------------------------------------------------------
        {
            constexpr int   kColumns   = 5000;
            constexpr int   kSweeps    = 8;
            constexpr float kVoxelSize = 0.5f;

            const auto sweptCloud = mrpt::maps::CSimplePointsMap::Create();
            for (int s = 0; s < kSweeps; ++s)
            {
                for (int j = 0; j < kColumns; ++j)
                {
                    // One voxel per column, plus a jitter well inside it so the
                    // kSweeps points of a voxel are all distinct:
                    sweptCloud->insertPoint(
                        static_cast<float>(j) + 0.05f * static_cast<float>(s), 0.25f, 0.25f);
                }
            }

            const auto runFilter = [&](unsigned int minPointsPerVoxel, unsigned int desiredCount)
            {
                mp2p_icp::metric_map_t m;
                m.layers["raw"] = sweptCloud;

                FilterDecimateAdaptive filter;
                mrpt::containers::yaml p;
                p["input_pointcloud_layer"]         = "raw";
                p["output_pointcloud_layer"]        = "out";
                p["desired_output_point_count"]     = desiredCount;
                p["voxel_size"]                     = kVoxelSize;
                p["minimum_input_points_per_voxel"] = minPointsPerVoxel;
                // Force many blocks, so several threads see the same voxels:
                p["parallelization_grain_size"] = 512;

                filter.initialize(p);
                filter.filter(m);

                auto out = m.layer<mrpt::maps::CPointsMap>("out");
                ASSERT_(out);
                return out;
            };

            // (a) A voxel above the threshold must survive, no matter how its
            //     points were distributed across threads:
            {
                auto out = runFilter(kSweeps, kColumns);
                ASSERT_EQUAL_(out->size(), static_cast<size_t>(kColumns));
                std::cout
                    << "[Test Passed] minimum_input_points_per_voxel under parallel binning\n";
            }

            // (b) Asking for exactly one point per voxel must give one point
            //     per voxel, not one per voxel fragment:
            {
                auto out = runFilter(1, kColumns);
                ASSERT_EQUAL_(out->size(), static_cast<size_t>(kColumns));

                std::vector<bool> seen(kColumns, false);
                for (size_t i = 0; i < out->size(); i++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    out->getPointFast(i, x, y, z);

                    const int col = static_cast<int>(std::round(x - 0.05f * 0.5f * (kSweeps - 1)));
                    ASSERT_GE_(col, 0);
                    ASSERT_LT_(col, kColumns);
                    ASSERTMSG_(!seen[col], "Two output points fell in the same voxel");
                    seen[col] = true;
                }
                std::cout << "[Test Passed] one representative per voxel under parallel binning\n";
            }

            // (c) Two identical runs must give bit-identical output:
            {
                auto outA = runFilter(1, kColumns / 3);
                auto outB = runFilter(1, kColumns / 3);

                ASSERT_EQUAL_(outA->size(), outB->size());
                ASSERT_(outA->size() > 0);

                for (size_t i = 0; i < outA->size(); i++)
                {
                    float ax = 0;
                    float ay = 0;
                    float az = 0;
                    float bx = 0;
                    float by = 0;
                    float bz = 0;
                    outA->getPointFast(i, ax, ay, az);
                    outB->getPointFast(i, bx, by, bz);
                    ASSERT_EQUAL_(ax, bx);
                    ASSERT_EQUAL_(ay, by);
                    ASSERT_EQUAL_(az, bz);
                }
                std::cout << "[Test Passed] run-to-run determinism (" << outA->size()
                          << " points)\n";
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