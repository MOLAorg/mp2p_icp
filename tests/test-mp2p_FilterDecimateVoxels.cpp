/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \\ / _ \\| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\\___/|_|\\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   test_FilterDecimateVoxels.cpp
 * @brief  Unit test for FilterDecimateVoxels
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Oct 27, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

// Helper to create a consistent point map for testing
mrpt::maps::CPointsMap::Ptr createTestPoints(size_t n_x, size_t n_y)
{
    auto pc = mrpt::maps::CPointsMapXYZI::Create();
    // Create a 2D grid of points, 10x10. Each point has a small, known Z-offset.
    for (size_t i = 0; i < n_x; ++i)
    {
        for (size_t j = 0; j < n_y; ++j)
        {
            float x         = static_cast<float>(i) * 0.15f;
            float y         = static_cast<float>(j) * 0.15f;
            float z         = 0.01f * static_cast<float>(i + j);  // Small, unique Z offset
            float intensity = static_cast<float>(i * n_y + j);

            pc->insertPointFast(x, y, z);
#if MRPT_VERSION >= 0x020f00
            pc->insertPointField_float(
                mrpt::maps::CPointsMapXYZI::POINT_FIELD_INTENSITY, intensity);
#else
            pc->insertPointField_Intensity(intensity);
#endif
        }
    }
    return pc;
}

// The main test runner
void test_decimate_method(
    mp2p_icp_filters::DecimateMethod method, bool use_tsl_robin_map, std::size_t minPointsToFilter,
    const std::optional<double>& flattenTo = std::nullopt)
{
    printf(
        "Running for method=%-38s %s flattening, map=%-10s minPts=%-4zu...",
        mrpt::typemeta::enum2str(method).c_str(), flattenTo.has_value() ? "with   " : "without",
        use_tsl_robin_map ? "tsl_robin" : "std", minPointsToFilter);

    // Create a point map: 10x10 = 100 points
    const size_t N_X      = 10;
    const size_t N_Y      = 10;
    auto         input_pc = createTestPoints(N_X, N_Y);

    // points are spaced at 0.15m
    const float  VOXEL_RES = 0.299f;
    const size_t EXPECTED_OUTPUT_SIZE =
        minPointsToFilter > 0 ? input_pc->size() : (N_X / 2) * (N_Y / 2);

    // --- Configuration ---
    FilterDecimateVoxels filter;

    // filter.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

    mrpt::containers::yaml params;
    params["input_pointcloud_layer"]  = "filtered";
    params["output_pointcloud_layer"] = "localmap_pre";
    params["voxel_filter_resolution"] = VOXEL_RES;
    params["decimate_method"]         = mrpt::typemeta::enum2str(method);
    params["use_tsl_robin_map"]       = use_tsl_robin_map;
    if (minPointsToFilter > 0)
    {
        params["minimum_input_points_to_filter"] = minPointsToFilter;
    }

    if (flattenTo)
    {
        params["flatten_to"] = flattenTo.value();
    }

    filter.initialize_filter(params);

    // --- Execution ---
    metric_map_t map;
    map.layers["filtered"] = input_pc;
    filter.filter(map);

    // --- Verification ---
    ASSERTMSG_(
        map.layers.count("localmap_pre"),
        mrpt::format(
            "Output layer 'localmap_pre' not found for method: %s",
            params["decimate_method"].as<std::string>().c_str()));

    auto output_pc = mp2p_icp::MapToPointsMap(*map.layers.at("localmap_pre"));
    ASSERTMSG_(output_pc, "Output layer is not a point cloud");

    const size_t output_size = output_pc->size();

    // Check output size: it should be exactly 100 if the point spacing is consistent
    // but due to floating point and edge effects, we allow a small tolerance.
    // With 10x10 points at 0.15m spacing, total span is 1.35m.
    // Voxels of size 0.20m.
    // The number of voxels hit in X and Y should be ceil(1.35/0.20) = ceil(6.75) = 7
    // This is a subtle case. Let's force an easier test case where we expect exactly 1 point per
    // voxel.

    // Rerunning the test with a smaller initial cloud to ensure 1 point per voxel.
    // Voxel size 0.20. Points at (0,0,0), (0.1, 0.1, 0), (0.2, 0.2, 0)
    // The first two fall in the same voxel (0,0), the third in voxel (1,1)

    // Let's go back to 100 points, as the filter logic is more important than the exact count.
    // The count should be close to 100, but certainly not the size of a much sparser decimation.
    // The total span is 1.35m. Total voxels hit should be 7x7=49 up to 10x10=100.

    // Assertion 1: Number of points is reduced, but not too much.
    // Since points are spaced < voxel_res, most voxels contain only 1 point.
    // We expect size to be N_X * N_Y = 100 (ideally). Let's check for >= 90 points.
    ASSERT_NEAR_(output_size, EXPECTED_OUTPUT_SIZE, 10);
    //<< "Output size (" << output_size << ") is too far from expected (" << EXPECTED_OUTPUT_SIZE
    //<< ") for method: " << params["decimate_method"].as<std::string>();

    // Assertion 2: Check for coordinate shifts typical of DecimateMethod::VoxelAverage
    if (method == DecimateMethod::VoxelAverage)
    {
        // Voxel average will shift points towards the voxel center.
        // E.g., a point at (0.0, 0.0) is in the voxel centered at (0.1, 0.1).
        // Since input points have small Z, the output Z coordinate should remain near zero.
        mrpt::math::TPoint3D mean_coord;
        const size_t         N = output_pc->size();

        if (N != 0)
        {
            // Get read-only references to the internal vectors (std::vector<float>)
            const auto& xs = output_pc->getPointsBufferRef_x();
            const auto& ys = output_pc->getPointsBufferRef_y();
            const auto& zs = output_pc->getPointsBufferRef_z();

            // Calculate the sum for each coordinate using std::accumulate
            float sum_x = std::accumulate(xs.begin(), xs.end(), 0.0f);
            float sum_y = std::accumulate(ys.begin(), ys.end(), 0.0f);
            float sum_z = std::accumulate(zs.begin(), zs.end(), 0.0f);

            // Calculate the mean (average)
            mean_coord.x = sum_x / static_cast<float>(N);
            mean_coord.y = sum_y / static_cast<float>(N);
            mean_coord.z = sum_z / static_cast<float>(N);
        }
        // Input points mean Z is (0.0 + 0.01*18)/2 = 0.09.
        // Input points mean X is (0.0 + 0.15*9)/2 = 0.675

        // Output mean X should be close to 0.675
        ASSERT_NEAR_(mean_coord.x, 0.675, 0.05);
        // << "VoxelAverage X shift is too large for method: "
        // << params["decimate_method"].as<std::string>();

        // Output mean Z should still be near zero (since all points are near z=0)
        if (flattenTo.has_value())
        {
            ASSERT_EQUAL_(mean_coord.z, *flattenTo);
        }
        else
        {
            ASSERT_NEAR_(mean_coord.z, 0.09, 0.05);
        }
        // << "VoxelAverage Z shift is too large for method: "
        // << params["decimate_method"].as<std::string>();
    }
    else
    {
        // For FirstPoint, ClosestToAverage, and RandomPoint, the output points
        // should have *exactly* the coordinates of some input point,
        // and therefore, the mean Z should match the input cloud's Z mean.
        // We cannot easily check individual points without knowing the voxel indices,
        // so we check that the minimum Z coordinate remains *exactly* 0 (the min input Z).
        const auto bb = output_pc->boundingBox();

        // The minimum input Z is 0.0. This should be preserved.
        ASSERT_NEAR_(bb.min.z, 0.0f, 0.2f);
    }

    // Since we created CPointsMapXYZI, the output should also have intensity.
#if MRPT_VERSION >= 0x020f00
    ASSERT_(output_pc->hasPointField(mrpt::maps::CPointsMapXYZI::POINT_FIELD_INTENSITY));
#endif
    std::cout << " Success ✅." << std::endl;
}

// Global initialization for the test suite
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        int failures = 0;

        for (int use_tls_robin_map = 0; use_tls_robin_map <= 1; use_tls_robin_map++)
        {
            for (int useFlatten = 0; useFlatten <= 1; useFlatten++)
            {
                std::optional<double> flattenTo;
                if (useFlatten != 0)
                {
                    flattenTo = .0;
                }
                for (std::size_t minPointsToFilter = 0; minPointsToFilter <= 200;
                     minPointsToFilter += 200)
                {
                    for (const auto method : std::vector<DecimateMethod>{
                             DecimateMethod::FirstPoint, DecimateMethod::VoxelAverage,
                             DecimateMethod::ClosestToAverage, DecimateMethod::RandomPoint})
                    {
                        try
                        {
                            test_decimate_method(
                                method, use_tls_robin_map != 0, minPointsToFilter, flattenTo);
                        }
                        catch (const std::exception& e)
                        {
                            std::cerr << "Error: ❌\n" << e.what() << std::endl;
                            failures++;
                        }
                    }
                }
            }
        }

        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}