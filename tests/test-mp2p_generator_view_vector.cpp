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
 * @file   test-mp2p_generator_view_vector.cpp
 * @brief  Unit tests for Generator::generate_view_vector feature
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 */

#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/cpu.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>

namespace
{
// Helper: build a CObservationPointCloud from a CGenericPointsMap
mrpt::obs::CObservationPointCloud::Ptr makeObs(
    const mrpt::maps::CGenericPointsMap::Ptr& pts,
    const mrpt::poses::CPose3D&               sensorPose = mrpt::poses::CPose3D::Identity())
{
    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = pts;
    obs->sensorPose = sensorPose;
    return obs;
}

mp2p_icp_filters::GeneratorSet makeGeneratorSet(bool generate_view_vector)
{
    const auto yaml_content = mrpt::format(
        R"(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: "raw"
      generate_view_vector: %s
      metric_map_definition:
        class: mrpt::maps::CGenericPointsMap
)",
        generate_view_vector ? "true" : "false");

    const auto generators = mp2p_icp_filters::generators_from_yaml(
        mrpt::containers::yaml::FromText(yaml_content)["generators"]);

    return generators;
}

// -----------------------------------------------------------------------
// Test 1: view vectors are generated and are unit vectors
// -----------------------------------------------------------------------
void test_view_vector_generated_and_unit()
{
    const auto generators = makeGeneratorSet(true);

    ASSERT_EQUAL_(generators.size(), 1UL);

    // Points at known positions relative to sensor at origin:
    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->insertPoint(5.0f, 0.0f, 0.0f);
    pts->insertPoint(0.0f, 3.0f, 0.0f);
    pts->insertPoint(0.0f, 0.0f, 4.0f);
    pts->insertPoint(1.0f, 1.0f, 1.0f);

    const auto map = mp2p_icp_filters::apply_generators(generators, *makeObs(pts));

    auto outLayer = map.layer<mrpt::maps::CGenericPointsMap>("raw");
    ASSERT_(outLayer);
    ASSERT_EQUAL_(outLayer->size(), 4UL);

    // Each view vector should be a unit vector
    for (size_t i = 0; i < outLayer->size(); i++)
    {
        const float vx = outLayer->getPointField_float(i, "view_x");
        const float vy = outLayer->getPointField_float(i, "view_y");
        const float vz = outLayer->getPointField_float(i, "view_z");

        const float norm2 = vx * vx + vy * vy + vz * vz;
        ASSERT_NEAR_(norm2, 1.0f, 1e-5f);
    }

    std::cout << "test_view_vector_generated_and_unit: ✅\n";
}

// -----------------------------------------------------------------------
// Test 2: view vectors point towards the sensor (inward direction)
// -----------------------------------------------------------------------
void test_view_vector_direction()
{
    const auto generators = makeGeneratorSet(true);

    // Sensor at origin, single point along +X axis at distance 10
    // View vector should point in -X direction (towards sensor)
    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->insertPoint(10.0f, 0.0f, 0.0f);

    const auto map = mp2p_icp_filters::apply_generators(generators, *makeObs(pts));

    auto outLayer = map.layer<mrpt::maps::CGenericPointsMap>("raw");
    ASSERT_(outLayer);
    ASSERT_EQUAL_(outLayer->size(), 1UL);

    const float vx = outLayer->getPointField_float(0, "view_x");
    const float vy = outLayer->getPointField_float(0, "view_y");
    const float vz = outLayer->getPointField_float(0, "view_z");

    // The point is at (10,0,0) and sensor is at (0,0,0).
    // View vector (towards sensor) = normalize((0,0,0)-(10,0,0)) = (-1,0,0)
    ASSERT_NEAR_(vx, -1.0f, 1e-2f);
    ASSERT_NEAR_(vy, 0.0f, 1e-2f);
    ASSERT_NEAR_(vz, 0.0f, 1e-2f);

    std::cout << "test_view_vector_direction: ✅\n";
}

// -----------------------------------------------------------------------
// Test 3: view vectors are correct with a non-zero sensor pose
// -----------------------------------------------------------------------
void test_view_vector_with_sensor_pose()
{
    const auto generators = makeGeneratorSet(true);

    const mrpt::poses::CPose3D robotPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Sensor is at (5, 0, 0) in the world frame; point is at (5, 3, 0)
    // in sensor local frame → (10, 3, 0) in world frame.
    // View vector world = normalize((5,0,0)-(10,3,0)) = normalize((-5,-3,0))
    const mrpt::poses::CPose3D sensorPose(4.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->insertPoint(5.0f, 3.0f, 0.0f);  // in sensor frame

    const auto map =
        mp2p_icp_filters::apply_generators(generators, *makeObs(pts, sensorPose), robotPose);

    auto outLayer = map.layer<mrpt::maps::CGenericPointsMap>("raw");
    ASSERT_(outLayer);
    ASSERT_EQUAL_(outLayer->size(), 1UL);

    const float vx = outLayer->getPointField_float(0, "view_x");
    const float vy = outLayer->getPointField_float(0, "view_y");
    const float vz = outLayer->getPointField_float(0, "view_z");

    // Expected: normalize((-5, -3, 0))
    const double ex = -5.0, ey = -3.0, ez = 0.0;
    const double enrm = std::sqrt(ex * ex + ey * ey + ez * ez);
    ASSERT_NEAR_(vx, static_cast<float>(ex / enrm), 1e-5f);
    ASSERT_NEAR_(vy, static_cast<float>(ey / enrm), 1e-5f);
    ASSERT_NEAR_(vz, static_cast<float>(ez / enrm), 1e-5f);

    // Also verify norm = 1
    ASSERT_NEAR_(vx * vx + vy * vy + vz * vz, 1.0f, 1e-5f);

    std::cout << "test_view_vector_with_sensor_pose: ✅\n";
}

// -----------------------------------------------------------------------
// Test 4: generate_view_vector=false → no fields added
// -----------------------------------------------------------------------
void test_view_vector_disabled()
{
    const auto generators = makeGeneratorSet(false);

    auto pts = mrpt::maps::CGenericPointsMap::Create();
    pts->insertPoint(1.0f, 0.0f, 0.0f);

    const auto map = mp2p_icp_filters::apply_generators(generators, *makeObs(pts));

    auto outLayer = map.layer<mrpt::maps::CGenericPointsMap>("raw");
    ASSERT_(outLayer);
    ASSERT_EQUAL_(outLayer->size(), 1UL);

    ASSERT_(!outLayer->hasPointField("view_x"));
    ASSERT_(!outLayer->hasPointField("view_y"));
    ASSERT_(!outLayer->hasPointField("view_z"));

    std::cout << "test_view_vector_disabled: ✅\n";
}

// -----------------------------------------------------------------------
// Test 5: non-CGenericPointsMap output → silently skipped, no crash
// -----------------------------------------------------------------------
void test_view_vector_non_generic_map()
{
    // Default generator with CSimplePointsMap output (no metric_map_definition)
    constexpr const char* yaml_content = R"(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: "raw"
      generate_view_vector: true
)";

    const auto generators = mp2p_icp_filters::generators_from_yaml(
        mrpt::containers::yaml::FromText(yaml_content)["generators"]);

    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(1.0f, 0.0f, 0.0f);

    auto obs        = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud = pts;

    // Should not throw:
    const auto map = mp2p_icp_filters::apply_generators(generators, *obs);

    auto outLayer = map.layer<mrpt::maps::CPointsMap>("raw");
    ASSERT_(outLayer);
    ASSERT_EQUAL_(outLayer->size(), 1UL);

    std::cout << "test_view_vector_non_generic_map: ✅\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Run 3 rounds: with all native accelerations, with explicitly disabled AVX, then SSE2:
        for (int round = 0; round < 3; round++)
        {
            switch (round)
            {
                case 0:
                    std::cout << "=== TESTING: Native SIMD = " << mrpt::cpu::features_as_string()
                              << "\n";
                    break;
                case 1:
                    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::AVX, false);
                    std::cout << "=== TESTING: Disabling AVX\n";
                    break;
                case 2:
                    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, false);
                    std::cout << "=== TESTING: Disabling SSE2\n";
                    break;
                default:
                    break;
            };

            test_view_vector_generated_and_unit();
            test_view_vector_direction();
            test_view_vector_with_sensor_pose();
            test_view_vector_disabled();
            test_view_vector_non_generic_map();
        }

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
