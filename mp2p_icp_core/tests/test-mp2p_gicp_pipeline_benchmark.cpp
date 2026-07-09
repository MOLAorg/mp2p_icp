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
 * @file   test-mp2p_gicp_pipeline_benchmark.cpp
 * @brief  Realistic end-to-end benchmark of the GICP pipeline used in MOLA-LO.
 *
 *  Builds two synthetic 100k-point CObservationPointCloud scans (floor + two
 *  orthogonal walls), runs the (reduced) GICP pipeline: raw generator, range
 *  filter, two FilterDecimateAdaptive stages, FilterMerge into a
 *  mola::KeyframePointCloudMap "observation" layer, then aligns against a
 *  "localmap" KeyframePointCloudMap initialized from the first scan, using
 *  Matcher_Cov2Cov + Solver_GaussNewton, and checks that the recovered SE(3)
 *  translation matches the imposed 0.3 m forward step.
 *
 *  Internal CTimeLogger of mp2p_icp::ICP and the filter pipeline is enabled
 *  and dumped to stdout to serve as a profiling reference.
 *
 *  This test requires the mola_metric_maps package; if not available at build
 *  time it is silently skipped via CMake.
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/CTimeLogger.h>

#include <cmath>
#include <iostream>
#include <sstream>

namespace
{
constexpr size_t kNumPoints    = 100000;
constexpr double kForwardStep  = 0.3;  // [m]
constexpr double kRoomHalfSize = 10.0;
constexpr double kFloorZ       = 0.0;
constexpr double kWallXatPlus  = 10.0;
constexpr double kWallYatPlus  = 10.0;

// -----------------------------------------------------------------------
// Synthetic scan: flat floor + two orthogonal walls (X=+10 and Y=+10).
// Returns a CObservationPointCloud with a CGenericPointsMap inside, with
// extra float32 fields color_{r,g,b} and t (relative timestamp).
// The scan is built in the *sensor* frame; applying the given sensorPose
// translates the observed world (sensor moves forward in +X).
// -----------------------------------------------------------------------
mrpt::obs::CObservationPointCloud::Ptr makeSyntheticScan(
    const mrpt::poses::CPose3D& sensorWorldPose, double timestampOffset, unsigned int seed)
{
    auto pts = mrpt::maps::CGenericPointsMap::Create();

    pts->registerField_float("color_r");
    pts->registerField_float("color_g");
    pts->registerField_float("color_b");
    pts->registerField_float("t");

    pts->reserve(kNumPoints);

    mrpt::random::CRandomGenerator rng;
    rng.randomize(seed);

    const size_t third = kNumPoints / 3;

    // Helper to push one point in the sensor frame, after expressing a
    // world-frame point in sensor-local coords.
    const auto pushWorldPoint = [&](double xw, double yw, double zw, double tOff)
    {
        // Transform world point into sensor local frame:
        mrpt::math::TPoint3D pLocal;
        sensorWorldPose.inverseComposePoint({xw, yw, zw}, pLocal);

        pts->insertPointFast(
            static_cast<float>(pLocal.x), static_cast<float>(pLocal.y),
            static_cast<float>(pLocal.z));
        pts->insertPointField_float("color_r", static_cast<float>(rng.drawUniform(0.0, 1.0)));
        pts->insertPointField_float("color_g", static_cast<float>(rng.drawUniform(0.0, 1.0)));
        pts->insertPointField_float("color_b", static_cast<float>(rng.drawUniform(0.0, 1.0)));
        pts->insertPointField_float("t", static_cast<float>(tOff));
    };

    // Floor: z=0, x,y in [-R, R]
    for (size_t i = 0; i < third; i++)
    {
        const double x = rng.drawUniform(-kRoomHalfSize, kRoomHalfSize);
        const double y = rng.drawUniform(-kRoomHalfSize, kRoomHalfSize);
        pushWorldPoint(x, y, kFloorZ, timestampOffset);
    }
    // Wall at x = +10, y in [-R,R], z in [0, 3]
    for (size_t i = 0; i < third; i++)
    {
        const double y = rng.drawUniform(-kRoomHalfSize, kRoomHalfSize);
        const double z = rng.drawUniform(0.0, 3.0);
        pushWorldPoint(kWallXatPlus, y, z, timestampOffset);
    }
    // Wall at y = +10, x in [-R,R], z in [0, 3]
    const size_t remaining = kNumPoints - 2 * third;
    for (size_t i = 0; i < remaining; i++)
    {
        const double x = rng.drawUniform(-kRoomHalfSize, kRoomHalfSize);
        const double z = rng.drawUniform(0.0, 3.0);
        pushWorldPoint(x, kWallYatPlus, z, timestampOffset);
    }

    auto obs         = mrpt::obs::CObservationPointCloud::Create();
    obs->pointcloud  = pts;
    obs->sensorPose  = mrpt::poses::CPose3D::Identity();  // sensor == robot for this test
    obs->timestamp   = mrpt::Clock::now();
    obs->sensorLabel = "lidar";
    return obs;
}

// -----------------------------------------------------------------------
// YAML literals: reduced from lidar3d-gicp.yaml (no deskew, no IMU).
// -----------------------------------------------------------------------
constexpr const char* kYamlObservationsGenerator = R"yaml(
- class_name: mp2p_icp_filters::Generator
  params:
    name: "Generator (raw)"
    target_layer: "raw"
    throw_on_unhandled_observation_class: true
    process_class_names_regex: ".*"
    process_sensor_labels_regex: ".*"
    metric_map_definition:
      class: mrpt::maps::CGenericPointsMap

- class_name: mp2p_icp_filters::Generator
  params:
    name: "Generator (obs)"
    target_layer: "observation"
    throw_on_unhandled_observation_class: true
    process_class_names_regex: ""
    metric_map_definition:
      class: mola::KeyframePointCloudMap
      plugin: "libmola_metric_maps.so"
      creationOpts: ~
      insertOpts: ~
      likelihoodOpts: ~
      renderOpts: ~
)yaml";

constexpr const char* kYamlLocalmapGenerator = R"yaml(
- class_name: mp2p_icp_filters::Generator
  params:
    target_layer: "localmap"
    throw_on_unhandled_observation_class: true
    process_class_names_regex: ""
    metric_map_definition:
      class: mola::KeyframePointCloudMap
      plugin: "libmola_metric_maps.so"
      creationOpts:
        max_search_keyframes: 3
        k_correspondences_for_cov: 20
        use_view_direction_filter: false
        rotation_distance_weight: 2.0
        num_diverse_keyframes: 1
      insertOpts:
        remove_frames_farther_than: 0
      likelihoodOpts: ~
      renderOpts: ~
)yaml";

constexpr const char* kYamlFilters1stPass = R"yaml(
- class_name: mp2p_icp_filters::FilterByRange
  params:
    input_pointcloud_layer: "raw"
    output_layer_between: "raw_range_filtered"
    range_min: 0.5
    range_max: 30.0
    metric_l_infinity: true

- class_name: mp2p_icp_filters::FilterDecimateAdaptive
  params:
    name: "FilterDecimateAdaptive (map)"
    input_pointcloud_layer: "raw_range_filtered"
    output_pointcloud_layer: "decimated_for_map"
    voxel_size: 0.15
    desired_output_point_count: 10000

- class_name: mp2p_icp_filters::FilterDecimateAdaptive
  params:
    name: "FilterDecimateAdaptive (icp)"
    input_pointcloud_layer: "decimated_for_map"
    output_pointcloud_layer: "decimated_for_icp"
    voxel_size: 0.10
    desired_output_point_count: 3000
)yaml";

constexpr const char* kYamlFilters2ndPass = R"yaml(
- class_name: mp2p_icp_filters::FilterMerge
  params:
    name: "FilterMerge (icp into obs)"
    input_pointcloud_layer: "decimated_for_icp"
    target_layer: "observation"
)yaml";

constexpr const char* kYamlInsertIntoLocalMap = R"yaml(
- class_name: mp2p_icp_filters::FilterMerge
  params:
    name: "FilterMerge (map into localmap)"
    input_pointcloud_layer: "decimated_for_map"
    target_layer: "localmap"
    input_layer_in_local_coordinates: true
    robot_pose: [0, 0, 0, 0, 0, 0]
)yaml";

constexpr const char* kYamlIcp = R"yaml(
class_name: mp2p_icp::ICP
params:
  maxIterations: 25
  minAbsStep_trans: 1e-3
  minAbsStep_rot: 1e-4
  covariance:
    method: Censi3D
    defaultPointSigma: 0.01
    floor_sigma_xyz: 0.001
    floor_sigma_angles_deg: 0.1
solvers:
  - class: mp2p_icp::Solver_GaussNewton
    params:
      maxIterations: 1
      robustKernel: "RobustKernel::GemanMcClure"
      robustKernelParam: 6.0
matchers:
  - class: mp2p_icp::Matcher_Cov2Cov
    params:
      threshold: 1.0
      pairingsPerPoint: 1
      allowMatchAlreadyMatchedGlobalPoints: true
      layerMatches:
        - { global: "localmap", local: "observation" }
quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params: ~
)yaml";

mrpt::containers::yaml parseSeq(const char* text)
{
    std::istringstream     iss(text);
    mrpt::containers::yaml y;
    y.loadFromStream(iss);
    return y;
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
    try
    {
        // --- Build observations ---
        const mrpt::poses::CPose3D pose0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        const mrpt::poses::CPose3D pose1(kForwardStep, 0.0, 0.0, 0.0, 0.0, 0.0);

        auto obs0 = makeSyntheticScan(pose0, 0.00, 1234);
        auto obs1 = makeSyntheticScan(pose1, 0.10, 5678);

        // --- Parse YAML blocks ---
        const auto yObsGen   = parseSeq(kYamlObservationsGenerator);
        const auto yLmGen    = parseSeq(kYamlLocalmapGenerator);
        const auto yF1       = parseSeq(kYamlFilters1stPass);
        const auto yF2       = parseSeq(kYamlFilters2ndPass);
        const auto yInsertLm = parseSeq(kYamlInsertIntoLocalMap);
        const auto yIcp      = mrpt::containers::yaml::FromText(kYamlIcp);

        // --- Build generators / filters ---
        const auto obsGens      = mp2p_icp_filters::generators_from_yaml(yObsGen);
        const auto lmGens       = mp2p_icp_filters::generators_from_yaml(yLmGen);
        const auto filtersPass1 = mp2p_icp_filters::filter_pipeline_from_yaml(yF1);
        const auto filtersPass2 = mp2p_icp_filters::filter_pipeline_from_yaml(yF2);
        const auto filtersToLm  = mp2p_icp_filters::filter_pipeline_from_yaml(yInsertLm);

        // --- Build ICP from yaml ---
        auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(yIcp);
        ASSERTMSG_(icp, "icp_pipeline_from_yaml returned null");
        icp->profiler().enable(true);

        mrpt::system::CTimeLogger pipelineProfiler(true, "GICP pipeline (test)");

        // --- Build local map from first observation ---
        mp2p_icp::metric_map_t mmLocal;
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "01.localmap.generate");
            mp2p_icp_filters::apply_generators(lmGens, *obs0, mmLocal);
        }
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "02.localmap.raw_gen");
            mp2p_icp_filters::apply_generators(obsGens, *obs0, mmLocal);
        }
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "03.localmap.filters_pass1");
            mp2p_icp_filters::apply_filter_pipeline(filtersPass1, mmLocal, pipelineProfiler);
        }
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "04.localmap.insert_into_lm");
            mp2p_icp_filters::apply_filter_pipeline(filtersToLm, mmLocal, pipelineProfiler);
        }

        // --- Build observation map for the 2nd scan ---
        mp2p_icp::metric_map_t mmObs;
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "05.obs.generate");
            mp2p_icp_filters::apply_generators(obsGens, *obs1, mmObs);
        }
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "06.obs.filters_pass1");
            mp2p_icp_filters::apply_filter_pipeline(filtersPass1, mmObs, pipelineProfiler);
        }
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "07.obs.filters_pass2");
            mp2p_icp_filters::apply_filter_pipeline(filtersPass2, mmObs, pipelineProfiler);
        }

        // --- Build a global metric_map_t that exposes the "localmap" layer ---
        mp2p_icp::metric_map_t mmGlobal;
        mmGlobal.layers["localmap"] = mmLocal.layers.at("localmap");

        // --- Run ICP ---
        mp2p_icp::Results         icpResults;
        const mrpt::math::TPose3D initialGuess(0, 0, 0, 0, 0, 0);
        {
            mrpt::system::CTimeLoggerEntry t(pipelineProfiler, "08.icp.align");
            icp->align(mmObs, mmGlobal, initialGuess, icpParams, icpResults);
        }

        const auto& T = icpResults.optimal_tf.mean;

        std::cout << "\n=== Results ===\n"
                  << "ICP iterations : " << icpResults.nIterations << "\n"
                  << "ICP quality    : " << icpResults.quality << "\n"
                  << "Recovered pose : " << T.asString() << "\n"
                  << "Expected XYZ   : (" << kForwardStep << ", 0, 0)\n";

        std::cout << "\n=== ICP internal profiler ===\n";
        icp->profiler().dumpAllStats();

        // --- Validate translation: sensor moved +0.3m in X (world frame), so
        //     the local observation expressed in localmap frame is shifted by
        //     +0.3m in +X. ICP should recover that. ---
        const double dx        = T.x() - kForwardStep;
        const double dy        = T.y();
        const double dz        = T.z();
        const double translErr = std::sqrt(dx * dx + dy * dy + dz * dz);

        std::cout << "Translation error: " << translErr << " m\n";
        ASSERT_LT_(translErr, 0.05);

        std::cout << "\n[OK] GICP pipeline benchmark test passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAIL: " << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
