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
 * @file   test-mp2p_pipeline_from_yaml.cpp
 * @brief  Parses complete ICP pipelines from embedded YAML literals and
 *         verifies the resulting ICP object + Parameters (with emphasis on
 *         the new covariance.* fields).
 * @author Jose Luis Blanco Claraco
 * @date   Apr 30, 2026
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/covariance.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>

#include <iostream>
#include <sstream>

namespace
{
constexpr const char* kYamlMinimal = R"yaml(
class_name: mp2p_icp::ICP

params:
  maxIterations: 50
  minAbsStep_trans: 1e-4
  minAbsStep_rot:   1e-4

solvers:
  - class: mp2p_icp::Solver_Horn
    params: ~

matchers:
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 0.15
      thresholdAngularDeg: 0

quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params: ~
)yaml";

constexpr const char* kYamlWithCovariance = R"yaml(
class_name: mp2p_icp::ICP

params:
  maxIterations: 75
  minAbsStep_trans: 5e-4
  minAbsStep_rot:   1e-4

  covariance:
    method: Censi3D
    defaultPointSigma: 0.05
    floor_sigma_xyz: 0.01
    floor_sigma_angles_deg: 0.5

solvers:
  - class: mp2p_icp::Solver_Horn
    params: ~

matchers:
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 0.20
      thresholdAngularDeg: 0

quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params: ~
)yaml";

constexpr const char* kYamlBadCovarianceMethod = R"yaml(
class_name: mp2p_icp::ICP

params:
  maxIterations: 10
  covariance:
    method: NotARealMethod

solvers:
  - class: mp2p_icp::Solver_Horn
    params: ~

matchers:
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 0.10

quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params: ~
)yaml";

mrpt::containers::yaml parse(const char* text)
{
    std::istringstream     iss(text);
    mrpt::containers::yaml y;
    y.loadFromStream(iss);
    return y;
}

// ---------------------------------------------------------------------------
void test_minimal_pipeline()
{
    std::cout << "Test 1 - parse minimal ICP pipeline (no covariance block) ... ";

    const auto y       = parse(kYamlMinimal);
    auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(y);

    ASSERTMSG_(icp, "Expected a non-null ICP::Ptr");
    ASSERT_EQUAL_(params.maxIterations, 50U);
    ASSERT_NEAR_(params.minAbsStep_trans, 1e-4, 1e-15);

    // Defaults for the (absent) covariance block
    ASSERT_(
        params.covariance_params.method == mp2p_icp::CovarianceParameters::Method::InverseHessian);
    ASSERT_NEAR_(params.covariance_params.floor_sigma_xyz, 0.0, 1e-15);
    ASSERT_NEAR_(params.covariance_params.floor_sigma_angles, 0.0, 1e-15);

    std::cout << "OK\n";
}

// ---------------------------------------------------------------------------
void test_pipeline_with_covariance()
{
    std::cout << "Test 2 - parse pipeline with a Censi3D covariance block ... ";

    const auto y       = parse(kYamlWithCovariance);
    auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(y);

    ASSERTMSG_(icp, "Expected a non-null ICP::Ptr");
    ASSERT_EQUAL_(params.maxIterations, 75U);

    const auto& cp = params.covariance_params;
    ASSERT_(cp.method == mp2p_icp::CovarianceParameters::Method::Censi3D);
    ASSERT_NEAR_(cp.defaultPointSigma, 0.05, 1e-12);
    ASSERT_NEAR_(cp.floor_sigma_xyz, 0.01, 1e-12);
    // floor_sigma_angles_deg: 0.5 -> 0.5 * pi/180 rad
    ASSERT_NEAR_(cp.floor_sigma_angles, 0.5 * M_PI / 180.0, 1e-12);

    std::cout << "OK\n";
}

// ---------------------------------------------------------------------------
void test_pipeline_bad_covariance_method_throws()
{
    std::cout << "Test 3 - bad covariance.method raises an exception ... ";

    const auto y     = parse(kYamlBadCovarianceMethod);
    bool       threw = false;
    try
    {
        (void)mp2p_icp::icp_pipeline_from_yaml(y);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERTMSG_(threw, "Expected an exception for unknown covariance.method value");

    std::cout << "OK\n";
}

// ---------------------------------------------------------------------------
void test_covariance_params_save_load_roundtrip()
{
    std::cout << "Test 4 - CovarianceParameters save_to / load_from round-trip ... ";

    mp2p_icp::CovarianceParameters in;
    in.method             = mp2p_icp::CovarianceParameters::Method::Censi3D;
    in.defaultPointSigma  = 0.123;
    in.floor_sigma_xyz    = 0.05;
    in.floor_sigma_angles = 0.01;
    in.finDif_xyz         = 1e-6;
    in.finDif_angles      = 1e-5;

    mrpt::containers::yaml y = mrpt::containers::yaml::Map();
    in.save_to(y);

    mp2p_icp::CovarianceParameters out;
    out.load_from(y);

    ASSERT_(out.method == in.method);
    ASSERT_NEAR_(out.defaultPointSigma, in.defaultPointSigma, 1e-12);
    ASSERT_NEAR_(out.floor_sigma_xyz, in.floor_sigma_xyz, 1e-12);
    ASSERT_NEAR_(out.floor_sigma_angles, in.floor_sigma_angles, 1e-12);
    ASSERT_NEAR_(out.finDif_xyz, in.finDif_xyz, 1e-15);
    ASSERT_NEAR_(out.finDif_angles, in.finDif_angles, 1e-15);

    std::cout << "OK\n";
}

}  // namespace

// ---------------------------------------------------------------------------
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_minimal_pipeline();
        test_pipeline_with_covariance();
        test_pipeline_bad_covariance_method_throws();
        test_covariance_params_save_load_roundtrip();
        std::cout << "\nAll pipeline-from-yaml tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}
