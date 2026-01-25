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
 * @file   test-mp2p_WeightParameters.cpp
 * @brief  Unit test for WeightParameters
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/WeightParameters.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{
void test_default_construction()
{
    WeightParameters wp;

    ASSERT_EQUAL_(wp.use_scale_outlier_detector, false);
    ASSERT_NEAR_(wp.scale_outlier_threshold, 1.20, 1e-6);
    ASSERT_(wp.robust_kernel == RobustKernel::None);
    ASSERT_NEAR_(wp.robust_kernel_param, 1.0, 1e-6);
    ASSERT_(!wp.currentEstimateForRobust.has_value());

    // Check default pair weights
    ASSERT_EQUAL_(wp.pair_weights.pt2pt, 1.0);
    ASSERT_EQUAL_(wp.pair_weights.pt2ln, 1.0);
    ASSERT_EQUAL_(wp.pair_weights.pt2pl, 1.0);
}

void test_load_from_yaml()
{
    mrpt::containers::yaml cfg;
    cfg["use_scale_outlier_detector"] = true;
    cfg["scale_outlier_threshold"]    = 1.5;
    cfg["robust_kernel"]              = "RobustKernel::Cauchy";
    cfg["robust_kernel_param"]        = 2.0;

    // Add pair weights
    cfg["pair_weights"]["pt2pt"] = 2.0;
    cfg["pair_weights"]["pt2ln"] = 3.0;
    cfg["pair_weights"]["pt2pl"] = 4.0;
    cfg["pair_weights"]["ln2ln"] = 5.0;
    cfg["pair_weights"]["pl2pl"] = 6.0;

    WeightParameters wp;
    wp.load_from(cfg);

    ASSERT_EQUAL_(wp.use_scale_outlier_detector, true);
    ASSERT_NEAR_(wp.scale_outlier_threshold, 1.5, 1e-6);
    ASSERT_(wp.robust_kernel == RobustKernel::Cauchy);
    ASSERT_NEAR_(wp.robust_kernel_param, 2.0, 1e-6);

    ASSERT_EQUAL_(wp.pair_weights.pt2pt, 2.0);
    ASSERT_EQUAL_(wp.pair_weights.pt2ln, 3.0);
    ASSERT_EQUAL_(wp.pair_weights.pt2pl, 4.0);
    ASSERT_EQUAL_(wp.pair_weights.ln2ln, 5.0);
    ASSERT_EQUAL_(wp.pair_weights.pl2pl, 6.0);
}

void test_save_to_yaml()
{
    WeightParameters wp;
    wp.use_scale_outlier_detector = true;
    wp.scale_outlier_threshold    = 1.8;
    wp.robust_kernel              = RobustKernel::GemanMcClure;
    wp.robust_kernel_param        = 3.5;

    wp.pair_weights.pt2pt = 1.5;
    wp.pair_weights.pt2ln = 2.5;
    wp.pair_weights.pt2pl = 3.5;

    mrpt::containers::yaml cfg;
    wp.save_to(cfg);

    ASSERT_EQUAL_(cfg["use_scale_outlier_detector"].as<bool>(), true);
    ASSERT_NEAR_(cfg["scale_outlier_threshold"].as<double>(), 1.8, 1e-6);
    ASSERT_EQUAL_(cfg["robust_kernel"].as<std::string>(), "RobustKernel::GemanMcClure");
    ASSERT_NEAR_(cfg["robust_kernel_param"].as<double>(), 3.5, 1e-6);
}

void test_serialization()
{
    WeightParameters wp1;
    wp1.use_scale_outlier_detector = true;
    wp1.scale_outlier_threshold    = 1.3;
    wp1.robust_kernel              = RobustKernel::Cauchy;
    wp1.robust_kernel_param        = 1.345;
    wp1.pair_weights.pt2pt         = 2.0;
    wp1.pair_weights.pt2ln         = 3.0;

    // Serialize
    mrpt::io::CMemoryStream buf;
    auto                    arch_out = mrpt::serialization::archiveFrom(buf);
    arch_out << wp1;

    // Deserialize
    buf.Seek(0);
    auto             arch_in = mrpt::serialization::archiveFrom(buf);
    WeightParameters wp2;
    arch_in >> wp2;

    ASSERT_EQUAL_(wp2.use_scale_outlier_detector, true);
    ASSERT_NEAR_(wp2.scale_outlier_threshold, 1.3, 1e-6);
    ASSERT_(wp2.robust_kernel == RobustKernel::Cauchy);
    ASSERT_NEAR_(wp2.robust_kernel_param, 1.345, 1e-6);
    ASSERT_EQUAL_(wp2.pair_weights.pt2pt, 2.0);
    ASSERT_EQUAL_(wp2.pair_weights.pt2ln, 3.0);
}

void test_robust_kernels()
{
    // Test all robust kernel types
    std::vector<std::pair<std::string, RobustKernel>> kernels = {
        {"RobustKernel::None", RobustKernel::None},
        {"RobustKernel::Cauchy", RobustKernel::Cauchy},
        {"RobustKernel::GemanMcClure", RobustKernel::GemanMcClure},
    };

    for (const auto& [name, kernel] : kernels)
    {
        mrpt::containers::yaml cfg;
        cfg["robust_kernel"] = name;

        WeightParameters wp;
        wp.load_from(cfg);

        ASSERT_(wp.robust_kernel == kernel);
    }
}

void test_current_estimate()
{
    WeightParameters wp;

    // Initially should not have a value
    ASSERT_(!wp.currentEstimateForRobust.has_value());

    // Set a pose
    mrpt::poses::CPose3D pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    wp.currentEstimateForRobust = pose;

    ASSERT_(wp.currentEstimateForRobust.has_value());

    const auto& storedPose = wp.currentEstimateForRobust.value();
    ASSERT_NEAR_(storedPose.x(), 1.0, 1e-6);
    ASSERT_NEAR_(storedPose.y(), 2.0, 1e-6);
    ASSERT_NEAR_(storedPose.z(), 3.0, 1e-6);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_default_construction();
        std::cout << "test_default_construction: Success ✅" << std::endl;

        test_load_from_yaml();
        std::cout << "test_load_from_yaml: Success ✅" << std::endl;

        test_save_to_yaml();
        std::cout << "test_save_to_yaml: Success ✅" << std::endl;

        test_serialization();
        std::cout << "test_serialization: Success ✅" << std::endl;

        test_robust_kernels();
        std::cout << "test_robust_kernels: Success ✅" << std::endl;

        test_current_estimate();
        std::cout << "test_current_estimate: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
