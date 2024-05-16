/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_quality_voxels.cpp
 * @brief  Unit tests for quality evaluator
 * @author Jose Luis Blanco Claraco
 * @date   May 16, 2024
 */

#include <mp2p_icp/QualityEvaluator_Voxels.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>  // pathJoin()

void unit_test()
{
    const std::string datasetDir =
        mrpt::system::pathJoin({MOLA_TEST_DATASET_DIR, "loop_closures"});

    ASSERT_DIRECTORY_EXISTS_(datasetDir);

    const std::string datasetListFile =
        mrpt::system::pathJoin({datasetDir, "dataset.yaml"});

    const auto dataset = mrpt::containers::yaml::FromFile(datasetListFile);

    const auto badOnes  = dataset["bad"];
    const auto goodOnes = dataset["good"];

    ASSERT_(badOnes.isSequence());
    ASSERT_(goodOnes.isSequence());

    /// Initialize quality evaluator module:
    mrpt::containers::yaml params;
    params["voxel_layer_name"]   = "localmap_voxels";
    params["dist2quality_scale"] = 0.1;

    mp2p_icp::QualityEvaluator_Voxels q;
    q.initialize(params);

    struct Entry
    {
        std::string          global, local;
        mrpt::poses::CPose3D local_pose_wrt_global;
        bool                 is_good_lc;
    };

    std::vector<Entry> LCs;

    auto lambdaProcessYaml =
        [&](const mrpt::containers::yaml::sequence_t& seq, bool are_good_lcs)
    {
        for (const auto& p : seq)
        {
            auto& lc      = LCs.emplace_back();
            lc.is_good_lc = are_good_lcs;
            lc.global     = p.asMap().at("global").as<std::string>();
            lc.local      = p.asMap().at("local").as<std::string>();
            const auto v  = p.asMap().at("final_pose").asSequence();
            ASSERT_EQUAL_(v.size(), 6UL);
            lc.local_pose_wrt_global =
                mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                    v[0].as<double>(), v[1].as<double>(), v[2].as<double>(),
                    mrpt::DEG2RAD(v[3].as<double>()),
                    mrpt::DEG2RAD(v[4].as<double>()),
                    mrpt::DEG2RAD(v[5].as<double>()));
        }
    };

    // Load GOOD loop-closures:
    lambdaProcessYaml(goodOnes.asSequenceRange(), true);

    // Load BAD loop-closures:
    lambdaProcessYaml(badOnes.asSequenceRange(), false);

    for (const auto& e : LCs)
    {
        mp2p_icp::metric_map_t pcG;
        pcG.load_from_file(mrpt::system::pathJoin({datasetDir, e.global}));

        mp2p_icp::metric_map_t pcL;
        pcL.load_from_file(mrpt::system::pathJoin({datasetDir, e.local}));

        const double quality =
            q.evaluate(pcG, pcL, e.local_pose_wrt_global, {});

        std::cout << "global: " << e.global << "\n"
                  << "local: " << e.local << "\n"
                  << "is_good: " << e.is_good_lc << "\n"
                  << "result_quality: " << quality << "\n";

#if 0
if (std::abs(quality - expectedVal) > 0.1)
        {
            std::cerr << "Failed for test case:\n"
                      << " relPoseGT   : " << relPoseGT << "\n"
                      << " relPoseTest : " << relPoseTest << "\n"
                      << " expectedVal : " << expectedVal << "\n"
                      << " quality     : " << quality << "\n";
            throw std::runtime_error("test failed (see cerr above)");
        }
#endif
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
