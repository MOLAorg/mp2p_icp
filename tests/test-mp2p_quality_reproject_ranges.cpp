/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_quality_reproject_ranges.cpp
 * @brief  Unit tests for quality evaluator
 * @author Jose Luis Blanco Claraco
 * @date   Jul 06, 2020
 */

#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mp2p_icp/load_xyz_file.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cstdlib>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    const std::string datasetDir = MP2P_DATASET_DIR;

    try
    {
        const auto inFile = std::string("happy_buddha_decim.xyz.gz");

        const auto                        fileFullPath = datasetDir + inFile;
        mrpt::maps::CSimplePointsMap::Ptr pts =
            mp2p_icp::load_xyz_file(fileFullPath);

        mrpt::containers::yaml params;

        params["ncols"] = 100;
        params["nrows"] = 60;

        params["fx"] = 20.0;
        params["fy"] = 20.0;
        params["cx"] = 50.0;
        params["cy"] = 30.0;

        params["sigma"] = 0.1;

        // params["debug_save_all_matrices"] = true;

        mp2p_icp::QualityEvaluator_RangeImageSimilarity q;
        q.initialize(params);

        {
            const auto viewPose = mrpt::poses::CPose3D(-0.2, 0, 0, 0, 0, 0);
            pts->changeCoordinatesReference(viewPose);
        }

        // Pairs: ground-truth transformation (xyz yaw pitch roll) + test pose:
        std::vector<
            std::tuple<mrpt::poses::CPose3D, mrpt::poses::CPose3D, double>>
            testPairs = {
                // #1:
                {{.0, .0, .0, .0, .0, .0}, {.0, .0, .0, .0, .0, .0}, 1.0},
                // #1:
                {{1.0, 1.0, .0, .0, .0, .0}, {1.0, 1.0, .0, .0, .0, .0}, 1.0},
                // #2:
                {{.1, .1, .2, .0, .0, .0}, {.101, .1, .2, .0, .0, .0}, 0.93},
            };

        // Test 1: quality for identity pose:
        for (const auto& p : testPairs)
        {
            const mrpt::poses::CPose3D& relPoseGT   = std::get<0>(p);
            const mrpt::poses::CPose3D& relPoseTest = std::get<1>(p);
            const double                expectedVal = std::get<2>(p);

            auto p1 = mrpt::maps::CSimplePointsMap::Create();
            p1->insertAnotherMap(pts.get(), mrpt::poses::CPose3D::Identity());

            auto p2 = mrpt::maps::CSimplePointsMap::Create();
            p2->insertAnotherMap(pts.get(), relPoseGT);

            mp2p_icp::metric_map_t pcG;
            pcG.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = p1;

            mp2p_icp::metric_map_t pcL;
            pcL.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = p2;

            const auto   res     = q.evaluate(pcG, pcL, relPoseTest, {});
            const double quality = res.quality;

            if (std::abs(quality - expectedVal) > 0.1)
            {
                std::cerr << "Failed for test case:\n"
                          << " relPoseGT   : " << relPoseGT << "\n"
                          << " relPoseTest : " << relPoseTest << "\n"
                          << " expectedVal : " << expectedVal << "\n"
                          << " quality     : " << quality << "\n";

                throw std::runtime_error("test failed (see cerr above)");
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
