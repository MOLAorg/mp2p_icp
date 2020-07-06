/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_quality_reproject_ranges.cpp
 * @brief  Unit tests for quality evaluator
 * @author Jose Luis Blanco Claraco
 * @date   Jul 06, 2020
 */

#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mrpt/containers/Parameters.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>  // fileNameStripInvalidChars()
#include <cstdlib>
#include <fstream>
#include <sstream>

const std::string datasetDir = MP2P_DATASET_DIR;

MRPT_TODO("load_xyz_file is duplicated. refactor.");

// Loads from XYZ file, possibly gz-compressed:
static mrpt::maps::CSimplePointsMap::Ptr load_xyz_file(const std::string& fil)
{
    ASSERT_FILE_EXISTS_(fil);

    mrpt::io::CFileGZInputStream f(fil);
    std::string                  buf;
    while (!f.checkEOF())
    {
        const size_t N = 10000;
        std::string  tmp;
        tmp.resize(N);
        const auto n = f.Read(&tmp[0], N);
        tmp.resize(n);
        buf += tmp;
    }

    const auto tmpFil = mrpt::system::getTempFileName();
    {
        std::ofstream fo;
        fo.open(tmpFil.c_str());
        ASSERT_(fo.is_open());
        fo << buf;
    }

    auto m = mrpt::maps::CSimplePointsMap::Create();
    m->load3D_from_text_file(tmpFil);
    ASSERT_ABOVE_(m->size(), 100U);

    return m;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        const auto inFile = std::string("happy_buddha_decim.xyz.gz");

        const auto                        fileFullPath = datasetDir + inFile;
        mrpt::maps::CSimplePointsMap::Ptr pts = load_xyz_file(fileFullPath);

        mrpt::containers::Parameters params;

        params["ncols"].asRef<uint32_t>() = 100;
        params["nrows"].asRef<uint32_t>() = 60;

        params["fx"] = 20.0;
        params["fy"] = 20.0;
        params["cx"] = 50.0;
        params["cy"] = 30.0;

        params["sigma"] = 0.1;

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

            mp2p_icp::pointcloud_t pcG;
            pcG.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] = p1;

            mp2p_icp::pointcloud_t pcL;
            pcL.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] = p2;

            const double quality = q.evaluate(pcG, pcL, relPoseTest, {});

            ASSERT_BELOW_(std::abs(quality - expectedVal), 0.1);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
