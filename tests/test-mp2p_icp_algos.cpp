/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_icp-olae.cpp
 * @brief  Unit tests for the mp2p_icp OLAE solver
 * @author Jose Luis Blanco Claraco
 * @date   May 12, 2019
 */

#include <mp2p_icp/ICP_GaussNewton.h>
#include <mp2p_icp/ICP_Horn_MultiCloud.h>
#include <mp2p_icp/ICP_OLAE.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <fstream>
#include <iostream>

// Used to validate OLAE. However, it may make the Gauss-Newton solver, or the
// robust kernel with outliers to fail.
static double XYZ_RANGE = mrpt::get_env<double>("XYZ_RANGE", 5.0);
static int    NUM_REPS  = mrpt::get_env<int>("NUM_REPS", 10);
static bool   DO_SAVE_STAT_FILES =
    mrpt::get_env<bool>("DO_SAVE_STAT_FILES", false);
static bool DO_PRINT_ALL = mrpt::get_env<bool>("DO_PRINT_ALL", false);

const std::string datasetDir = MP2P_DATASET_DIR;

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

static void test_icp(const std::string& inFile, const std::string& algoName)
{
    using namespace mrpt::poses::Lie;

    const auto fileFullPath = datasetDir + inFile;

    const mrpt::maps::CSimplePointsMap::Ptr pts = load_xyz_file(fileFullPath);

    std::cout << "Running " << algoName << " test on: " << inFile << " with "
              << pts->size() << " points\n";

    double outliers_ratio = 0;
    bool   use_robust     = true;

    const std::string tstName = mrpt::format(
        "test_icp_Model=%s_Algo=%s_outliers=%6.03f_robust=%i", inFile.c_str(),
        algoName.c_str(), outliers_ratio, use_robust ? 1 : 0);

    mrpt::math::TPoint3D bbox_min, bbox_max;
    pts->boundingBox(bbox_min, bbox_max);
    const auto   bbox_size = bbox_max - bbox_min;
    const double max_dim   = mrpt::max3(bbox_size.x, bbox_size.y, bbox_size.z);

    const double f = 0.25;

    mrpt::system::CTicTac timer;

    // Collect stats: columns are (see write TXT to file code at the bottom)
    mrpt::math::CMatrixDouble stats(NUM_REPS, 2 + 2 + 1);

    for (int rep = 0; rep < NUM_REPS; rep++)
    {
        // Create transformed (and corrupted?) point cloud:
        auto&        rnd   = mrpt::random::getRandomGenerator();
        const double Dx    = rnd.drawUniform(-f * bbox_size.x, f * bbox_size.x);
        const double Dy    = rnd.drawUniform(-f * bbox_size.y, f * bbox_size.y);
        const double Dz    = rnd.drawUniform(-f * bbox_size.z, f * bbox_size.z);
        const double yaw   = mrpt::DEG2RAD(rnd.drawUniform(-20.0, 20.0));
        const double pitch = mrpt::DEG2RAD(rnd.drawUniform(-20.0, 20.0));
        const double roll  = mrpt::DEG2RAD(rnd.drawUniform(-20.0, 20.0));

        const auto gt_pose = mrpt::poses::CPose3D(Dx, Dy, Dz, yaw, pitch, roll);

        stats(rep, 0) = SO<3>::log(gt_pose.getRotationMatrix()).norm();
        stats(rep, 1) = gt_pose.norm();

        auto pts_reg = mrpt::maps::CSimplePointsMap::Create();
        pts_reg->changeCoordinatesReference(*pts, gt_pose);

        // Point clouds: reference and modified:
        mp2p_icp::pointcloud_t pc_ref, pc_mod;
        pc_ref.point_layers["raw"] = pts;
        pc_mod.point_layers["raw"] = pts_reg;

        const auto init_guess = mrpt::math::TPose3D::Identity();

        mp2p_icp::ICP_Base::Ptr icp =
            std::dynamic_pointer_cast<mp2p_icp::ICP_Base>(
                mrpt::rtti::classFactory(algoName));

        if (!icp)
            THROW_EXCEPTION_FMT(
                "Could not create object of type `%s`, is it registered?",
                algoName.c_str());

        mp2p_icp::Parameters icp_params;
        mp2p_icp::Results    icp_results;

        icp_params.maxIterations = 100;
        icp_params.thresholdDist = 0.15 * max_dim;

        // Important, only the layers in this list will be aligned for pt-to-pt:
        icp_params.weight_pt2pt_layers["raw"] = 1.0;

        timer.Tic();

        icp->align(pc_mod, pc_ref, init_guess, icp_params, icp_results);

        const double dt = timer.Tac();

        const auto pos_error = gt_pose - icp_results.optimal_tf.mean;
        const auto err_log_n = SO<3>::log(pos_error.getRotationMatrix()).norm();
        const auto err_xyz   = pos_error.norm();

        stats(rep, 2 + 0) = err_log_n;
        stats(rep, 2 + 1) = err_xyz;
        stats(rep, 2 + 2) = dt;

        if (DO_PRINT_ALL)
        {
            std::cout << "GT pose       : " << gt_pose.asString() << "\n";
            std::cout << "ICP pose       : "
                      << icp_results.optimal_tf.mean.asString() << "\n";
            std::cout << "ICP goodness   : " << icp_results.goodness << "\n";
            std::cout << "ICP iterations : " << icp_results.nIterations << "\n";
        }

    }  // for reps

    if (DO_SAVE_STAT_FILES)
        stats.saveToTextFile(
            mrpt::system::fileNameStripInvalidChars(tstName) +
                std::string(".txt"),
            mrpt::math::MATRIX_FORMAT_ENG, true,
            "% Columns: norm GT_rot, norm_GT_XYZ, norm(SO3_error) "
            "norm(XYZ_error) icp_time\n\n");
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        auto& rnd = mrpt::random::getRandomGenerator();
        rnd.randomize(1234);  // for reproducible tests

        const std::vector<const char*> lst_files{
            {"bunny_decim.xyz.gz", "happy_buddha_decim.xyz.gz"}};

        const std::vector<const char*> lst_algos{
            {"mp2p_icp::ICP_Horn_MultiCloud", "mp2p_icp::ICP_OLAE",
             "mp2p_icp::ICP_GaussNewton"}};

        for (const auto& algo : lst_algos)
            for (const auto& fil : lst_files) test_icp(fil, algo);
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
