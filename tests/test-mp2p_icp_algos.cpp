/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_icp-algos.cpp
 * @brief  Unit tests for mp2p_icp solvers
 * @author Jose Luis Blanco Claraco
 * @date   May 12, 2019
 */

#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mp2p_icp/Matcher_Point2Plane.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/Solver_Horn.h>
#include <mp2p_icp/Solver_OLAE.h>
#include <mp2p_icp/load_xyz_file.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <Eigen/Dense>
#include <iostream>

static int  NUM_REPS = mrpt::get_env<int>("NUM_REPS", 3);
static bool DO_SAVE_STAT_FILES =
    mrpt::get_env<bool>("DO_SAVE_STAT_FILES", false);
static bool DO_PRINT_ALL = mrpt::get_env<bool>("DO_PRINT_ALL", false);

const std::string datasetDir = MP2P_DATASET_DIR;

static void test_icp(
    const std::string& inFile, const std::string& icpClassName,
    const std::string& solverName, const std::string& matcherName,
    int pointDecimation)
{
    using namespace mrpt::poses::Lie;

    const auto fileFullPath = datasetDir + inFile;

    mrpt::maps::CSimplePointsMap::Ptr pts =
        mp2p_icp::load_xyz_file(fileFullPath);

    if (pointDecimation > 1)
    {
        auto decimPts = mrpt::maps::CSimplePointsMap::Create();
        for (size_t i = 0; i < pts->size(); i += pointDecimation)
        {
            float x, y, z;
            pts->getPoint(i, x, y, z);
            decimPts->insertPoint(x, y, z);
        }
        pts = decimPts;
    }

    std::cout << "Running " << icpClassName << "|" << solverName << "|"
              << matcherName << " test on: " << inFile << " with "
              << pts->size() << " points\n";

    double outliers_ratio = 0;
    bool   use_robust     = true;

    const std::string tstName = mrpt::format(
        "test_icp_Model=%s_Algo=%s_%s_%s_outliers=%06.03f_robust=%i",
        inFile.c_str(), icpClassName.c_str(), solverName.c_str(),
        matcherName.c_str(), outliers_ratio, use_robust ? 1 : 0);

    const auto bbox      = pts->boundingBox();
    const auto bbox_size = bbox.max - bbox.min;

    const double max_dim = mrpt::max3(bbox_size.x, bbox_size.y, bbox_size.z);

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
        pts_reg->changeCoordinatesReference(*pts, -gt_pose);

        // Point clouds: reference and modified:
        mp2p_icp::metric_map_t pc_ref, pc_mod;
        pc_ref.layers["raw"] = pts;
        pc_mod.layers["raw"] = pts_reg;

        const auto init_guess = mrpt::math::TPose3D::Identity();

        mp2p_icp::ICP::Ptr icp = std::dynamic_pointer_cast<mp2p_icp::ICP>(
            mrpt::rtti::classFactory(icpClassName));

        if (!icp)
            THROW_EXCEPTION_FMT(
                "Could not create object of type `%s`, is it registered?",
                icpClassName.c_str());

        // Initialize solvers:
        if (!solverName.empty())
        {
            mp2p_icp::Solver::Ptr solver =
                std::dynamic_pointer_cast<mp2p_icp::Solver>(
                    mrpt::rtti::classFactory(solverName));

            if (!solver)
                THROW_EXCEPTION_FMT(
                    "Could not create Solver of type `%s`, is it registered?",
                    solverName.c_str());

            icp->solvers().clear();
            icp->solvers().push_back(solver);
        }

        // Initialize matchers:
        if (!matcherName.empty())
        {
            mp2p_icp::Matcher::Ptr matcher =
                std::dynamic_pointer_cast<mp2p_icp::Matcher>(
                    mrpt::rtti::classFactory(matcherName));

            if (!matcher)
                THROW_EXCEPTION_FMT(
                    "Could not create Matcher of type `%s`, is it registered?",
                    matcherName.c_str());

            icp->matchers().push_back(matcher);
        }

        // Special parameters:
        if (!icp->matchers().empty())
        {
            if (auto m = std::dynamic_pointer_cast<
                    mp2p_icp::Matcher_Points_DistanceThreshold>(
                    icp->matchers().at(0));
                m)
            {
                mrpt::containers::yaml ps;
                ps["threshold"] = 0.15 * max_dim;
                m->initialize(ps);
            }

            if (auto m =
                    std::dynamic_pointer_cast<mp2p_icp::Matcher_Point2Plane>(
                        icp->matchers().at(0));
                m)
            {
                mrpt::containers::yaml ps;
                ps["distanceThreshold"]   = 0.15 * max_dim;
                ps["planeEigenThreshold"] = 10.0;
                ps["minimumPlanePoints"]  = 5;
                ps["knn"]                 = 5;

                m->initialize(ps);
            }
        }

        // ICP test itself:
        mp2p_icp::Parameters icp_params;
        mp2p_icp::Results    icp_results;

        icp_params.maxIterations = 100;

        timer.Tic();

        icp->align(pc_mod, pc_ref, init_guess, icp_params, icp_results);

        const double dt = timer.Tac();

        const auto pos_error = gt_pose - icp_results.optimal_tf.mean;
        const auto err_log_n = SO<3>::log(pos_error.getRotationMatrix()).norm();
        const auto err_xyz   = pos_error.norm();
        const auto err_se3   = SE<3>::log(pos_error).norm();

        ASSERT_LT_(err_se3, 0.1);

        stats(rep, 2 + 0) = err_log_n;
        stats(rep, 2 + 1) = err_xyz;
        stats(rep, 2 + 2) = dt;

        if (DO_PRINT_ALL)
        {
            std::cout << "GT pose       : " << gt_pose.asString() << "\n";
            std::cout << "ICP pose      : "
                      << icp_results.optimal_tf.mean.asString() << "\n";
            std::cout << "Error SE(3)   : " << err_se3 << "\n";
            std::cout << "ICP pose stddev: "
                      << icp_results.optimal_tf.cov.asEigen()
                             .diagonal()
                             .array()
                             .sqrt()
                             .matrix()
                             .transpose()
                      << "\n";
            std::cout << "ICP quality    : " << icp_results.quality << "\n";
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

        using lst_algos_t = std::vector<std::tuple<
            const char*, const char*, const char*, int /*decimation*/>>;
        // clang-format off
        lst_algos_t lst_algos = {
             {"mp2p_icp::ICP", "mp2p_icp::Solver_Horn",        "mp2p_icp::Matcher_Points_DistanceThreshold", 1},
             {"mp2p_icp::ICP", "mp2p_icp::Solver_Horn",        "mp2p_icp::Matcher_Points_InlierRatio", 10},
             // Horn method cannot handle pt-to-plane.
             //{"mp2p_icp::ICP", "mp2p_icp::Solver_Horn",        "mp2p_icp::Matcher_Point2Plane", 10},
             
             
             {"mp2p_icp::ICP", "mp2p_icp::Solver_OLAE",        "mp2p_icp::Matcher_Points_DistanceThreshold", 10},
             {"mp2p_icp::ICP", "mp2p_icp::Solver_OLAE",        "mp2p_icp::Matcher_Points_InlierRatio", 10},
             // OLAE method cannot handle pt-to-plane only.
             //{"mp2p_icp::ICP", "mp2p_icp::Solver_OLAE",        "mp2p_icp::Matcher_Point2Plane", 10},
             
             {"mp2p_icp::ICP", "mp2p_icp::Solver_GaussNewton", "mp2p_icp::Matcher_Points_DistanceThreshold", 10},
             {"mp2p_icp::ICP", "mp2p_icp::Solver_GaussNewton", "mp2p_icp::Matcher_Points_InlierRatio", 10},
             {"mp2p_icp::ICP", "mp2p_icp::Solver_GaussNewton", "mp2p_icp::Matcher_Point2Plane", 10},
             };
        // clang-format on

        // Optional methods:
        if (mp2p_icp::ICP_LibPointmatcher::methodAvailable())
            lst_algos.push_back({"mp2p_icp::ICP_LibPointmatcher", "", "", 1});

        for (const auto& algo : lst_algos)
            for (const auto& fil : lst_files)
                test_icp(
                    fil, std::get<0>(algo), std::get<1>(algo),
                    std::get<2>(algo), std::get<3>(algo));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
