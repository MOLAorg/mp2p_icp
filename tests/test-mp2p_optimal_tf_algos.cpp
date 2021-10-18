/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_icp-olae.cpp
 * @brief  Unit tests for the mp2p_icp OLAE solver
 * @author Jose Luis Blanco Claraco
 * @date   May 12, 2019
 */

#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mp2p_icp/optimal_tf_horn.h>
#include <mp2p_icp/optimal_tf_olae.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>  // fileNameStripInvalidChars()

#include <cstdlib>
#include <iostream>
#include <sstream>

// Used to validate OLAE. However, it may make the Gauss-Newton solver, or the
// robust kernel with outliers to fail.
static bool TEST_LARGE_ROTATIONS = nullptr != ::getenv("TEST_LARGE_ROTATIONS");
static bool DO_SAVE_STAT_FILES   = nullptr != ::getenv("DO_SAVE_STAT_FILES");
static bool DO_PRINT_ALL         = nullptr != ::getenv("DO_PRINT_ALL");
static bool SKIP_OUTLIERS        = nullptr != ::getenv("SKIP_OUTLIERS");
static const size_t num_reps     = mrpt::get_env<int>("NUM_REPS", 25);

using TPoints = std::vector<mrpt::math::TPoint3D>;
using TPlanes = std::vector<mp2p_icp::plane_patch_t>;

TPoints generate_points(const size_t nPts)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    TPoints pA;
    pA.resize(nPts);

    for (size_t i = 0; i < nPts; i++)
    {
        pA[i].x = rnd.drawUniform(0.0, 50.0);
        pA[i].y = rnd.drawUniform(0.0, 50.0);
        pA[i].z = rnd.drawUniform(0.0, 50.0);
    }
    return pA;
}

TPlanes generate_planes(const size_t nPlanes)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    TPlanes plA;
    plA.resize(nPlanes);

    for (size_t i = 0; i < nPlanes; i++)
    {
        plA[i].centroid.x = rnd.drawUniform(0.0, 50.0);
        plA[i].centroid.y = rnd.drawUniform(0.0, 50.0);
        plA[i].centroid.z = rnd.drawUniform(0.0, 50.0);

        auto n = mrpt::math::TVector3D(
            rnd.drawUniform(-1.0, 1.0), rnd.drawUniform(-1.0, 1.0),
            rnd.drawUniform(-1.0, 1.0));
        n *= 1.0 / n.norm();

        plA[i].plane = mrpt::math::TPlane(plA[i].centroid, n);
    }
    return plA;
}

// transform features
std::tuple<mrpt::poses::CPose3D, std::vector<std::size_t>>
    transform_points_planes(
        const TPoints& pA, TPoints& pB,
        mrpt::tfest::TMatchingPairList& pointsPairs, const TPlanes& plA,
        TPlanes& plB, std::vector<mp2p_icp::matched_plane_t>& planePairs,
        mp2p_icp::TMatchedPointPlaneList& pt2plPairs,
        const double xyz_noise_std, const double n_err_std /* normals noise*/,
        const double outliers_ratio)
{
    const double             outliers_bbox = 50.0;
    std::vector<std::size_t> gt_outlier_indices;

    auto& rnd = mrpt::random::getRandomGenerator();

    double Dx, Dy, Dz, yaw, pitch, roll;
    if (TEST_LARGE_ROTATIONS)
    {
        Dx = rnd.drawUniform(10.0, 20.0);
        Dy = rnd.drawUniform(-30.0, -20.0);
        Dz = rnd.drawUniform(5.0, 9.0);

        yaw   = mrpt::DEG2RAD(rnd.drawUniform(-180.0, 180.0));
        pitch = mrpt::DEG2RAD(rnd.drawUniform(-89.0, 89.0));
        roll  = mrpt::DEG2RAD(rnd.drawUniform(-89.0, 89.0));
    }
    else
    {
        Dx = rnd.drawUniform(-1.0, 1.0);
        Dy = rnd.drawUniform(-1.0, 1.0);
        Dz = rnd.drawUniform(-1.0, 1.0);

        yaw   = mrpt::DEG2RAD(rnd.drawUniform(-5.0, 5.0));
        pitch = mrpt::DEG2RAD(rnd.drawUniform(-5.0, 5.0));
        roll  = mrpt::DEG2RAD(rnd.drawUniform(-5.0, 5.0));
    }

    const auto pose = mrpt::poses::CPose3D(Dx, Dy, Dz, yaw, pitch, roll);
    // just the rotation, to transform vectors (vs. R^3 points):
    const auto pose_rot_only = mrpt::poses::CPose3D(0, 0, 0, yaw, pitch, roll);

    // Points:
    pB.resize(pA.size());
    for (std::size_t i = 0; i < pA.size(); ++i)
    {
        // outlier?
        const bool is_outlier = (rnd.drawUniform(0.0, 1.0) < outliers_ratio);
        if (is_outlier) gt_outlier_indices.push_back(i);

        if (!is_outlier)
        {
            // Transform + noise:
            pose.inverseComposePoint(pA[i], pB[i]);

            pB[i].x += rnd.drawGaussian1D(0, xyz_noise_std);
            pB[i].y += rnd.drawGaussian1D(0, xyz_noise_std);
            pB[i].z += rnd.drawGaussian1D(0, xyz_noise_std);
        }
        else
        {
            pB[i].x = rnd.drawUniform(.0, outliers_bbox);
            pB[i].y = rnd.drawUniform(.0, outliers_bbox);
            pB[i].z = rnd.drawUniform(.0, outliers_bbox);
        }

        // Add pairing:
        mrpt::tfest::TMatchingPair pair;
        pair.this_idx = pair.other_idx = i;

        pair.this_x = pA[i][0];
        pair.this_y = pA[i][1];
        pair.this_z = pA[i][2];

        pair.other_x = pB[i][0];
        pair.other_y = pB[i][1];
        pair.other_z = pB[i][2];

        pointsPairs.push_back(pair);
    }

    // Planes: transform + noise
    plB.resize(plA.size());
    pt2plPairs.clear();
    pt2plPairs.reserve(plA.size());

    for (std::size_t i = 0; i < plA.size(); ++i)
    {
        const bool is_outlier = (rnd.drawUniform(0.0, 1.0) < outliers_ratio);
        if (is_outlier) gt_outlier_indices.push_back(pA.size() + i);

        if (!is_outlier)
        {
            // Centroid: transform + noise
            plB[i].centroid = pose.inverseComposePoint(plA[i].centroid);
        }
        else
        {
            // Outlier
            for (int k = 0; k < 3; k++)
                plB[i].centroid[k] =
                    rnd.drawUniform(-outliers_bbox, outliers_bbox);
        }

        const auto sigma_c = xyz_noise_std;
        const auto sigma_n = n_err_std;

        plB[i].centroid.x += rnd.drawGaussian1D(0, sigma_c);
        plB[i].centroid.y += rnd.drawGaussian1D(0, sigma_c);
        plB[i].centroid.z += rnd.drawGaussian1D(0, sigma_c);

        if (!is_outlier)
        {
            // Plane: rotate + noise
            plB[i].plane = plA[i].plane;

            {
                const mrpt::math::TVector3D ug = plA[i].plane.getNormalVector();
                mrpt::math::TVector3D       ul;
                pose_rot_only.inverseComposePoint(ug, ul);

                auto& coefs = plB[i].plane.coefs;

                // Generate a random SO(3) rotation: angle \sim Gaussian(0,s_n)
                if (std::abs(sigma_n) > 1e-9)
                {
                    // unit vector at an arbitrary direction:
                    auto v = mrpt::math::TVector3D(
                        rnd.drawUniform(-1.0, 1.0), rnd.drawUniform(-1.0, 1.0),
                        rnd.drawUniform(-1.0, 1.0));
                    v *= (1.0 / v.norm());

                    const double rnd_ang = rnd.drawGaussian1D(0, sigma_n);
                    v *= rnd_ang;

                    mrpt::math::CVectorFixed<double, 3> vv;
                    for (int k = 0; k < 3; k++) vv[k] = v[k];

                    const auto R33 = mrpt::poses::Lie::SO<3>::exp(vv);
                    const auto p   = mrpt::poses::CPose3D(
                        R33, mrpt::math::CVectorFixed<double, 3>::Zero());

                    // Noisy plane normal:
                    ul = p.rotateVector(ul);
                }

                // Ax+By+Cz+D=0
                coefs[0] = ul.x;
                coefs[1] = ul.y;
                coefs[2] = ul.z;
                coefs[3] = 0;  // temporary.
                plB[i].plane.unitarize();

                coefs[3] =
                    -(coefs[0] * plB[i].centroid.x +
                      coefs[1] * plB[i].centroid.y +
                      coefs[2] * plB[i].centroid.z);
            }
        }
        else
        {
            // Outlier:
            for (int k = 0; k < 4; k++)
                plB[i].plane.coefs[k] = rnd.drawUniform(-1.0, 1.0);
            plB[i].plane.unitarize();
        }

        // Add plane-plane pairing:
        mp2p_icp::matched_plane_t pair;
        pair.p_this  = plA[i];
        pair.p_other = plB[i];
        planePairs.push_back(pair);

        // Add point-plane pairing:
        mp2p_icp::point_plane_pair_t pt2pl;
        pt2pl.pl_this  = plA[i];
        pt2pl.pt_other = mrpt::math::TPoint3Df(
            plB[i].centroid.x, plB[i].centroid.y, plB[i].centroid.z);

        pt2plPairs.push_back(pt2pl);
    }

    return {pose, gt_outlier_indices};
}

bool test_icp_algos(
    const size_t numPts, const size_t numLines, const size_t numPlanes,
    const double xyz_noise_std = .0, const double n_err_std = .0,
    bool use_robust = false, const double outliers_ratio = .0)
{
    using namespace mrpt::poses::Lie;

    MRPT_START

    const std::string tstName = mrpt::format(
        "test_icp_algos_nPt=%06u_nLin=%06u_nPl=%06u_xyzStd=%.04f_nStd=%."
        "04f_outliers=%06.03f_robust=%i",
        static_cast<unsigned int>(numPts), static_cast<unsigned int>(numLines),
        static_cast<unsigned int>(numPlanes), xyz_noise_std, n_err_std,
        outliers_ratio, use_robust ? 1 : 0);

    std::cout << "\n== [TEST] " << tstName << " =================\n";

    mrpt::system::CTimeLogger profiler;
    profiler.setMinLoggingLevel(mrpt::system::LVL_ERROR);  // to make it quiet

    // Repeat the test many times, with different random values:
    mp2p_icp::OptimalTF_Result res_olae, res_horn, res_gn;
    mrpt::poses::CPose3D       gt_pose;

    const auto max_allowed_error =
        std::min(1.0, 0.1 + 10 * xyz_noise_std + 50 * n_err_std);
    // 0.01;

    // Collect stats: columns are (see write TXT to file code at the bottom)
    mrpt::math::CMatrixDouble stats(num_reps, 1 + 3 + 3 + 3);

    double rmse_olea = 0, rmse_horn = 0, rmse_gn = 0;
    double rmse_xyz_olea = 0, rmse_xyz_horn = 0, rmse_xyz_gn = 0;

    for (size_t rep = 0; rep < num_reps; rep++)
    {
        // The input points & planes
        const TPoints pA  = generate_points(numPts);
        const TPlanes plA = generate_planes(numPlanes);

        TPoints pB;
        TPlanes plB;

        mrpt::tfest::TMatchingPairList   pointPairs;
        mp2p_icp::TMatchedPlaneList      planePairs;
        mp2p_icp::TMatchedPointPlaneList pt2plPairs;

        const auto [this_gt_pose, this_outliers] = transform_points_planes(
            pA, pB, pointPairs, plA, plB, planePairs, pt2plPairs, xyz_noise_std,
            n_err_std, outliers_ratio);

        (void)this_outliers;  // unused var

        stats(rep, 0) = SO<3>::log(gt_pose.getRotationMatrix()).norm();

#if 0
        // Also add the plane-centroid-to-plane-centroid as point-to-point
        // constraints:
        for (const auto& p : pt2plPairs)
            pointPairs.emplace_back(
                0, 0, /*indices not used here*/ p.pl_this.centroid.x,
                p.pl_this.centroid.y, p.pl_this.centroid.z, p.pt_other.x,
                p.pt_other.y, p.pt_other.z);
#endif

        // to show the result of the last iteration at end
        gt_pose = this_gt_pose;

        mp2p_icp::WeightParameters wp;

        mp2p_icp::Pairings in;
        in.paired_pt2pt = pointPairs;
        in.paired_pl2pl = planePairs;

        wp.use_scale_outlier_detector = use_robust;
        if (auto sTh = ::getenv("SCALE_OUTLIER_THRESHOLD"); sTh != nullptr)
            wp.scale_outlier_threshold = ::atof(sTh);

        // Only for tests with outliers, and if we are using small rotations,
        // use the robust kernel, using the identity SE(3) transform as
        // gross initial guess for the pose:
        if (!TEST_LARGE_ROTATIONS && outliers_ratio > 0)
        {
            wp.use_robust_kernel        = true;
            wp.currentEstimateForRobust = mrpt::poses::CPose3D::Identity();
        }

        // ========  TEST: olae_match ========
        {
            profiler.enter("olea_match");

            mp2p_icp::optimal_tf_olae(in, wp, res_olae);

            // const double dt_last =
            const auto dt_olea = profiler.leave("olea_match");

            // Collect stats:

            // Measure errors in SE(3) if we have many points, in SO(3)
            // otherwise:
            const auto pos_error = gt_pose - res_olae.optimalPose;
            const auto err_log_n =
                SO<3>::log(pos_error.getRotationMatrix()).norm();
            const auto err_xyz = pos_error.norm();

            if (DO_PRINT_ALL ||
                (outliers_ratio < 1e-5 && err_log_n > max_allowed_error))
            {
                std::cout << " -Ground_truth : " << gt_pose.asString() << "\n"
                          << " -OLAE_output  : "
                          << res_olae.optimalPose.asString() << "\n -GT_rot:\n"
                          << gt_pose.getRotationMatrix() << "\n -OLAE_rot:\n"
                          << res_olae.optimalPose.getRotationMatrix() << "\n";
                ASSERT_LT_(err_log_n, max_allowed_error);
            }

            stats(rep, 1 + 3 * 0 + 0) = err_log_n;
            stats(rep, 1 + 3 * 1 + 0) = err_xyz;
            stats(rep, 1 + 3 * 2 + 0) = dt_olea;
            rmse_olea += mrpt::square(err_log_n);
            rmse_xyz_olea += mrpt::square(err_xyz);
        }

        // ========  TEST: Classic Horn ========
        {
            profiler.enter("se3_l2");

            mp2p_icp::optimal_tf_horn(in, wp, res_horn);

            const auto dt_horn = profiler.leave("se3_l2");

            const auto pos_error = gt_pose - res_horn.optimalPose;
            const auto err_log_n =
                SO<3>::log(pos_error.getRotationMatrix()).norm();
            const auto err_xyz = pos_error.norm();

            // Don't make the tests fail if we have outliers, since it IS
            // expected that, sometimes, we don't get to the optimum
            if (DO_PRINT_ALL ||
                (outliers_ratio < 1e-5 && err_log_n > max_allowed_error))
            {
                std::cout << " -Ground_truth : " << gt_pose.asString() << "\n"
                          << " -Horn_output  : "
                          << res_horn.optimalPose.asString() << "\n -GT_rot:\n"
                          << gt_pose.getRotationMatrix() << "\n";
                ASSERT_LT_(err_log_n, max_allowed_error);
            }

            stats(rep, 1 + 3 * 0 + 1) = err_log_n;
            stats(rep, 1 + 3 * 1 + 1) = err_xyz;
            stats(rep, 1 + 3 * 2 + 1) = dt_horn;

            rmse_horn += mrpt::square(err_log_n);
            rmse_xyz_horn += mrpt::square(err_xyz);
        }

        // ========  TEST: GaussNewton method ========
        {
            profiler.enter("optimal_tf_gauss_newton");

            mp2p_icp::OptimalTF_GN_Parameters gnParams;
            gnParams.verbose = false;
            // Linearization point: Identity
            gnParams.linearizationPoint = mrpt::poses::CPose3D();

            // NOTE: this is an "unfair" comparison, since we only run ONE
            // iteration of the GN while the other methods are one-shot
            // solutions. But GN is good enough even so for small rotations to
            // solve it in just one iteration.
            mp2p_icp::optimal_tf_gauss_newton(in, res_gn, gnParams);

            const auto dt_gn = profiler.leave("optimal_tf_gauss_newton");

            const auto pos_error = gt_pose - res_gn.optimalPose;
            const auto err_log_n =
                SO<3>::log(pos_error.getRotationMatrix()).norm();
            const auto err_xyz = pos_error.norm();

            // Don't make the tests fail if we have outliers, since it IS
            // expected that, sometimes, we don't get to the optimum
            // Also, disable gauss-newton checks for large rotations, as it
            // fails, and that's expected since it's a local algorithm.
            if (0)
            /*
            !TEST_LARGE_ROTATIONS &&
            //(DO_PRINT_ALL ||
             (outliers_ratio < 1e-5 && err_log_n > max_allowed_error)))
             */
            {
                std::cout << " -Ground truth        : " << gt_pose.asString()
                          << "\n"
                          << " -GaussNewton output  : "
                          << res_gn.optimalPose.asString() << "\n -GT_rot:\n"
                          << gt_pose.getRotationMatrix() << "\n";
                ASSERT_LT_(err_log_n, max_allowed_error);
            }

            stats(rep, 1 + 3 * 0 + 2) = err_log_n;
            stats(rep, 1 + 3 * 1 + 2) = err_xyz;
            stats(rep, 1 + 3 * 2 + 2) = dt_gn;

            rmse_gn += mrpt::square(err_log_n);
            rmse_xyz_gn += mrpt::square(err_xyz);
        }
    }  // for each repetition

    // RMSE:
    rmse_olea     = std::sqrt(rmse_olea / num_reps);
    rmse_xyz_olea = std::sqrt(rmse_xyz_olea / num_reps);
    rmse_horn     = std::sqrt(rmse_horn / num_reps);
    rmse_xyz_horn = std::sqrt(rmse_xyz_horn / num_reps);
    rmse_gn       = std::sqrt(rmse_gn / num_reps);
    rmse_xyz_gn   = std::sqrt(rmse_xyz_gn / num_reps);

    const double dt_olea = profiler.getMeanTime("olea_match");
    const double dt_horn = profiler.getMeanTime("se3_l2");
    const double dt_gn   = profiler.getMeanTime("optimal_tf_gauss_newton");

    std::cout << " -Ground_truth: " << gt_pose.asString() << "\n"
              << " -OLAE output : " << res_olae.optimalPose.asString() << " ("
              << res_olae.outliers.size() << " outliers detected)\n"
              << " -Horn output : " << res_horn.optimalPose.asString() << "\n"
              << " -GN output   : " << res_gn.optimalPose.asString()
              << "\n"
              // clang-format off
              << " -OLAE        : " << mrpt::format("SO3 rmse=%e XYZ rmse=%e time=%7.03f us\n",rmse_olea, rmse_xyz_olea, dt_olea * 1e6)
              << " -Horn        : " << mrpt::format("SO3 rmse=%e XYZ rmse=%e time=%7.03f us\n",rmse_horn, rmse_xyz_horn, dt_horn * 1e6)
              << " -Gauss-Newton: " << mrpt::format("SO3 rmse=%e XYZ rmse=%e time=%7.03f us\n",rmse_gn, rmse_xyz_gn, dt_gn * 1e6);
    // clang-format on

    if (DO_SAVE_STAT_FILES)
        stats.saveToTextFile(
            mrpt::system::fileNameStripInvalidChars(tstName) +
                std::string(".txt"),
            mrpt::math::MATRIX_FORMAT_ENG, true,
            "% Columns: execution time x 3, norm(SO3_error) x3, "
            "norm(XYZ_error) x3 (OLAE, Horn, GaussNewton)\n\n");

    return true;  // all ok.
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        auto& rnd = mrpt::random::getRandomGenerator();
        rnd.randomize(1234);  // for reproducible tests

        const double nXYZ = 0.001;  // [meters] std. noise of XYZ points
        const double nN   = mrpt::DEG2RAD(0.5);  // normals noise

        // arguments: nPts, nLines, nPlanes
        // Points only. Noiseless:
        ASSERT_(test_icp_algos(3 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(test_icp_algos(4 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(test_icp_algos(10 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(test_icp_algos(100 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(test_icp_algos(1000 /*pt*/, 0 /*li*/, 0 /*pl*/));

        // Points only. Noisy:
        ASSERT_(test_icp_algos(100 /*pt*/, 0 /*li*/, 0 /*pl*/, nXYZ));
        ASSERT_(test_icp_algos(1000 /*pt*/, 0 /*li*/, 0 /*pl*/, nXYZ));

        // Planes + 1 pt. Noiseless:
        ASSERT_(test_icp_algos(2 /*pt*/, 0 /*li*/, 1 /*pl*/));
        ASSERT_(test_icp_algos(2 /*pt*/, 0 /*li*/, 10 /*pl*/));
        ASSERT_(test_icp_algos(2 /*pt*/, 0 /*li*/, 100 /*pl*/));

        // Points and planes, noisy.
        ASSERT_(test_icp_algos(2 /*pt*/, 0 /*li*/, 1 /*pl*/, nXYZ, nN));
        ASSERT_(test_icp_algos(10 /*pt*/, 0 /*li*/, 10 /*pl*/, nXYZ, nN));
        ASSERT_(test_icp_algos(100 /*pt*/, 0 /*li*/, 100 /*pl*/, nXYZ, nN));

        if (!SKIP_OUTLIERS)
        {
            // Points only. Noisy w. outliers:
            for (int robust = 0; robust <= 1; robust++)
                for (double Or = .025; Or < 0.76; Or += 0.025)
                    ASSERT_(test_icp_algos(
                        200 /*pt*/, 0, 0, nXYZ, .0, robust != 0, Or));
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
