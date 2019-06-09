/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-test-mp2_icp-olae.cpp
 * @brief  Unit tests for the mp2_icp OLAE solver
 * @author Jose Luis Blanco Claraco
 * @date   May 12, 2019
 */

#include <mp2_icp/OLAE_ICP.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <sstream>

// Used to validate OLEA. May make the Gauss-Newton solver to fail, in turn.
//#define TEST_LARGE_ROTATIONS

using TPoints = std::vector<mrpt::math::TPoint3D>;
using TPlanes = std::vector<mp2_icp::OLAE_ICP::plane_patch_t>;

TPoints generate_points(const size_t nPts)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    TPoints pA;
    pA.resize(nPts);

    for (size_t i = 0; i < nPts; i++)
    {
        pA[i].x = rnd.drawUniform(-50.0, 50.0);
        pA[i].y = rnd.drawUniform(-50.0, 50.0);
        pA[i].z = rnd.drawUniform(-50.0, 50.0);
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
        plA[i].centroid.x = rnd.drawUniform(-50.0, 50.0);
        plA[i].centroid.y = rnd.drawUniform(-50.0, 50.0);
        plA[i].centroid.z = rnd.drawUniform(-50.0, 50.0);

        auto n = mrpt::math::TVector3D(
            rnd.drawUniform(-1.0, 1.0), rnd.drawUniform(-1.0, 1.0),
            rnd.drawUniform(-1.0, 1.0));
        n *= 1.0 / n.norm();

        plA[i].plane = mrpt::math::TPlane(plA[i].centroid, n);
    }
    return plA;
}

// transform features
mrpt::poses::CPose3D transform_points_planes(
    const TPoints& pA, TPoints& pB, mrpt::tfest::TMatchingPairList& pointsPairs,
    const TPlanes& plA, TPlanes& plB,
    std::vector<mp2_icp::OLAE_ICP::matched_plane_t>& planePairs,
    mp2_icp::OLAE_ICP::TMatchedPointPlaneList&       pt2plPairs,
    const double xyz_noise_std, const double n_err_std /* normals noise*/)
{
    auto& rnd = mrpt::random::getRandomGenerator();

#if defined(TEST_LARGE_ROTATIONS)
    const double Dx = rnd.drawUniform(-10.0, 10.0);
    const double Dy = rnd.drawUniform(-10.0, 10.0);
    const double Dz = rnd.drawUniform(-10.0, 10.0);

    const double yaw   = mrpt::DEG2RAD(rnd.drawUniform(-180.0, 180.0));
    const double pitch = mrpt::DEG2RAD(rnd.drawUniform(-89.0, 89.0));
    const double roll  = mrpt::DEG2RAD(rnd.drawUniform(-89.0, 89.0));
#else
    const double Dx = rnd.drawUniform(-2.0, 2.0);
    const double Dy = rnd.drawUniform(-2.0, 2.0);
    const double Dz = rnd.drawUniform(-2.0, 2.0);

    const double yaw   = mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0));
    const double pitch = mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0));
    const double roll  = mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0));
#endif

    const auto pose = mrpt::poses::CPose3D(Dx, Dy, Dz, yaw, pitch, roll);
    // just the rotation, to transform vectors (vs. R^3 points):
    const auto pose_rot_only = mrpt::poses::CPose3D(0, 0, 0, yaw, pitch, roll);

    // Points:
    pB.resize(pA.size());
    for (std::size_t i = 0; i < pA.size(); ++i)
    {
        // Transform + noise:
        pose.inverseComposePoint(pA[i], pB[i]);
        pB[i].x += rnd.drawGaussian1D(0, xyz_noise_std);
        pB[i].y += rnd.drawGaussian1D(0, xyz_noise_std);
        pB[i].z += rnd.drawGaussian1D(0, xyz_noise_std);

        // Add pairing:
        mrpt::tfest::TMatchingPair pair;
        pair.this_idx = pair.other_idx = i;
        pair.this_x                    = pA[i][0];
        pair.this_y                    = pA[i][1];
        pair.this_z                    = pA[i][2];

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
        // Centroid: transform + noise
        plB[i].centroid = pose.inverseComposePoint(plA[i].centroid);

        plB[i].centroid.x += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.y += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.z += rnd.drawGaussian1D(0, xyz_noise_std);

        // Plane: rotate + noise
        plB[i].plane = plA[i].plane;
        {
            const mrpt::math::TVector3D ug = plA[i].plane.getNormalVector();
            mrpt::math::TVector3D       ul;
            pose_rot_only.inverseComposePoint(ug, ul);

            auto& coefs = plB[i].plane.coefs;

            // Ax+By+Cz+D=0
            coefs[0] = ul.x + rnd.drawGaussian1D(0, n_err_std);
            coefs[1] = ul.y + rnd.drawGaussian1D(0, n_err_std);
            coefs[2] = ul.z + rnd.drawGaussian1D(0, n_err_std);
            coefs[3] = 0;  // temporary.
            plB[i].plane.unitarize();

            coefs[3] =
                -(coefs[0] * plB[i].centroid.x + coefs[1] * plB[i].centroid.y +
                  coefs[2] * plB[i].centroid.z);
        }

        // Add plane-plane pairing:
        mp2_icp::OLAE_ICP::matched_plane_t pair;
        pair.p_this  = plA[i];
        pair.p_other = plB[i];
        planePairs.push_back(pair);

        // Add point-plane pairing:
        mp2_icp::OLAE_ICP::point_plane_pair_t pt2pl;
        pt2pl.pl_this  = plA[i];
        pt2pl.pt_other = mrpt::math::TPoint3Df(
            plB[i].centroid.x, plB[i].centroid.y, plB[i].centroid.z);

        pt2plPairs.push_back(pt2pl);
    }

    return pose;
}

bool TEST_mp2_icp_olae(
    const size_t numPts, const size_t numLines, const size_t numPlanes,
    const double xyz_noise_std = .0, const double n_err_std = .0)
{
    using namespace mrpt::poses::Lie;

    MRPT_START

    const std::string tstName = mrpt::format(
        "TEST_mp2_icp_olae_nPt=%06u_nLin=%06u_nPl=%06u_xyzStd=%.04f_nStd=%.04f",
        static_cast<unsigned int>(numPts), static_cast<unsigned int>(numLines),
        static_cast<unsigned int>(numPlanes), xyz_noise_std, n_err_std);

    std::cout << "[TEST] " << tstName << "\n";

    mrpt::system::CTimeLogger profiler;
    profiler.setMinLoggingLevel(mrpt::system::LVL_ERROR);  // to make it quiet

    // Repeat the test many times, with different random values:
    mp2_icp::OLAE_ICP::OLAE_Match_Result res;
    mp2_icp::OLAE_ICP::P2P_Match_Result  res2;
    mrpt::poses::CPose3D                        gt_pose;

    const auto max_allowed_error =
        std::min(1.0, 0.1 + 10 * xyz_noise_std + 50 * n_err_std);

    const size_t num_reps = 2000;

    // Collect stats: execution time, norm(error)
    mrpt::math::CMatrixDouble stats(num_reps, 2);

    double avr_err_olea = .0, avr_err_p2p = .0;

    for (size_t rep = 0; rep < num_reps; rep++)
    {
        // The input points & planes
        const TPoints pA  = generate_points(numPts);
        const TPlanes plA = generate_planes(numPlanes);

        TPoints pB;
        TPlanes plB;

        mrpt::tfest::TMatchingPairList                   pointPairs;
        mp2_icp::OLAE_ICP::TMatchedPlaneList      planePairs;
        mp2_icp::OLAE_ICP::TMatchedPointPlaneList pt2plPairs;

        gt_pose = transform_points_planes(
            pA, pB, pointPairs, plA, plB, planePairs, pt2plPairs, xyz_noise_std,
            n_err_std);

        // ========  TEST: olae_match ========
        {
            mp2_icp::OLAE_ICP::OLAE_Match_Input in;
            in.paired_points = pointPairs;
            in.paired_planes = planePairs;

            profiler.enter("olea_match");

            mp2_icp::OLAE_ICP::olae_match(in, res);

            const double dt_last = profiler.leave("olea_match");

            // Collect stats:

            // Measure errors in SE(3) if we have many points, in SO(3)
            // otherwise:
            const auto pos_error = gt_pose - res.optimal_pose;
            const auto err_log_n =
                numPts < (numPlanes + numLines)
                    ? SO<3>::log(pos_error.getRotationMatrix()).norm()
                    : SE<3>::log(pos_error).norm();

            if (err_log_n > max_allowed_error)
            {
                std::cout << " -Ground_truth : " << gt_pose.asString() << "\n"
                          << " -OLEA_output  : " << res.optimal_pose.asString()
                          << "\n -GT_rot:\n"
                          << gt_pose.getRotationMatrix() << "\n";
                ASSERT_BELOW_(err_log_n, max_allowed_error);
            }

            stats(rep, 0) = dt_last;
            stats(rep, 1) = err_log_n;
            avr_err_olea += err_log_n;
        }

#if 0
        // ========  TEST: p2p_match ========
        {
            mp2_icp::OLAE_ICP::P2P_Match_Input in2;
            in2.paired_points = pointPairs;
            in2.paired_pt2pl  = pt2plPairs;

            profiler.enter("p2p_match");

            mp2_icp::OLAE_ICP::p2p_match(in2, res2);

            profiler.leave("p2p_match");

            const auto pos_error = gt_pose - res2.optimal_pose;
            const auto err_log_n = SE<3>::log(pos_error).norm();

            if (err_log_n > max_allowed_error)
            {
                std::cout << " -Ground_truth : " << gt_pose.asString() << "\n"
                          << " -P2P_output  : " << res2.optimal_pose.asString()
                          << "\n -GT_rot:\n"
                          << gt_pose.getRotationMatrix() << "\n";
                ASSERT_BELOW_(err_log_n, max_allowed_error);
            }

            avr_err_p2p += err_log_n;
        }
#endif
    }
    avr_err_olea /= num_reps;
    avr_err_p2p /= num_reps;

    const double dt_olea = profiler.getMeanTime("olea_match");
    const double dt_p2p  = profiler.getMeanTime("p2p_match");

    std::cout << " -Ground_truth   : " << gt_pose.asString() << "\n"
              << " -OLEA_output    : " << res.optimal_pose.asString() << "\n"
              << " -OLEA avr. error: " << avr_err_olea
              << "  Time: " << dt_olea * 1e6 << " [us]\n"
              << " -P2P avr. error : " << avr_err_p2p
              << "  Time: " << dt_p2p * 1e6 << " [us]\n";

    return true;  // all ok.
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        auto& rnd = mrpt::random::getRandomGenerator();
        rnd.randomize();

        const double nXYZ = 0.1;  // [meters] std. noise of XYZ points
        const double nN   = mrpt::DEG2RAD(0.5);  // normals noise

        // arguments: nPts, nLines, nPlanes
        // Points only. Noiseless:
        ASSERT_(TEST_mp2_icp_olae(3 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(4 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(10 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(100 /*pt*/, 0 /*li*/, 0 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(1000 /*pt*/, 0 /*li*/, 0 /*pl*/));

        // Points only. Noisy:
        ASSERT_(TEST_mp2_icp_olae(100 /*pt*/, 0 /*li*/, 0 /*pl*/, nXYZ));
        ASSERT_(TEST_mp2_icp_olae(1000 /*pt*/, 0 /*li*/, 0 /*pl*/, nXYZ));

        // Planes only. Noiseless:
        // ASSERT_(TEST_mp2_icp_olae(0 /*pt*/, 0 /*li*/, 3 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(0 /*pt*/, 0 /*li*/, 10 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(0 /*pt*/, 0 /*li*/, 100 /*pl*/));

        // Planes only. Noisy:
        ASSERT_(TEST_mp2_icp_olae(0 /*pt*/, 0 /*li*/, 10 /*pl*/, 0, nN));
        ASSERT_(TEST_mp2_icp_olae(0 /*pt*/, 0 /*li*/, 100 /*pl*/, 0, nN));

        // Points and planes, noisy.
        // ASSERT_(TEST_mp2_icp_olae(1 /*pt*/, 0 /*li*/, 3 /*pl*/));
        // ASSERT_(TEST_mp2_icp_olae(2 /*pt*/, 0 /*li*/, 1 /*pl*/));
        ASSERT_(TEST_mp2_icp_olae(20 /*pt*/, 0 /*li*/, 10 /*pl*/, nXYZ, nN));
        ASSERT_(TEST_mp2_icp_olae(400 /*pt*/, 0 /*li*/, 100 /*pl*/, nXYZ, nN));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
