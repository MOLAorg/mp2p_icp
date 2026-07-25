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
 * @file   test-mp2p_gravity_prior.cpp
 * @brief  Unit tests for the rank-2, yaw-free gravity prior of the GN solver
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/tfest/TMatchingPair.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
/** A minimal, well-conditioned point cloud pairing set: a cube of points that
 *  pins all 6 DOF on its own, so any change in the solution when the gravity
 *  prior is added is attributable to the prior alone. */
mp2p_icp::Pairings makeCubePairings(const mrpt::poses::CPose3D& groundTruth)
{
    mp2p_icp::Pairings p;
    const double       C[8][3] = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {10, 10, 0},
                                  {0, 0, 5}, {10, 0, 5}, {0, 10, 5}, {10, 10, 5}};
    for (const auto& c : C)
    {
        const mrpt::math::TPoint3D global(c[0], c[1], c[2]);
        // local = GT^-1 * global, so that the optimum is exactly groundTruth:
        const mrpt::math::TPoint3D local = groundTruth.inverseComposePoint(global);

        mrpt::tfest::TMatchingPair pr;
        pr.global = mrpt::math::TPoint3Df(global.x, global.y, global.z);
        pr.local  = mrpt::math::TPoint3Df(local.x, local.y, local.z);
        p.paired_pt2pt.push_back(pr);
    }
    return p;
}

double tiltAngleDeg(const mrpt::poses::CPose3D& P, const mrpt::math::TVector3D& up_body)
{
    const auto            R = P.getRotationMatrix();
    mrpt::math::TVector3D u_map;
    u_map.x        = R(0, 0) * up_body.x + R(0, 1) * up_body.y + R(0, 2) * up_body.z;
    u_map.y        = R(1, 0) * up_body.x + R(1, 1) * up_body.y + R(1, 2) * up_body.z;
    u_map.z        = R(2, 0) * up_body.x + R(2, 1) * up_body.y + R(2, 2) * up_body.z;
    const double n = std::sqrt(u_map.x * u_map.x + u_map.y * u_map.y + u_map.z * u_map.z);
    return mrpt::RAD2DEG(std::acos(std::min(1.0, std::max(-1.0, u_map.z / n))));
}
}  // namespace

/** The gravity prior must leave YAW free at ANY accumulated yaw. This is the
 *  property a diagonal 6x6 information in the Lie tangent cannot express: it
 *  only isolates roll/pitch when yaw is near zero. */
static void test_yaw_is_free_at_any_yaw()
{
    for (const double yawDeg : {0.0, 45.0, 90.0, 170.0, -120.0})
    {
        const mrpt::poses::CPose3D gt(1.0, 2.0, 3.0, mrpt::DEG2RAD(yawDeg), 0.0, 0.0);
        const auto                 pairings = makeCubePairings(gt);

        mp2p_icp::OptimalTF_GN_Parameters gn;
        // Start displaced in yaw: only the geometry should pull it back; the
        // gravity term must contribute nothing along yaw.
        gn.linearizationPoint =
            mrpt::poses::CPose3D(1.0, 2.0, 3.0, mrpt::DEG2RAD(yawDeg + 10.0), 0.0, 0.0);
        gn.maxInnerLoopIterations = 25;

        auto& g     = gn.gravityPrior.emplace();
        g.up_body   = {0, 0, 1};
        g.up_map    = {0, 0, 1};
        g.sigma_rad = 1e-3;  // extremely strong: would dominate if it touched yaw

        mp2p_icp::OptimalTF_Result res;
        const bool                 ok = mp2p_icp::optimal_tf_gauss_newton(pairings, res, gn);
        ASSERT_(ok);

        const double yawErr = mrpt::RAD2DEG(mrpt::math::wrapToPi(res.optimalPose.yaw() - gt.yaw()));
        std::cout << "[yaw-free] yaw=" << yawDeg << " deg -> recovered yaw err=" << yawErr
                  << " deg\n";
        // Geometry alone must still recover yaw despite a near-hard gravity term:
        ASSERT_LT_(std::abs(yawErr), 0.5);
    }
}

/** A strong gravity prior must actually exert force: with a GENUINELY TILTED
 *  geometric optimum, it must pull the solution away from that optimum and
 *  toward level. (A level ground truth would make this pass trivially, since
 *  geometry alone already lands level - it would prove nothing.) */
static void test_gravity_levels_the_solution()
{
    // Ground truth is tilted 5 deg in pitch: geometry alone WILL return it.
    const mrpt::poses::CPose3D gt(
        0, 0, 0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(5.0), mrpt::DEG2RAD(-3.0));
    const auto pairings = makeCubePairings(gt);

    mp2p_icp::OptimalTF_GN_Parameters gn;
    gn.linearizationPoint     = gt;  // start AT the geometric optimum
    gn.maxInnerLoopIterations = 25;

    mp2p_icp::OptimalTF_Result resNoGrav;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, resNoGrav, gn));

    auto& g     = gn.gravityPrior.emplace();
    g.up_body   = {0, 0, 1};
    g.up_map    = {0, 0, 1};
    g.sigma_rad = mrpt::DEG2RAD(0.01);  // near-mandatory: must dominate geometry

    mp2p_icp::OptimalTF_Result resGrav;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, resGrav, gn));

    const double tiltNo   = tiltAngleDeg(resNoGrav.optimalPose, {0, 0, 1});
    const double tiltWith = tiltAngleDeg(resGrav.optimalPose, {0, 0, 1});
    std::cout << "[leveling] tilt without prior=" << tiltNo << " deg, with prior=" << tiltWith
              << " deg\n";

    // Sanity: without the prior the solution really is tilted (else the test
    // below would be vacuous):
    ASSERT_GT_(tiltNo, 4.0);
    // With a near-mandatory prior, the tilt must be largely removed:
    ASSERT_LT_(tiltWith, 0.5);

    // ...and it must remove the tilt WITHOUT dragging yaw, at a yaw far from
    // zero (30 deg here). This is precisely what a diagonal information in the
    // Lie tangent cannot guarantee, and where mixing up the rotation-vector
    // index ordering shows up as a spurious yaw constraint.
    const double yawDrift =
        mrpt::RAD2DEG(mrpt::math::wrapToPi(resGrav.optimalPose.yaw() - gt.yaw()));
    std::cout << "[leveling] yaw drift induced by the gravity term=" << yawDrift << " deg\n";
    ASSERT_LT_(std::abs(yawDrift), 0.5);
}

/** A gravity prior consistent with an already-level solution must be inert:
 *  it must not move the solution, and in particular must not inject Z. */
static void test_consistent_prior_is_inert()
{
    const mrpt::poses::CPose3D gt(1.0, -2.0, 0.5, mrpt::DEG2RAD(25.0), 0.0, 0.0);
    const auto                 pairings = makeCubePairings(gt);

    mp2p_icp::OptimalTF_GN_Parameters gn;
    gn.linearizationPoint     = gt;
    gn.maxInnerLoopIterations = 25;

    auto& g     = gn.gravityPrior.emplace();
    g.up_body   = {0, 0, 1};
    g.up_map    = {0, 0, 1};
    g.sigma_rad = 1e-4;  // near-mandatory

    mp2p_icp::OptimalTF_Result res;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, res, gn));

    std::cout << "[inert] dz=" << (res.optimalPose.z() - gt.z()) << " m\n";
    ASSERT_LT_(std::abs(res.optimalPose.z() - gt.z()), 1e-3);
    ASSERT_LT_(std::abs(res.optimalPose.x() - gt.x()), 1e-3);
    ASSERT_LT_(std::abs(res.optimalPose.y() - gt.y()), 1e-3);
}

/** A non-level MAP frame must be handled via up_map, with no Euler algebra. */
static void test_non_level_map_frame()
{
    // Map frame tilted 8 deg about its X axis => "up" in map coords:
    const double          a = mrpt::DEG2RAD(8.0);
    mrpt::math::TVector3D up_map{0.0, -std::sin(a), std::cos(a)};

    const mrpt::poses::CPose3D gt(0, 0, 0, mrpt::DEG2RAD(50.0), 0, 0);
    const auto                 pairings = makeCubePairings(gt);

    mp2p_icp::OptimalTF_GN_Parameters gn;
    gn.linearizationPoint =
        mrpt::poses::CPose3D(0, 0, 0, mrpt::DEG2RAD(50.0), mrpt::DEG2RAD(5.0), 0);
    gn.maxInnerLoopIterations = 25;

    auto& g     = gn.gravityPrior.emplace();
    g.up_body   = {0, 0, 1};
    g.up_map    = up_map;
    g.sigma_rad = mrpt::DEG2RAD(0.1);

    mp2p_icp::OptimalTF_Result res;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, res, gn));

    // The body "up" must end up aligned with the MAP's gravity direction:
    const auto            R = res.optimalPose.getRotationMatrix();
    mrpt::math::TVector3D u{R(0, 2), R(1, 2), R(2, 2)};
    const double          dot    = u.x * up_map.x + u.y * up_map.y + u.z * up_map.z;
    const double          angErr = mrpt::RAD2DEG(std::acos(std::min(1.0, std::max(-1.0, dot))));
    std::cout << "[non-level map] residual angle to map gravity=" << angErr << " deg\n";
    ASSERT_LT_(angErr, 0.2);
}

int main(int, char**)
{
    try
    {
        test_yaw_is_free_at_any_yaw();
        test_gravity_levels_the_solution();
        test_consistent_prior_is_inert();
        test_non_level_map_frame();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
