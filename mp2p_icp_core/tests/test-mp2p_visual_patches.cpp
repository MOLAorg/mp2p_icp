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
 * @file   test-mp2p_visual_patches.cpp
 * @brief  Unit tests for the photometric ("virtual patch") term of the solver
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/VisualPatches.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>
#include <iostream>
#include <random>

#include "visual_patch_terms.h"

namespace
{
constexpr unsigned int IMG_W = 640;
constexpr unsigned int IMG_H = 480;

/** A texture with gradient in both directions everywhere and, deliberately, a
 *  roughly 1/f spectrum: five octaves from a 256 px period down to 16 px, with
 *  amplitude falling as the frequency rises.
 *
 *  The spectrum is the point. A single-frequency pattern stays just as
 *  ambiguous after downsampling as before, so it cannot show whether a
 *  coarse-to-fine schedule helps; natural images have structure at every scale,
 *  which is the property the pyramid actually exploits. */
mrpt::img::CImage makeTexture()
{
    constexpr int    kOctaves      = 5;
    constexpr double kBasePeriodPx = 256.0;

    mrpt::img::CImage im(IMG_W, IMG_H, mrpt::img::CH_GRAY);
    for (unsigned int y = 0; y < IMG_H; y++)
    {
        auto* row = im.ptrLine<uint8_t>(y);
        for (unsigned int x = 0; x < IMG_W; x++)
        {
            double v = 128.0;
            for (int k = 0; k < kOctaves; k++)
            {
                const double scale = static_cast<double>(1 << k);
                const double w     = 2.0 * M_PI * scale / kBasePeriodPx;
                const double amp   = 45.0 / scale;
                v += amp * std::sin(w * x + 0.7 * k) * std::cos(0.9 * w * y + 1.3 * k);
            }
            row[x] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, v)));
        }
    }
    return im;
}

mrpt::img::TCamera makeCamera(mrpt::img::DistortionModel model)
{
    mrpt::img::TCamera c;
    c.ncols = IMG_W;
    c.nrows = IMG_H;
    c.intrinsicParams.setZero();
    c.intrinsicParams(0, 0) = 400.0;
    c.intrinsicParams(1, 1) = 400.0;
    c.intrinsicParams(0, 2) = 0.5 * IMG_W;
    c.intrinsicParams(1, 2) = 0.5 * IMG_H;
    c.intrinsicParams(2, 2) = 1.0;
    c.distortion            = model;
    if (model == mrpt::img::DistortionModel::kannala_brandt)
    {
        c.dist    = {{0, 0, 0, 0, 0, 0, 0, 0}};
        c.dist[0] = -0.02;  // k1
        c.dist[1] = 0.004;  // k2
        c.dist[4] = -0.001;  // k3
        c.dist[5] = 0.0002;  // k4
    }
    return c;
}

double sampleBilinear(const mrpt::img::CImage& im, double u, double v)
{
    const int    x0 = static_cast<int>(std::floor(u));
    const int    y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;

    const auto* r0 = im.ptrLine<uint8_t>(static_cast<unsigned int>(y0));
    const auto* r1 = im.ptrLine<uint8_t>(static_cast<unsigned int>(y0 + 1));

    return (1 - ay) * ((1 - ax) * r0[x0] + ax * r0[x0 + 1]) +
           ay * ((1 - ax) * r1[x0] + ax * r1[x0 + 1]);
}

/** Builds a term whose exact optimum is `truePose`: the anchors are back-
 *  projected from that pose, and every reference patch is cut from the very
 *  image the term will be scored against. */
mp2p_icp::VisualPatchTerm makeTerm(
    const mrpt::poses::CPose3D& truePose, mrpt::img::DistortionModel model, unsigned int nPatches,
    bool flatTexture = false, uint32_t levels = 1)
{
    mp2p_icp::VisualPatchTerm term;
    term.image  = makeTexture();
    term.camera = makeCamera(model);
    // The local frame IS the camera frame here, so nothing about the extrinsic
    // can mask a sign error in the pose Jacobian.
    term.camera_pose_on_local = mrpt::poses::CPose3D::Identity();
    term.half_size            = 3;
    term.sigma_intensity      = 8.0;
    term.huber_delta          = 1e9;  // no down-weighting: tests check raw math
    term.max_rms_sigmas       = 1e9;

    if (flatTexture)
    {
        term.image.filledRectangle(0, 0, IMG_W - 1, IMG_H - 1, mrpt::img::TColor(100, 100, 100));
    }

    term.pyramid_levels = levels;
    term.buildPyramid(levels);

    const int half = static_cast<int>(term.half_size);

    std::mt19937                           rng(42);
    std::uniform_real_distribution<double> uPix(60.0, IMG_W - 60.0);
    std::uniform_real_distribution<double> vPix(60.0, IMG_H - 60.0);
    std::uniform_real_distribution<double> depth(4.0, 9.0);

    while (term.patches.size() < nPatches)
    {
        const double u = uPix(rng);
        const double v = vPix(rng);
        const double z = depth(rng);

        // Back-project with the ideal pinhole part. The anchor need not land on
        // exactly (u,v) under distortion; what matters is that the patch below
        // is cut wherever the model actually projects it.
        const mrpt::math::TPoint3D pCam(
            z * (u - term.camera.cx()) / term.camera.fx(),
            z * (v - term.camera.cy()) / term.camera.fy(), z);

        const auto px = mp2p_icp::projectToPixel(term.camera, pCam, 0.05);
        if (!px)
        {
            continue;
        }
        if (px->x < half + 4 || px->y < half + 4 || px->x > IMG_W - half - 5 ||
            px->y > IMG_H - half - 5)
        {
            continue;
        }

        mp2p_icp::visual_patch_t p;
        p.pt_global       = truePose.composePoint(pCam);
        p.ref_camera_pose = truePose;  // camera_pose_on_local is the identity
        // One reference patch per pyramid level, sampled on that level's grid.
        for (size_t lv = 0; lv < term.image_pyramid.size(); lv++)
        {
            const auto&        im = term.image_pyramid[lv];
            const double       sc = 1.0 / static_cast<double>(1u << lv);
            std::vector<float> lvPatch;
            lvPatch.reserve(static_cast<size_t>(2 * half + 1) * (2 * half + 1));
            for (int dy = -half; dy <= half; dy++)
            {
                for (int dx = -half; dx <= half; dx++)
                {
                    lvPatch.push_back(
                        static_cast<float>(sampleBilinear(im, px->x * sc + dx, px->y * sc + dy)));
                }
            }
            p.ref_patches.push_back(std::move(lvPatch));
        }
        term.patches.push_back(std::move(p));
    }
    return term;
}

double costAt(const mp2p_icp::VisualPatchTerm& term, const mrpt::poses::CPose3D& pose)
{
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    return mp2p_icp::accumulate_visual_patches(term, pose, H, g).chi2;
}

/** At the pose the patches were captured from, the term must vanish. */
void test_zero_residual_at_truth()
{
    const mrpt::poses::CPose3D truth(0.3, -0.2, 0.1, 0.02, -0.01, 0.015);
    const auto                 term = makeTerm(truth, mrpt::img::DistortionModel::none, 60);

    Eigen::Matrix<double, 6, 6> H  = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g  = Eigen::Matrix<double, 6, 1>::Zero();
    const auto                  st = mp2p_icp::accumulate_visual_patches(term, truth, H, g);

    // The gradient is in gray-levels x pixels x (1/m); what has to vanish is
    // the pose correction it implies, which is in metres and radians.
    const Eigen::Matrix<double, 6, 1> step = H.ldlt().solve(g);

    std::cout << "[truth] used=" << st.used << " rejected=" << st.rejected << " chi2=" << st.chi2
              << " |GN step|=" << step.norm() << "\n";

    ASSERT_EQUAL_(st.used, term.patches.size());
    ASSERT_LT_(st.chi2, 1e-6);
    ASSERT_LT_(step.norm(), 1e-5);
    ASSERT_(term.patches.front().ref_patches.size() == 1);
    // All 6 DOF are observed: a purely photometric H must be full rank.
    ASSERT_GT_(static_cast<int>(H.fullPivLu().rank()), 5);
}

/** The accumulated gradient must be the derivative of the cost it reports.
 *
 * Only up to the frozen-warp approximation: the analytic Jacobian holds the
 * affine patch warp fixed within an iteration, while the numeric derivative
 * below re-warps at every probe. The two therefore differ most along the
 * optical axis, whose signal for a point near the principal point is almost
 * entirely a change of patch scale, i.e. exactly what the warp absorbs. This
 * is the standard linearization for direct photometric methods, and it costs
 * convergence rate in one DOF, not correctness. */
void test_gradient_matches_numeric()
{
    const mrpt::poses::CPose3D truth(0.3, -0.2, 0.1, 0.02, -0.01, 0.015);
    const auto                 term = makeTerm(truth, mrpt::img::DistortionModel::none, 80);

    // Away from the optimum, where the gradient is not trivially zero:
    mrpt::math::CVectorFixedDouble<6> off;
    off[0]          = 0.02;
    off[1]          = -0.015;
    off[2]          = 0.01;
    off[3]          = 0.002;
    off[4]          = -0.0015;
    off[5]          = 0.001;
    const auto pose = truth + mrpt::poses::Lie::SE<3>::exp(off);

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    mp2p_icp::accumulate_visual_patches(term, pose, H, g);

    // The reported chi2 is sum(w*r^2), so its gradient is 2*g.
    for (int i = 0; i < 6; i++)
    {
        const double eps = i < 3 ? 1e-4 : 1e-5;

        mrpt::math::CVectorFixedDouble<6> dp;
        dp.setZero();
        dp[i]               = eps;
        const double cPlus  = costAt(term, pose + mrpt::poses::Lie::SE<3>::exp(dp));
        dp[i]               = -eps;
        const double cMinus = costAt(term, pose + mrpt::poses::Lie::SE<3>::exp(dp));

        const double numeric  = (cPlus - cMinus) / (2 * eps);
        const double analytic = 2.0 * g[i];
        const double scale    = std::max(1.0, std::abs(numeric));

        std::cout << "[grad] dof " << i << " numeric=" << numeric << " analytic=" << analytic
                  << " rel=" << std::abs(numeric - analytic) / scale << "\n";

        ASSERT_LT_(std::abs(numeric - analytic) / scale, 0.15);
    }
}

/** Gauss-Newton on the photometric block alone must walk back to the pose the
 *  patches were captured from, for both a pinhole and a fisheye model. */
void test_recovers_a_perturbed_pose(mrpt::img::DistortionModel model, const char* label)
{
    const mrpt::poses::CPose3D truth(0.3, -0.2, 0.1, 0.02, -0.01, 0.015);
    const auto                 term = makeTerm(truth, model, 120);

    mrpt::math::CVectorFixedDouble<6> off;
    off[0]    = 0.03;
    off[1]    = -0.025;
    off[2]    = 0.02;
    off[3]    = 0.004;
    off[4]    = -0.003;
    off[5]    = 0.0025;
    auto pose = mrpt::poses::CPose3D(truth + mrpt::poses::Lie::SE<3>::exp(off));

    const double err0 = (pose - truth).norm();

    for (int it = 0; it < 25; it++)
    {
        Eigen::Matrix<double, 6, 6> H  = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> g  = Eigen::Matrix<double, 6, 1>::Zero();
        const auto                  st = mp2p_icp::accumulate_visual_patches(term, pose, H, g);
        ASSERT_GT_(st.used, 50U);

        // Light Levenberg damping: the photometric cost is not quadratic far
        // from the optimum, and this test starts a few pixels away.
        H.diagonal() *= 1.02;
        const Eigen::Matrix<double, 6, 1> delta = -H.ldlt().solve(g);
        pose = pose + mrpt::poses::Lie::SE<3>::exp(mrpt::math::CVectorFixed<double, 6>(delta));

        if (delta.norm() < 1e-9)
        {
            break;
        }
    }

    const auto   d      = pose - truth;
    const double posErr = std::sqrt(d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
    const double rotErr = mrpt::poses::Lie::SO<3>::log(d.getRotationMatrix()).norm();

    std::cout << "[" << label << "] start err=" << err0 << " -> pos=" << posErr
              << " m, rot=" << mrpt::RAD2DEG(rotErr) << " deg\n";

    ASSERT_LT_(posErr, 5e-3);
    ASSERT_LT_(mrpt::RAD2DEG(rotErr), 0.05);
}

/** Gauss-Newton from a perturbation too large for one level, which a
 *  coarse-to-fine schedule is supposed to recover and a single level is not.
 *  This is the whole claim of the pyramid, so it is asserted both ways. */
void test_pyramid_widens_the_basin()
{
    const mrpt::poses::CPose3D truth(0.3, -0.2, 0.1, 0.02, -0.01, 0.015);

    // Far enough that a single-level linearization has no reason to point the
    // right way: the texture below repeats every ~27 px and this is a
    // comparable displacement at the patches' depths.
    mrpt::math::CVectorFixedDouble<6> off;
    off[0] = 0.30;
    off[1] = -0.26;
    off[2] = 0.10;
    off[3] = 0.030;
    off[4] = -0.026;
    off[5] = 0.020;

    const auto runFrom = [&](uint32_t levels)
    {
        const auto term =
            makeTerm(truth, mrpt::img::DistortionModel::none, 150, /*flat*/ false, levels);
        auto pose = mrpt::poses::CPose3D(truth + mrpt::poses::Lie::SE<3>::exp(off));

        for (int it = 0; it < 40; it++)
        {
            const unsigned int level = levels > 1 ? static_cast<unsigned int>(std::max<int>(
                                                        0, static_cast<int>(levels) - 1 - it / 8))
                                                  : 0u;

            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
            const auto st = mp2p_icp::accumulate_visual_patches(term, pose, H, g, level);
            if (st.used < 30)
            {
                break;
            }
            H.diagonal() *= 1.02;
            const Eigen::Matrix<double, 6, 1> delta = -H.ldlt().solve(g);
            pose = pose + mrpt::poses::Lie::SE<3>::exp(mrpt::math::CVectorFixed<double, 6>(delta));
        }
        const auto d = pose - truth;
        return std::sqrt(d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
    };

    const double errOneLevel   = runFrom(1);
    const double errThreeLevel = runFrom(3);

    std::cout << "[pyramid] from a large start: 1 level -> " << errOneLevel << " m, 3 levels -> "
              << errThreeLevel << " m\n";

    // The point is not that three levels is slightly better: it is that one
    // level does not get there at all.
    ASSERT_LT_(errThreeLevel, 0.02);
    ASSERT_GT_(errOneLevel, 4 * errThreeLevel);
}

/** A texture-free image carries no information, and must not fake any. */
void test_flat_texture_is_inert()
{
    const mrpt::poses::CPose3D truth(0.3, -0.2, 0.1, 0.02, -0.01, 0.015);
    const auto term = makeTerm(truth, mrpt::img::DistortionModel::none, 40, /*flatTexture*/ true);

    mrpt::math::CVectorFixedDouble<6> off;
    off[0]          = 0.05;
    off[1]          = 0.04;
    off[2]          = -0.03;
    off[3]          = 0.005;
    off[4]          = 0.004;
    off[5]          = -0.003;
    const auto pose = truth + mrpt::poses::Lie::SE<3>::exp(off);

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    mp2p_icp::accumulate_visual_patches(term, pose, H, g);

    std::cout << "[flat] |H|=" << H.norm() << " |g|=" << g.norm() << "\n";
    ASSERT_LT_(H.norm(), 1e-9);
    ASSERT_LT_(g.norm(), 1e-9);
}

/** Patches behind the camera or out of frame must be dropped, not clamped. */
void test_rejects_invisible_patches()
{
    const mrpt::poses::CPose3D truth = mrpt::poses::CPose3D::Identity();
    auto                       term  = makeTerm(truth, mrpt::img::DistortionModel::none, 20);

    const size_t nGood = term.patches.size();

    auto behind      = term.patches.front();
    behind.pt_global = mrpt::math::TPoint3D(0, 0, -5.0);
    term.patches.push_back(behind);

    auto farOut      = term.patches.front();
    farOut.pt_global = mrpt::math::TPoint3D(50.0, 0, 5.0);
    term.patches.push_back(farOut);

    Eigen::Matrix<double, 6, 6> H  = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g  = Eigen::Matrix<double, 6, 1>::Zero();
    const auto                  st = mp2p_icp::accumulate_visual_patches(term, truth, H, g);

    std::cout << "[visibility] used=" << st.used << " rejected=" << st.rejected << "\n";
    ASSERT_EQUAL_(st.used, nGood);
    ASSERT_EQUAL_(st.rejected, 2U);
}

}  // namespace

int main(int, char**)
{
    try
    {
        test_zero_residual_at_truth();
        test_gradient_matches_numeric();
        test_recovers_a_perturbed_pose(mrpt::img::DistortionModel::none, "pinhole");
        test_recovers_a_perturbed_pose(mrpt::img::DistortionModel::kannala_brandt, "fisheye");
        test_pyramid_widens_the_basin();
        test_flat_texture_is_inert();
        test_rejects_invisible_patches();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
