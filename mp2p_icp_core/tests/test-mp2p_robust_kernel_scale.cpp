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
 * @file   test-mp2p_robust_kernel_scale.cpp
 * @brief  The robust kernel threshold must be in sigmas for every pair type
 * @author Jose Luis Blanco Claraco
 * @date   Sep 2, 2026
 */

#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/robust_kernels.h>
#include <mrpt/poses/Lie/SE.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
constexpr double OUTLIER_DIST = 5.0;  // [m]

/** Point-to-plane pairings on the six faces of a cube, exact for the identity
 *  pose, plus a few gross outliers displaced along +z.
 *
 *  Only the z-normal faces constrain z, so the outliers pull the solution along
 *  that one axis and the test can read the damage off a single number.
 */
mp2p_icp::Pairings build_pairings(size_t nOutliers)
{
    mp2p_icp::Pairings p;

    const std::vector<mrpt::math::TVector3D> normals = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                                        {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

    for (const auto& n : normals)
    {
        // Two directions spanning the face, so the points are not collinear:
        const mrpt::math::TVector3D t1 = {n.y, n.z, n.x};
        const mrpt::math::TVector3D t2 = {n.z, n.x, n.y};

        for (int i = 0; i < 10; i++)
        {
            const double a = 0.3 * static_cast<double>(i % 5) - 0.6;
            const double b = 0.3 * static_cast<double>(i / 5) - 0.15;

            auto& pp     = p.paired_pt2pl.emplace_back();
            pp.pl_global = {
                mrpt::math::TPlane::FromPointAndNormal({n.x, n.y, n.z}, n), {n.x, n.y, n.z}};
            pp.pt_local = {
                static_cast<float>(n.x + a * t1.x + b * t2.x),
                static_cast<float>(n.y + a * t1.y + b * t2.y),
                static_cast<float>(n.z + a * t1.z + b * t2.z)};
        }
    }

    const mrpt::math::TVector3D up = {0, 0, 1};
    for (size_t i = 0; i < nOutliers; i++)
    {
        const double a = 0.3 * static_cast<double>(i) - 0.6;

        auto& pp     = p.paired_pt2pl.emplace_back();
        pp.pl_global = {mrpt::math::TPlane::FromPointAndNormal({0, 0, 1}, up), {0, 0, 1}};
        pp.pt_local  = {static_cast<float>(a), 0.0f, static_cast<float>(1.0 + OUTLIER_DIST)};
    }

    return p;
}

/** Solves the above with a Geman-McClure kernel and the given point-to-plane
 *  weight, and returns |z| of the resulting pose. Zero is the right answer:
 *  the inliers are exact for the identity.
 */
double solve_z_error(double pt2plWeight, size_t nOutliers)
{
    mp2p_icp::Solver_GaussNewton solver;

    mrpt::containers::yaml params;
    params["maxIterations"]     = 25;
    params["robustKernel"]      = "RobustKernel::GemanMcClure";
    params["robustKernelScale"] = 3.0;

    mrpt::containers::yaml w;
    w["pt2pt"]             = 1.0;
    w["pt2pl"]             = pt2plWeight;
    w["pt2ln"]             = 1.0;
    w["ln2ln"]             = 1.0;
    w["pl2pl"]             = 1.0;
    params["pair_weights"] = w;

    solver.initialize(params);

    const auto p = build_pairings(nOutliers);

    mp2p_icp::OptimalTF_Result result;
    mp2p_icp::SolverContext    sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    const bool ok = solver.optimal_pose(p, result, sc);
    ASSERT_(ok);

    return std::abs(result.optimalPose.z());
}

/** Without outliers the weight is a global scale on H and g, so it cancels in
 *  the Gauss-Newton step and cannot move the solution. This pins down that the
 *  weight acts ONLY through the kernel, which is what makes the next test a
 *  measurement of the kernel and not of the weighting.
 */
void test_weight_alone_does_not_move_the_solution()
{
    const double e1   = solve_z_error(1.0, 0);
    const double e400 = solve_z_error(400.0, 0);

    std::cout << "no outliers: z err at w=1: " << e1 << ", at w=400: " << e400 << "\n";

    ASSERT_NEAR_(e1, 0.0, 1e-6);
    ASSERT_NEAR_(e400, 0.0, 1e-6);
}

/** The regression this file exists for: a metric residual fed to the kernel
 *  raw is compared against a threshold in metres, so a 3-sigma kernel becomes a
 *  3-metre one and never fires on residuals of a few centimetres. Whitening the
 *  kernel argument by the pair weight puts the threshold back in sigmas.
 */
void test_kernel_threshold_is_in_sigmas()
{
    const size_t nOutliers = 12;

    // w = 1 means "sigma = 1 m": a 5 m outlier is 5 sigma and the kernel only
    // just starts to bite, so the outliers still drag the solution.
    const double eLoose = solve_z_error(1.0, nOutliers);

    // w = 400 means "sigma = 5 cm": the same outlier is 100 sigma and is
    // rejected outright.
    const double eTight = solve_z_error(400.0, nOutliers);

    std::cout << "with " << nOutliers << " outliers of " << OUTLIER_DIST
              << " m: z err at w=1: " << eLoose << ", at w=400: " << eTight << "\n";

    // The tight weight must recover the true pose:
    ASSERT_NEAR_(eTight, 0.0, 0.05);

    // And it must be a large improvement, not a rounding difference. Before
    // the kernel argument was whitened, both weights gave the same (bad)
    // answer, so this is the assertion that actually fails on a regression.
    ASSERT_GT_(eLoose, 10.0 * std::max(eTight, 1e-3));
}

/** The kernel parameter is a scale in the same units as the residual it is
 *  compared against, so the weight can only depend on the normalized residual
 *  x/c, and must be 1 at x=0 so that the kernel does not rescale its block
 *  against the terms that carry no kernel (the pose prior, the gravity one).
 *
 *  This is what a parameter entering the formula unsquared breaks: the weight
 *  then depends on x/sqrt(c), i.e. the shipped 6.0 acts as a 2.45 one, and no
 *  amount of retuning that number makes the shape scale-free again.
 */
void test_kernel_shape_is_a_function_of_the_normalized_residual()
{
    using mrpt::square;

    for (const auto kernel : {mp2p_icp::RobustKernel::GemanMcClure, mp2p_icp::RobustKernel::Cauchy})
    {
        const auto w1 = mp2p_icp::create_robust_kernel(kernel, 1.0);
        const auto w3 = mp2p_icp::create_robust_kernel(kernel, 3.0);

        ASSERT_NEAR_(w1(0.0), 1.0, 1e-12);
        ASSERT_NEAR_(w3(0.0), 1.0, 1e-12);

        for (double r = 0.25; r <= 4.0; r += 0.25)
        {
            // Equal normalized residuals r=x/c must get equal weights:
            ASSERT_NEAR_(w1(square(r)), w3(square(3.0 * r)), 1e-12);
        }
    }

    // And the two shapes are one the square of the other, so a residual at
    // exactly one "sigma" keeps half of its information under Cauchy and a
    // quarter of it under Geman-McClure:
    const double c  = 2.0;
    const auto   gm = mp2p_icp::create_robust_kernel(mp2p_icp::RobustKernel::GemanMcClure, c);
    const auto   ca = mp2p_icp::create_robust_kernel(mp2p_icp::RobustKernel::Cauchy, c);

    ASSERT_NEAR_(ca(square(c)), 0.5, 1e-12);
    ASSERT_NEAR_(gm(square(c)), 0.25, 1e-12);

    for (double x = 0.0; x <= 10.0; x += 0.5)
    {
        ASSERT_NEAR_(gm(square(x)), square(ca(square(x))), 1e-12);
    }
}
/** A pipeline written against the old key must keep the kernel it had. The
 *  conversion is checked on the loaded parameter and not on a solution: this
 *  problem is deliberately outlier-dominated, and its solve does not repeat to
 *  more than three digits.
 */
void test_deprecated_kernel_key_is_converted()
{
    const auto load = [](const char* key, double value)
    {
        mp2p_icp::Solver_GaussNewton solver;

        mrpt::containers::yaml params;
        params["maxIterations"] = 10;
        params["robustKernel"]  = "RobustKernel::GemanMcClure";
        params[key]             = value;
        solver.initialize(params);
        return solver.robustKernelScale;
    };

    // The old key named the square of the new one:
    ASSERT_NEAR_(load("robustKernelParam", 9.0), 3.0, 1e-12);
    ASSERT_NEAR_(load("robustKernelScale", 3.0), 3.0, 1e-12);

    // A formula-valued legacy entry is converted as a formula, not as a value:
    ASSERT_NEAR_(load("robustKernelParam", 0.25), 0.5, 1e-12);

    // Naming both is ambiguous, and silently picking one would be the very
    // mistake this rename exists to prevent:
    bool didThrow = false;
    try
    {
        mp2p_icp::Solver_GaussNewton solver;

        mrpt::containers::yaml params;
        params["maxIterations"]     = 10;
        params["robustKernel"]      = "RobustKernel::GemanMcClure";
        params["robustKernelParam"] = 9.0;
        params["robustKernelScale"] = 3.0;
        solver.initialize(params);
    }
    catch (const std::exception&)
    {
        didThrow = true;
    }
    ASSERT_(didThrow);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_kernel_shape_is_a_function_of_the_normalized_residual();
        test_weight_alone_does_not_move_the_solution();
        test_kernel_threshold_is_in_sigmas();
        test_deprecated_kernel_key_is_converted();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
