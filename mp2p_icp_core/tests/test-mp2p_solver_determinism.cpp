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
 * @file   test-mp2p_solver_determinism.cpp
 * @brief  optimal_tf_gauss_newton() must not depend on the number of TBB workers
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2026
 */

#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <vector>

#if defined(MP2P_HAS_TBB)
#include <tbb/global_control.h>
#endif

namespace
{
/** A deliberately large and irregular set of pairings of all three types that
 *  the solver accumulates in parallel.
 *
 *  Size and irregularity are both load-bearing: a short or perfectly uniform
 *  problem can sum to the same bits under any partition, which would make this
 *  test pass without measuring anything. The values come from a fixed seed, so
 *  the problem itself is identical in every run.
 */
mp2p_icp::Pairings build_pairings()
{
    constexpr size_t N = 20000;

    std::mt19937                           rng(1234);
    std::uniform_real_distribution<double> coord(-30.0, 30.0);
    std::uniform_real_distribution<double> noise(-0.05, 0.05);
    std::uniform_real_distribution<double> scale(0.1, 10.0);

    mp2p_icp::Pairings p;

    p.paired_pt2pt.reserve(N);
    p.paired_cov2cov.reserve(N);
    p.paired_pt2pl.reserve(N);

    for (size_t i = 0; i < N; i++)
    {
        const double x = coord(rng);
        const double y = coord(rng);
        const double z = coord(rng);

        {
            auto& pp  = p.paired_pt2pt.emplace_back();
            pp.global = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
            pp.local  = {
                 static_cast<float>(x + noise(rng)), static_cast<float>(y + noise(rng)),
                 static_cast<float>(z + noise(rng))};
        }
        {
            auto& pp  = p.paired_cov2cov.emplace_back();
            pp.global = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
            pp.local  = {
                 static_cast<float>(x + noise(rng)), static_cast<float>(y + noise(rng)),
                 static_cast<float>(z + noise(rng))};
            pp.cov_inv.setDiagonal(std::vector<float>(
                {static_cast<float>(scale(rng)), static_cast<float>(scale(rng)),
                 static_cast<float>(scale(rng))}));
        }
        {
            // A plane through (x,y,z) with a normal that keeps rotating:
            const double                a  = 0.001 * static_cast<double>(i);
            const double                nx = std::cos(a);
            const double                ny = std::sin(a) * std::cos(2 * a);
            const double                nz = std::sin(a) * std::sin(2 * a);
            const double                nn = std::sqrt(nx * nx + ny * ny + nz * nz);
            const mrpt::math::TVector3D n  = {nx / nn, ny / nn, nz / nn};

            auto& pp     = p.paired_pt2pl.emplace_back();
            pp.pl_global = {mrpt::math::TPlane::FromPointAndNormal({x, y, z}, n), {x, y, z}};
            pp.pt_local  = {
                 static_cast<float>(x + noise(rng)), static_cast<float>(y + noise(rng)),
                 static_cast<float>(z + noise(rng))};
        }
    }

    return p;
}

mp2p_icp::OptimalTF_Result solve(const mp2p_icp::Pairings& p)
{
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 20;
    gnParams.minDelta               = 0;  // never stop early: exercise every iteration
    // A robust kernel makes the weights a nonlinear function of the residual,
    // so a rounding difference in one iteration propagates into the next:
    gnParams.kernel      = mp2p_icp::RobustKernel::GemanMcClure;
    gnParams.kernelScale = 0.5;

    mp2p_icp::OptimalTF_Result result;
    const bool                 ok = mp2p_icp::optimal_tf_gauss_newton(p, result, gnParams);
    ASSERT_(ok);
    return result;
}

/** The object representation of a double, so that the comparison below is on
 *  the bits and not on the numeric value: `==` would call +0.0 and -0.0 equal,
 *  and they are two different results of two different summation orders, which
 *  is exactly what this test exists to catch.
 */
uint64_t bits_of(double v)
{
    uint64_t u = 0;
    std::memcpy(&u, &v, sizeof(u));
    return u;
}

void expect_bit_identical(
    const mp2p_icp::OptimalTF_Result& a, const mp2p_icp::OptimalTF_Result& b,
    const std::string& context)
{
    for (int i = 0; i < 6; i++)
    {
        if (bits_of(a.optimalPose[i]) == bits_of(b.optimalPose[i]))
        {
            continue;
        }

        std::cerr << "[test-mp2p_solver_determinism] " << context << "\n"
                  << " reference pose: " << a.optimalPose << "\n"
                  << " obtained pose : " << b.optimalPose << "\n"
                  << " first differing component: " << i << " ("
                  << (a.optimalPose[i] - b.optimalPose[i]) << " apart, bits " << std::hex
                  << bits_of(a.optimalPose[i]) << " vs " << bits_of(b.optimalPose[i]) << std::dec
                  << ")\n";

        THROW_EXCEPTION("The solver result depends on the thread scheduling.");
    }
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        const auto pairings = build_pairings();

#if defined(MP2P_HAS_TBB)
        // Worker counts that partition the same range differently:
        const std::vector<size_t> threadCounts = {1, 2, 3, 4, 8};

        std::optional<mp2p_icp::OptimalTF_Result> reference;

        for (const size_t nThreads : threadCounts)
        {
            tbb::global_control gc(tbb::global_control::max_allowed_parallelism, nThreads);

            // Twice per worker count: at a fixed count, plain parallel_reduce
            // can still partition differently from one run to the next.
            for (int rep = 0; rep < 2; rep++)
            {
                const auto r = solve(pairings);

                if (!reference)
                {
                    reference = r;
                    continue;
                }

                expect_bit_identical(
                    *reference, r,
                    "with " + std::to_string(nThreads) + " worker(s), repetition " +
                        std::to_string(rep));
            }
        }
#else
        // No TBB: the solver runs the sequential path. Just check it is
        // self-consistent, so the test still has meaning in such builds.
        const auto r1 = solve(pairings);
        const auto r2 = solve(pairings);
        expect_bit_identical(r1, r2, "sequential build, second run");
#endif

        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
