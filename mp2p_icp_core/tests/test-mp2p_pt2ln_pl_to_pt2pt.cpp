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
 * @file   test-mp2p_pt2ln_pl_to_pt2pt.cpp
 * @brief  Unit tests for pt2ln_pl_to_pt2pt()
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/pt2ln_pl_to_pt2pt.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{
/** A single pt2pl pairing must convert into exactly one pt2pt pairing, whose
 *  "global" point is the projection of the (transformed) local point onto the
 *  plane. */
void test_single_pt2pl_projects_onto_plane()
{
    Pairings in;
    auto&    p = in.paired_pt2pl.emplace_back();
    // Horizontal plane z=0, through the origin, with unit "up" normal:
    p.pl_global.plane    = mrpt::math::TPlane(0, 0, 1, 0);
    p.pl_global.centroid = {0, 0, 0};
    p.pt_local           = {1.0f, 2.0f, 3.0f};

    SolverContext sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    const Pairings out = pt2ln_pl_to_pt2pt(in, sc);

    ASSERT_EQUAL_(out.paired_pt2pt.size(), 1U);
    const auto& pr = out.paired_pt2pt.at(0);

    // local point is preserved as-is:
    ASSERT_NEAR_(pr.local.x, 1.0, 1e-5);
    ASSERT_NEAR_(pr.local.y, 2.0, 1e-5);
    ASSERT_NEAR_(pr.local.z, 3.0, 1e-5);

    // global point must be the projection onto z=0, i.e. (1,2,0):
    ASSERT_NEAR_(pr.global.x, 1.0, 1e-5);
    ASSERT_NEAR_(pr.global.y, 2.0, 1e-5);
    ASSERT_NEAR_(pr.global.z, 0.0, 1e-5);
}

/** A single pt2ln pairing must convert into exactly one pt2pt pairing, whose
 *  "global" point is the closest point on the line to the (transformed)
 *  local point. */
void test_single_pt2ln_projects_onto_line()
{
    Pairings in;
    auto&    p = in.paired_pt2ln.emplace_back();
    // The X axis:
    p.ln_global = mrpt::math::TLine3D({0, 0, 0}, {1, 0, 0});
    p.pt_local  = {5.0, 3.0, 0.0};

    SolverContext sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    const Pairings out = pt2ln_pl_to_pt2pt(in, sc);

    ASSERT_EQUAL_(out.paired_pt2pt.size(), 1U);
    const auto& pr = out.paired_pt2pt.at(0);

    ASSERT_NEAR_(pr.local.x, 5.0, 1e-5);
    ASSERT_NEAR_(pr.local.y, 3.0, 1e-5);

    // closest point on the X axis to (5,3,0) is (5,0,0):
    ASSERT_NEAR_(pr.global.x, 5.0, 1e-5);
    ASSERT_NEAR_(pr.global.y, 0.0, 1e-5);
    ASSERT_NEAR_(pr.global.z, 0.0, 1e-5);
}

/** The relative pose guess must be applied to the local point before
 *  computing the point-to-plane distance. */
void test_relative_pose_is_applied()
{
    Pairings in;
    auto&    p           = in.paired_pt2pl.emplace_back();
    p.pl_global.plane    = mrpt::math::TPlane(0, 0, 1, 0);  // z=0
    p.pl_global.centroid = {0, 0, 0};
    p.pt_local           = {0.0f, 0.0f, 0.0f};  // at the local origin

    SolverContext sc;
    // Shift the local frame up by 10m in Z:
    sc.guessRelativePose = mrpt::poses::CPose3D(0, 0, 10, 0, 0, 0);

    const Pairings out = pt2ln_pl_to_pt2pt(in, sc);

    ASSERT_EQUAL_(out.paired_pt2pt.size(), 1U);
    const auto& pr = out.paired_pt2pt.at(0);

    // projected onto z=0 regardless of the 10m shift:
    ASSERT_NEAR_(pr.global.z, 0.0, 1e-5);
}

/** Empty input must produce empty output, and must not touch the other
 *  pairing types. */
void test_empty_input()
{
    Pairings in;
    // A pt2pt pairing is passed through untouched (the function only reads
    // paired_pt2pl / paired_pt2ln):
    in.paired_pt2pt.emplace_back();

    SolverContext sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    const Pairings out = pt2ln_pl_to_pt2pt(in, sc);

    ASSERT_(out.paired_pt2pt.empty());
    ASSERT_(out.paired_pt2pl.empty());
    ASSERT_(out.paired_pt2ln.empty());
}

/** Without a relative pose guess, the function cannot project local points
 *  into the global frame and must throw. */
void test_missing_relative_pose_throws()
{
    Pairings in;
    auto&    p           = in.paired_pt2pl.emplace_back();
    p.pl_global.plane    = mrpt::math::TPlane(0, 0, 1, 0);
    p.pl_global.centroid = {0, 0, 0};
    p.pt_local           = {1.0f, 1.0f, 1.0f};

    SolverContext sc;  // guessRelativePose left unset

    bool didThrow = false;
    try
    {
        pt2ln_pl_to_pt2pt(in, sc);
    }
    catch (const std::exception&)
    {
        didThrow = true;
    }
    ASSERT_(didThrow);
}

/** The heuristic in append_from_sorted() keeps the largest-error pairings
 *  first and, once at least 3 are kept, drops the rest as soon as their error
 *  falls under 25% of the largest one. */
void test_heuristic_keeps_large_errors_and_at_least_three()
{
    Pairings in;
    // Five pt2pl pairings at increasing distances to the z=0 plane:
    // 0.01, 0.02, 0.03, 0.5, 1.0  (local x,y arbitrary but distinct)
    const std::vector<double> zs = {0.01, 0.02, 0.03, 0.5, 1.0};
    for (size_t i = 0; i < zs.size(); i++)
    {
        auto& p              = in.paired_pt2pl.emplace_back();
        p.pl_global.plane    = mrpt::math::TPlane(0, 0, 1, 0);
        p.pl_global.centroid = {0, 0, 0};
        p.pt_local           = {static_cast<float>(i), 0.0f, static_cast<float>(zs[i])};
    }

    SolverContext sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    const Pairings out = pt2ln_pl_to_pt2pt(in, sc);

    // largestError=1.0, threshold=0.25: pairings with error>=0.25 are kept in
    // full (1.0, 0.5), then the loop keeps adding smaller ones until it has
    // at least 3 entries, so the 3rd-largest (0.03) is kept too, and the
    // remaining two (0.02, 0.01) - both under the threshold - are dropped:
    ASSERT_EQUAL_(out.paired_pt2pt.size(), 3U);

    // Sorted by descending error: 1.0, 0.5, 0.03
    std::vector<double> gotZs;
    for (const auto& pr : out.paired_pt2pt)
    {
        gotZs.push_back(pr.local.z);
    }
    ASSERT_NEAR_(gotZs.at(0), 1.0, 1e-4);
    ASSERT_NEAR_(gotZs.at(1), 0.5, 1e-4);
    ASSERT_NEAR_(gotZs.at(2), 0.03, 1e-4);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_single_pt2pl_projects_onto_plane();
        std::cout << "test_single_pt2pl_projects_onto_plane: Success ✅" << std::endl;

        test_single_pt2ln_projects_onto_line();
        std::cout << "test_single_pt2ln_projects_onto_line: Success ✅" << std::endl;

        test_relative_pose_is_applied();
        std::cout << "test_relative_pose_is_applied: Success ✅" << std::endl;

        test_empty_input();
        std::cout << "test_empty_input: Success ✅" << std::endl;

        test_missing_relative_pose_throws();
        std::cout << "test_missing_relative_pose_throws: Success ✅" << std::endl;

        test_heuristic_keeps_large_errors_and_at_least_three();
        std::cout << "test_heuristic_keeps_large_errors_and_at_least_three: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
