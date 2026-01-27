/*
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
 * @file   test-mp2p_Pairings.cpp
 * @brief  Unit tests for the Pairings structure
 * @author Jose Luis Blanco Claraco
 * @date   Jan 27, 2026
 */

#include <mp2p_icp/Pairings.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mp2p_icp;
using namespace mrpt::math;

namespace
{

void test_Pairings_Serialization()
{
    Pairings p1;

    // 1. Populate with dummy data
    // ---------------------------------------------------
    // Point-to-Point
    {
        mrpt::tfest::TMatchingPair pair;
        pair.globalIdx                      = 10;
        pair.localIdx                       = 20;
        pair.global                         = {1.0, 2.0, 3.0};
        pair.local                          = {1.1, 2.1, 3.1};
        pair.errorSquareAfterTransformation = 0.05;
        p1.paired_pt2pt.push_back(pair);
    }

    // Point-to-Plane
    {
        point_plane_pair_t pair;
        pair.pt_local           = {5.0, 5.0, 5.0};
        pair.pl_global.centroid = {6.0, 6.0, 6.0};
        pair.pl_global.plane = TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 1, 0});  // Z=0 plane
        p1.paired_pt2pl.push_back(pair);
    }

    // Point-to-Line
    {
        point_line_pair_t pair;
        pair.pt_local           = {10.0, 10.0, 10.0};
        pair.ln_global.pBase    = {0, 0, 0};
        pair.ln_global.director = {0, 0, 1};
        p1.paired_pt2ln.push_back(pair);
    }

    // Line-to-Line
    {
        matched_line_t pair;
        pair.ln_local.pBase     = {1, 1, 1};
        pair.ln_local.director  = {1, 0, 0};
        pair.ln_global.pBase    = {2, 2, 2};
        pair.ln_global.director = {0, 1, 0};
        p1.paired_ln2ln.push_back(pair);
    }

    // Plane-to-Plane
    {
        matched_plane_t pair;
        pair.p_local.centroid  = {1, 2, 3};
        pair.p_local.plane     = TPlane(1, 0, 0, 5);
        pair.p_global.centroid = {4, 5, 6};
        pair.p_global.plane    = TPlane(0, 1, 0, 10);
        p1.paired_pl2pl.push_back(pair);
    }

    // Metadata
    p1.potential_pairings = 100;
    p1.point_weights.emplace_back(0, 1.5);  // Index 0 has weight 1.5

    // 2. Serialize
    // ---------------------------------------------------
    mrpt::io::CMemoryStream mem;
    auto                    arch = mrpt::serialization::archiveFrom(mem);
    arch << p1;

    // 3. Deserialize
    // ---------------------------------------------------
    mem.Seek(0);
    Pairings p2;
    arch >> p2;

    // 4. Verification
    // ---------------------------------------------------
    ASSERT_EQUAL_(p1.paired_pt2pt.size(), p2.paired_pt2pt.size());
    ASSERT_EQUAL_(p1.paired_pt2pl.size(), p2.paired_pt2pl.size());
    ASSERT_EQUAL_(p1.paired_pt2ln.size(), p2.paired_pt2ln.size());
    ASSERT_EQUAL_(p1.paired_ln2ln.size(), p2.paired_ln2ln.size());
    ASSERT_EQUAL_(p1.paired_pl2pl.size(), p2.paired_pl2pl.size());

    ASSERT_EQUAL_(p1.potential_pairings, p2.potential_pairings);
    ASSERT_EQUAL_(p1.point_weights.size(), p2.point_weights.size());

    // Deep check one element
    const auto& pt1 = p1.paired_pt2pt[0];
    const auto& pt2 = p2.paired_pt2pt[0];
    ASSERT_NEAR_(pt1.global.x, pt2.global.x, 1e-6);
    ASSERT_EQUAL_(pt1.globalIdx, pt2.globalIdx);

    const auto& w1 = p1.point_weights[0];
    const auto& w2 = p2.point_weights[0];
    ASSERT_EQUAL_(w1.first, w2.first);
    ASSERT_NEAR_(w1.second, w2.second, 1e-6);

    std::cout << "[Test Passed] Pairings Serialization\n";
}

void test_Pairings_PushBack()
{
    Pairings p1, p2;

    p1.paired_pt2pt.resize(1);
    p1.potential_pairings = 10;

    p2.paired_pt2pt.resize(2);
    p2.paired_pt2pl.resize(1);
    p2.potential_pairings = 20;

    // Merge p2 into p1
    p1.push_back(p2);

    ASSERT_EQUAL_(p1.paired_pt2pt.size(), 3ULL);
    ASSERT_EQUAL_(p1.paired_pt2pl.size(), 1ULL);
    ASSERT_EQUAL_(p1.potential_pairings, 30ULL);

    std::cout << "[Test Passed] Pairings push_back\n";
}

void test_Pairings_Viz()
{
    Pairings p;

    // 1. Populate Pairings with data
    // ---------------------------------------------------

    // pt2pt
    mrpt::tfest::TMatchingPair ptPair;
    ptPair.global = {10, 0, 0};
    ptPair.local  = {0, 0, 0};
    p.paired_pt2pt.push_back(ptPair);

    // pt2ln
    point_line_pair_t lnPair;
    lnPair.pt_local           = {0, 5, 0};
    lnPair.ln_global.pBase    = {10, 5, 0};
    lnPair.ln_global.director = {0, 1, 0};
    p.paired_pt2ln.push_back(lnPair);

    // pt2pl
    point_plane_pair_t plPair;
    plPair.pt_local           = {0, 0, 5};
    plPair.pl_global.centroid = {10, 0, 0};
    plPair.pl_global.plane    = TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    p.paired_pt2pl.push_back(plPair);

    // cov2cov
    point_with_cov_pair_t covPair;
    covPair.local   = {0, 1, 0};
    covPair.global  = {2, 1, 0};
    covPair.cov_inv = mrpt::math::CMatrixFloat33::Identity();  // Simple identity covariance
    p.paired_cov2cov.push_back(covPair);

    // 2. Generate Visualization
    // ---------------------------------------------------
    pairings_render_params_t params;
    params.pt2pt.visible   = true;
    params.pt2ln.visible   = true;
    params.pt2pl.visible   = true;
    params.cov2cov.visible = true;

    // Use Identity pose for local->global transform for simplicity
    auto glObjs = p.get_visualization(mrpt::poses::CPose3D::Identity(), params);

    // 3. Verify Contents
    // ---------------------------------------------------
    ASSERT_(glObjs);

    // We expect:
    // - pt2pt: 1 CSetOfLines object
    // - pt2ln: 1 CSetOfLines (pairing lines) + 1 CSetOfLines (global lines)
    // - pt2pl: 1 CSetOfLines (pairing lines) + 1 CTexturedPlane (patch)
    // - cov2cov: 1 CSetOfLines (pairing lines) + 1 CEllipsoid3D

    size_t lineSets   = 0;
    size_t planes     = 0;
    size_t ellipsoids = 0;

    for (const auto& obj : *glObjs)
    {
        if (std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(obj))
        {
            lineSets++;
        }
        if (std::dynamic_pointer_cast<mrpt::opengl::CTexturedPlane>(obj))
        {
            planes++;
        }
        if (std::dynamic_pointer_cast<mrpt::opengl::CEllipsoid3D>(obj))
        {
            ellipsoids++;
        }
    }

    // pt2pt adds 1 line set.
    // pt2ln adds 2 line sets.
    // pt2pl adds 1 line set + 1 plane.
    // cov2cov adds 1 line set + 1 ellipsoid.

    // Total expected line sets: 1 + 2 + 1 + 1 = 5
    // Total expected planes: 1
    // Total expected ellipsoids: 1

    ASSERT_EQUAL_(lineSets, 5ULL);
    ASSERT_EQUAL_(planes, 1ULL);
    ASSERT_EQUAL_(ellipsoids, 1ULL);

    std::cout << "[Test Passed] Pairings Visualization Construction\n";
}

}  // namespace

int main()
{
    try
    {
        test_Pairings_Serialization();
        test_Pairings_PushBack();
        test_Pairings_Viz();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}