/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Planes_Normals.cpp
 * @brief  Pointcloud matcher: similar planes
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */

// TODO

// Parameter:
/** Maximum angle (radians) between potential matching plane normals to be
 * accepted as a pairing. */
// TODO: Move to a new plane-to-plane matcher
// double thresholdPlane2PlaneNormalAng{mrpt::DEG2RAD(5.0)};

#if 0

#include <mp2p_icp/Matcher_Planes_Normals.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_DistanceThreshold, Matcher, mp2p_icp);

using namespace mp2p_icp;

void f()
{
    // Plane-to-plane correspondence:

    // We have pairs of planes whose centroids are quite close.
    // Check their normals too:
    for (const auto& pair : mpl)
    {
        // 1) Check fo pairing sanity:
        ASSERTDEB_(pair.this_idx < pcs1.planes.size());
        ASSERTDEB_(pair.other_idx < pcs2.planes.size());

        const auto& p1 = s.pc1.planes[pair.this_idx];
        const auto& p2 = s.pc2.planes[pair.other_idx];

        const mrpt::math::TVector3D n1 = p1.plane.getNormalVector();
        const mrpt::math::TVector3D n2 = p2.plane.getNormalVector();

        // dot product to find the angle between normals:
        const double dp      = n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
        const double n2n_ang = std::acos(dp);

        // 2) append to list of plane pairs:
        if (n2n_ang < p.thresholdPlane2PlaneNormalAng)
        {
            // Accept pairing:
            pairings.paired_pl2pl.emplace_back(p1, p2);
        }
    }
}

void f2()
{
    // point-to-planes
    if (!p.pt2pl_layer.empty())
    {
        const auto &m1 = s.pc1.point_layers.at("plane_centroids"),
                   &m2 = s.pc2.point_layers.at(p.pt2pl_layer);
        ASSERT_(m1);
        ASSERT_(m2);

        auto& mp = s.mps.at(p.pt2pl_layer);
        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint =
            mrpt::math::TPoint3D(s.currentSolutionasTPose());

        // Find closest pairings
        mrpt::tfest::TMatchingPairList mpl;
        m1->determineMatching3D(
            m2.get(), s.currentSolution mpl, mp, s.mres[p.pt2pl_layer]);
        // Plane-to-plane correspondence:

        // We have pairs of planes whose centroids are quite close.
        // Check their normals too:
        for (const auto& pair : mpl)
        {
            // 1) Check fo pairing sanity:
            ASSERTDEB_(pair.this_idx < pcs1.planes.size());
            ASSERTDEB_(pair.other_idx < m2->size());

            const auto&           pl_this = s.pc1.planes[pair.this_idx];
            mrpt::math::TPoint3Df pt_other;
            m2->getPoint(pair.other_idx, pt_other.x, pt_other.y, pt_other.z);

            // 2) append to list of plane pairs:
            // Accept pairing:
            pairings.paired_pt2pl.emplace_back(pl_this, pt_other);
        }
    }
}

#endif