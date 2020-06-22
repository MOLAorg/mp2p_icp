/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Planes_Normals.cpp
 * @brief  Pointcloud matcher: similar planes
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */

// TODO
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
            pairings.paired_planes.emplace_back(p1, p2);
        }
    }
}

#endif