/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_OLAE.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2p_icp/ICP_OLAE.h>
#include <mp2p_icp/optimal_tf_olae.h>

IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(ICP_OLAE, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_OLAE::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    MRPT_START
    // the global list of pairings:
    Pairings_OLAE pairings;

    // Correspondences for each point layer:
    // ---------------------------------------
    // Find correspondences for each point cloud "layer":
    for (const auto& kv1 : s.pc1.point_layers)
    {
        const auto &m1 = kv1.second, &m2 = s.pc2.point_layers.at(kv1.first);
        ASSERT_(m1);
        ASSERT_(m2);

        const bool is_layer_of_planes =
            (kv1.first == pointcloud_t::PT_LAYER_PLANE_CENTROIDS);

        // Ignore this layer if it has no weight:
        if (!is_layer_of_planes && p.weight_pt2pt_layers.count(kv1.first) == 0)
            continue;

        auto& mp = s.mps.at(kv1.first);
        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint =
            mrpt::math::TPoint3D(s.current_solution.asTPose());

        // Find closest pairings
        mrpt::tfest::TMatchingPairList mpl;
        m1->determineMatching3D(
            m2.get(), s.current_solution, mpl, mp, s.mres.at(kv1.first));

        // Shuffle decimated points for next iter:
        if (++mp.offset_other_map_points >= mp.decimation_other_map_points)
            mp.offset_other_map_points = 0;

        // merge lists:
        // handle specially the plane-to-plane matching:
        if (!is_layer_of_planes)
        {
            // layer weight:
            const double lyWeight = p.weight_pt2pt_layers.at(kv1.first);

            // A standard layer: point-to-point correspondences:
            pairings.paired_points.insert(
                pairings.paired_points.end(), mpl.begin(), mpl.end());

            // and their weights:
            pairings.point_weights.emplace_back(mpl.size(), lyWeight);
        }
        else
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
                MRPT_TODO("Set threshold parameter");
                if (n2n_ang < mrpt::DEG2RAD(5.0))
                {
                    // Accept pairing:
                    pairings.paired_planes.emplace_back(p1, p2);
                }
            }
        }
    }

    if (pairings.empty())
    {
        // Nothing we can do !!
        out.success = false;
        return;
    }

    // Compute the optimal pose, using the OLAE method
    // (Optimal linear attitude estimator)
    // ------------------------------------------------
    OptimalTF_Result res;

    // Weights: translation => trust points; attitude => trust planes
    pairings.attitude_weights.pl2pl = p.relative_weight_planes_attitude;
    pairings.attitude_weights.pt2pt = 1.0;

    pairings.use_robust_kernel = p.use_kernel;
    MRPT_TODO("make param");
    // pairings.robust_kernel_param = mrpt::DEG2RAD(0.05);
    // pairings.robust_kernel_scale = 1500.0;

    if (pairings.paired_points.size() >= 3)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // There's no check for this inside olae_match() because it also
        // handled lines, planes, etc. but we don't want to rely on that for
        // this application.
        optimal_tf_olae(pairings, res);
    }

    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
