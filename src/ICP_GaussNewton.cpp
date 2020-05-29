/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_GaussNewton.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2p_icp/ICP_GaussNewton.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/poses/Lie/SE.h>
#include <Eigen/Dense>

IMPLEMENTS_MRPT_OBJECT(ICP_GaussNewton, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_GaussNewton::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    using namespace std::string_literals;

    MRPT_START

    // the global list of pairings:
    Pairings_GaussNewton pairings;

    // Correspondences for each point layer:
    // ---------------------------------------
    // Find correspondences for each point cloud "layer":
    for (const auto& kv1 : s.pc1.point_layers)
    {
        const auto &m1 = kv1.second, &m2 = s.pc2.point_layers.at(kv1.first);
        ASSERT_(m1);
        ASSERT_(m2);

        // Don't try to match the layer of plane centroids
        if (kv1.first == pointcloud_t::PT_LAYER_PLANE_CENTROIDS) continue;

        // Ignore this layer?
        if (p.weight_pt2pt_layers.count(kv1.first) == 0) continue;

        auto& mp = s.mps.at(kv1.first);

        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint =
            mrpt::math::TPoint3D(s.current_solution.asTPose());

        // Find closest pairings
        mrpt::tfest::TMatchingPairList mpl;
        m1->determineMatching3D(
            m2.get(), out.new_solution, mpl, mp, s.mres[kv1.first]);

        // Shuffle decimated points for next iter:
        if (++mp.offset_other_map_points >= mp.decimation_other_map_points)
            mp.offset_other_map_points = 0;

        // merge lists:
        // layer weight:
        const double lyWeight = p.weight_pt2pt_layers.at(kv1.first);

        // A standard layer: point-to-point correspondences:
        pairings.paired_points.insert(
            pairings.paired_points.end(), mpl.begin(), mpl.end());

        // and their weights:
        pairings.point_weights.emplace_back(mpl.size(), lyWeight);
    }  // end for each layer

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
            mrpt::math::TPoint3D(s.current_solution.asTPose());

        // Find closest pairings
        mrpt::tfest::TMatchingPairList mpl;
        m1->determineMatching3D(
            m2.get(), s.current_solution, mpl, mp, s.mres[p.pt2pl_layer]);
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

    if (pairings.empty() || pairings.paired_points.size() < 3)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // There's no check for this inside olae_match() because it also
        // handled lines, planes, etc. but we don't want to rely on that for
        // this application.

        // Nothing we can do !!
        out.success = false;
        return;
    }

    // Compute the optimal pose, using the GN method
    // ------------------------------------------------
    OptimalTF_Result res;

    pairings.use_robust_kernel = p.use_kernel;
    MRPT_TODO("make param");
    // pairings.robust_kernel_param = mrpt::DEG2RAD(0.05);
    // pairings.robust_kernel_scale = 1500.0;

    optimal_tf_gauss_newton(pairings, res);
    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
