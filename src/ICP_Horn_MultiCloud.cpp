/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Horn_MultiCloud.cpp
 * @brief  ICP registration for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */

#include <mp2p_icp/ICP_Horn_MultiCloud.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(
    ICP_Horn_MultiCloud, mp2p_icp::ICP_Base, mp2p_icp);

using namespace mp2p_icp;

void ICP_Horn_MultiCloud::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    MRPT_START

    ASSERT_EQUAL_(s.pc1.point_layers.size(), s.pc2.point_layers.size());
    ASSERT_(!s.pc1.point_layers.empty());
    const auto nLayers = s.pc1.point_layers.size();
    ASSERT_(nLayers >= 1);

    // the global list of pairings, including all layers:
    mrpt::tfest::TMatchingPairList pairings;

    // Find correspondences for each point cloud "layer":
    for (const auto& kv1 : s.pc1.point_layers)
    {
        // Ignore layers of plane centroids, since they are not actual accurate
        // 3D points for this algorithm.
        if (kv1.first == pointcloud_t::PT_LAYER_PLANE_CENTROIDS) continue;

        const auto &m1 = kv1.second, &m2 = s.pc2.point_layers.at(kv1.first);
        ASSERT_(m1);
        ASSERT_(m2);

        auto& mp = s.mps.at(kv1.first);

        mp.decimation_other_map_points = std::max(
            1U, static_cast<unsigned>(m1->size() / (1.0 * p.maxPairsPerLayer)));

        // Find closest pairings
        mrpt::tfest::TMatchingPairList mpl;
        m1->determineMatching3D(
            m2.get(), s.current_solution, mpl, mp, s.mres[kv1.first]);

        // merge lists:
        pairings.insert(pairings.end(), mpl.begin(), mpl.end());

    }  // end for each "layer"

    if (pairings.empty())
    {
        // Nothing we can do !!
        out.success = false;
        // result.terminationReason = IterTermReason::NoPairings;
        return;
    }

    MRPT_TODO("Port to the new optimal_tf function");

    // Compute the estimated pose, using Horn's method:
    mrpt::poses::CPose3DQuat estPoseQuat;
    double                   transf_scale;
    mrpt::tfest::se3_l2(
        pairings, estPoseQuat, transf_scale,
        true /* DO force rigid transformation (scale=1) */);
    out.new_solution = mrpt::poses::CPose3D(estPoseQuat);
    out.success      = true;

    MRPT_END
}
