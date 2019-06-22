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
#include <mp2p_icp/optimal_tf_horn.h>
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

    // the global list of pairings:
    WeightedPairings pairings = ICP_Base::commonFindPairings(s, p);

    if (pairings.empty() && pairings.paired_points.size() < 3)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // There's no check for this inside olae_match() because it also
        // handled lines, planes, etc. but we don't want to rely on that for
        // this application.

        // Note: this condition could be refined to check for minimal sets of
        // well-defined problems, like 2 points and one plane, etc.

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

    pairings.use_scale_outlier_detector = p.use_scale_outlier_detector;
    pairings.scale_outlier_threshold    = p.scale_outlier_threshold;
    pairings.use_robust_kernel          = p.use_kernel;
    MRPT_TODO("make param");
    // pairings.robust_kernel_param = mrpt::DEG2RAD(0.05);
    // pairings.robust_kernel_scale = 1500.0;

    optimal_tf_horn(pairings, res);

    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
