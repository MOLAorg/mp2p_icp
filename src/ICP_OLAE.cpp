/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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

IMPLEMENTS_MRPT_OBJECT(ICP_OLAE, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_OLAE::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    MRPT_START

    // the global list of pairings:
    s.currentPairings = ICP_Base::runMatchers(s);

    Pairings& pairings = s.currentPairings;

    if (pairings.empty() || pairings.paired_pt2pt.size() < 3)
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
    optimal_tf_olae(pairings, p.pairingsWeightParameters, res);

    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
