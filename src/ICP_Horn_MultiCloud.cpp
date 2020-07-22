/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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

IMPLEMENTS_MRPT_OBJECT(ICP_Horn_MultiCloud, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_Horn_MultiCloud::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    MRPT_START

    // the global list of pairings:
    s.currentPairings = ICP_Base::runMatchers(s);

    // Compute the optimal pose:
    OptimalTF_Result res;

    try
    {
        optimal_tf_horn(s.currentPairings, p.pairingsWeightParameters, res);
    }
    catch (const std::exception& e)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // Nothing we can do:
        out.success = false;
        return;
    }

    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
