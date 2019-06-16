/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_horn.cpp
 * @brief  Classic Horn's solution for optimal SE(3) transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_horn.h>
#include <mrpt/tfest/se3.h>  // classic Horn method

using namespace mp2p_icp;

void mp2p_icp::optimal_tf_horn(
    const mrpt::tfest::TMatchingPairList& in, OptimalTF_Result& result)
{
    MRPT_START

    result = OptimalTF_Result();

    mrpt::poses::CPose3DQuat out_pose;
    double                   out_scale;
    const bool               force_scale_unity = true;

    mrpt::tfest::se3_l2(in, out_pose, out_scale, force_scale_unity);

    result.optimal_pose = mrpt::poses::CPose3D(out_pose);

    MRPT_END
}
