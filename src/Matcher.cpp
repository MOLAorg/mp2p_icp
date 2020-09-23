/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher.cpp
 * @brief  Pointcloud matching generic base class
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */

#include <mp2p_icp/Matcher.h>
#include <mrpt/core/exceptions.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Matcher, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void Matcher::initialize(const mrpt::containers::yaml& params)
{
    if (params.has("runFromIteration"))
        runFromIteration = params["runFromIteration"].as<uint32_t>();
    if (params.has("runUpToIteration"))
        runUpToIteration = params["runUpToIteration"].as<uint32_t>();
}

void Matcher::match(
    const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, const MatchContext& mc,
    Pairings& out) const
{
    if (mc.icpIteration < runFromIteration) return;
    if (runUpToIteration > 0 && mc.icpIteration > runUpToIteration) return;
    impl_match(pcGlobal, pcLocal, localPose, mc, out);
}
