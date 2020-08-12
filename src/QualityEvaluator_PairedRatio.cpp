/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_PairedRatio.cpp
 * @brief  Matching quality evaluator (virtual base class)
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/QualityEvaluator_PairedRatio.h>

IMPLEMENTS_MRPT_OBJECT(
    QualityEvaluator_PairedRatio, QualityEvaluator, mp2p_icp);

using namespace mp2p_icp;

void QualityEvaluator_PairedRatio::initialize(  //
    const mrpt::containers::Parameters& params)
{
    MCP_LOAD_REQ(params, thresholdDistance);
}

double QualityEvaluator_PairedRatio::evaluate(
    const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
    const mrpt::poses::CPose3D&      localPose,
    [[maybe_unused]] const Pairings& finalPairings) const
{
    mp2p_icp::Pairings                         pairings;
    mp2p_icp::Matcher_Points_DistanceThreshold matcher(thresholdDistance);

    matcher.match(pcGlobal, pcLocal, localPose, {}, pairings);

    return pairings.size() / (0.5 * (pcGlobal.size() + pcLocal.size()));
}
