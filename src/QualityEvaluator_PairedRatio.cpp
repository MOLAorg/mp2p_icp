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

#include <mp2p_icp/QualityEvaluator_PairedRatio.h>

IMPLEMENTS_MRPT_OBJECT(
    QualityEvaluator_PairedRatio, QualityEvaluator, mp2p_icp);

using namespace mp2p_icp;

void QualityEvaluator_PairedRatio::initialize(  //
    [[maybe_unused]] const mrpt::containers::Parameters& params)
{
    // None.
}
double QualityEvaluator_PairedRatio::evaluate(
    const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
    [[maybe_unused]] const mrpt::poses::CPose3D& localPose,
    const Pairings&                              finalPairings) const
{
    return finalPairings.size() / (0.5 * (pcGlobal.size() + pcLocal.size()));
}
