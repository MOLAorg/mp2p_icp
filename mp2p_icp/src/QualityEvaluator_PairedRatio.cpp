/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_PairedRatio.cpp
 * @brief  Matching quality evaluator (virtual base class)
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */

#include <mp2p_icp/QualityEvaluator_PairedRatio.h>

IMPLEMENTS_MRPT_OBJECT(QualityEvaluator_PairedRatio, QualityEvaluator, mp2p_icp)

using namespace mp2p_icp;

void QualityEvaluator_PairedRatio::initialize(
    const mrpt::containers::yaml& params)
{
    MCP_LOAD_OPT(params, reuse_icp_pairings);
    MCP_LOAD_OPT(params, absolute_minimum_pairing_ratio);

    if (!reuse_icp_pairings)
    {
        // By default, matchers only assign one pairing to each global point.
        // However, in quality assesment, it DOES make sense to count several
        // times the same global point:
        mrpt::containers::yaml p = params;
        if (!p.has("allowMatchAlreadyMatchedGlobalPoints"))
            p["allowMatchAlreadyMatchedGlobalPoints"] = true;

        matcher_.initialize(p);
    }
}

QualityEvaluator::Result QualityEvaluator_PairedRatio::evaluate(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& localPose,
    const Pairings&             pairingsFromICP) const
{
    const mp2p_icp::Pairings* pairings = nullptr;
    mp2p_icp::Pairings        newPairings;

    if (reuse_icp_pairings)
    {
        // Use last pairings:
        pairings = &pairingsFromICP;
    }
    else
    {
        MatchState ms(pcGlobal, pcLocal);
        matcher_.match(pcGlobal, pcLocal, localPose, {}, ms, newPairings);

        pairings = &newPairings;
    }

    const auto nEffectiveLocalPoints = pairings->potential_pairings;

    Result r;
    r.quality = nEffectiveLocalPoints
                    ? pairings->size() / double(nEffectiveLocalPoints)
                    : .0;

    r.hard_discard = r.quality < absolute_minimum_pairing_ratio;

    return r;
}
