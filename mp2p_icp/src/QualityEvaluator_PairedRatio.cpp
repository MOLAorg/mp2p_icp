/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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
    // By default, matchers only assign one pairing to each global point.
    // However, in quality assesment, it DOES make sense to count several times
    // the same global point:
    mrpt::containers::yaml p = params;
    if (!p.has("allowMatchAlreadyMatchedGlobalPoints"))
        p["allowMatchAlreadyMatchedGlobalPoints"] = true;

    MCP_LOAD_OPT(params, reuse_icp_pairings);

    // Even if reuse_icp_pairings==true, we need the matcher to check its
    // weight_pt2pt_layers:
    matcher_.initialize(p);
}

double QualityEvaluator_PairedRatio::evaluate(
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

    // The ratio must be accounted for using the number of points in
    // the active layers:
    size_t nEffectiveLocalPoints = 0;
    if (matcher_.weight_pt2pt_layers.empty())
    {
        // all layers:
        nEffectiveLocalPoints = pcLocal.size_points_only();
    }
    else
    {
        // only selected ones:
        for (const auto& p : matcher_.weight_pt2pt_layers)
        {
            for (const auto& kv : p.second)
            {
                const auto& localLayerName = kv.first;
                nEffectiveLocalPoints +=
                    pcLocal.point_layer(localLayerName)->size();
            }
        }
    }

    ASSERT_(nEffectiveLocalPoints != 0);

    return pairings->size() / double(nEffectiveLocalPoints);
}
