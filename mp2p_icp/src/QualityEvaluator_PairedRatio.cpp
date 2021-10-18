/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
    matcher_.initialize(params);
}

double QualityEvaluator_PairedRatio::evaluate(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D&      localPose,
    [[maybe_unused]] const Pairings& pairingsFromICP) const
{
    mp2p_icp::Pairings pairings;

    MatchState ms(pcGlobal, pcLocal);

    matcher_.match(pcGlobal, pcLocal, localPose, {}, ms, pairings);

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
                if (const auto itLy = pcLocal.point_layers.find(localLayerName);
                    itLy != pcLocal.point_layers.end())
                {
                    ASSERT_(itLy->second);
                    nEffectiveLocalPoints += itLy->second->size();
                }
                else
                    THROW_EXCEPTION_FMT(
                        "Could not find point layer '%s' in local "
                        "metric_map_t.",
                        localLayerName.c_str());
            }
        }
    }

    ASSERT_(nEffectiveLocalPoints != 0);

    return pairings.size() / double(nEffectiveLocalPoints);
}
