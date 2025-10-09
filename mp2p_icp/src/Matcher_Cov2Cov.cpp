/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Matcher_Cov2Cov.cpp
 * @brief  Point-to-point with associated local covariance matcher
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2025
 */

#include <mp2p_icp/Matcher_Cov2Cov.h>
#include <mp2p_icp/NearestPointWithCovCapable.h>

using namespace mp2p_icp;

IMPLEMENTS_MRPT_OBJECT(Matcher_Cov2Cov, Matcher, mp2p_icp)

bool Matcher_Cov2Cov::impl_match(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, [[maybe_unused]] const MatchContext& mc,
    [[maybe_unused]] MatchState& ms, Pairings& out) const
{
    MRPT_START

    out = Pairings();

    // Analyze layer pairs:
    for (const auto& [globalLayerName, localLayerName] : layer_matches)
    {
        auto itLocal = pcLocal.layers.find(localLayerName);
        if (itLocal == pcLocal.layers.end())
        {
            THROW_EXCEPTION_FMT(
                "Local layer '%s' not found trying to matching global layer '%s'",
                localLayerName.c_str(), globalLayerName.c_str());
        }

        auto itGlobal = pcGlobal.layers.find(globalLayerName);
        if (itGlobal == pcLocal.layers.end())
        {
            THROW_EXCEPTION_FMT(
                "Global layer '%s' not found trying to matching local layer '%s'",
                globalLayerName.c_str(), localLayerName.c_str());
        }

        const mrpt::maps::CMetricMap::Ptr& glLayerMap = itGlobal->second;
        ASSERT_(glLayerMap);
        const auto glLayer =
            std::dynamic_pointer_cast<mp2p_icp::NearestPointWithCovCapable>(glLayerMap);
        if (!glLayer)
        {
            THROW_EXCEPTION_FMT(
                "Global layer map must implement mp2p_icp::NearestPointWithCovCapable, but "
                "found type '%s'",
                glLayerMap->GetRuntimeClass()->className);
        }

        const mrpt::maps::CMetricMap::Ptr& lcLayerMap = itLocal->second;
        ASSERT_(lcLayerMap);
        const auto lcLayer =
            std::dynamic_pointer_cast<mp2p_icp::NearestPointWithCovCapable>(lcLayerMap);
        if (!lcLayer)
        {
            THROW_EXCEPTION_FMT(
                "Local layer map must implement mp2p_icp::NearestPointWithCovCapable, but "
                "found type '%s'",
                lcLayerMap->GetRuntimeClass()->className);
        }

        out.potential_pairings += lcLayer->point_count();

        // matcher implementation:
        glLayer->nn_search_cov2cov(*lcLayer, localPose, this->threshold, out.paired_cov2cov);
    }

    return true;
    MRPT_END
}

void Matcher_Cov2Cov::initialize(const mrpt::containers::yaml& params)
{
    Matcher::initialize(params);

    DECLARE_PARAMETER_REQ(params, threshold);
    MCP_LOAD_OPT(params, bounding_box_intersection_check_epsilon);

    if (params.has("layerMatches"))
    {
        auto& p = params["layerMatches"];

        layer_matches.clear();
        ASSERT_(p.isSequence());

        // - {global: "raw", local: "decimated"}
        // - {global: "raw", local: "decimated"}
        // ...

        for (const auto& entry : p.asSequence())
        {
            ASSERT_(entry.isMap());
            const auto& em = entry.asMap();

            ASSERT_(em.count("global"));
            ASSERT_(em.count("local"));

            const std::string globalLayer = em.at("global").as<std::string>();
            const std::string localLayer  = em.at("local").as<std::string>();

            layer_matches.emplace_back(globalLayer, localLayer);
        }
    }
}
