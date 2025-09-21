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

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

using namespace mp2p_icp;

bool Matcher_Cov2Cov::impl_match(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, [[maybe_unused]] const MatchContext& mc, MatchState& ms,
    Pairings& out) const
{
    MRPT_START

    out = Pairings();

    // Analyze point cloud layers, one by one:
    for (const auto& glLayerKV : pcGlobal.layers)
    {
        const auto& glLayerName = glLayerKV.first;

        // List of local layers to match against (and optional weights)
        std::map<std::string, std::optional<double>> localLayers;

        if (!weight_pt2pt_layers.empty())
        {
            const auto itGlob = weight_pt2pt_layers.find(glLayerName);
            // If we have weights and this layer is not listed, Skip it:
            if (itGlob == weight_pt2pt_layers.end())
            {
                continue;
            }

            for (const auto& kv : itGlob->second)
            {
                localLayers[kv.first] = kv.second;
            }
        }
        else
        {
            // Default: match by identical layer names:
            localLayers[glLayerName] = {};
        }

        for (const auto& localWeight : localLayers)
        {
            const auto& localLayerName = localWeight.first;
            const bool  hasWeight      = localWeight.second.has_value();

            // Look for a matching layer in "local":
            auto itLocal = pcLocal.layers.find(localLayerName);
            if (itLocal == pcLocal.layers.end())
            {
                // Silently ignore it:
                if (!hasWeight)
                {
                    continue;
                }

                THROW_EXCEPTION_FMT(
                    "Local pointcloud layer '%s' not found matching global "
                    "layer '%s'",
                    localLayerName.c_str(), glLayerName.c_str());
            }

            const mrpt::maps::CMetricMap::Ptr& glLayer = glLayerKV.second;
            ASSERT_(glLayer);

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

            const size_t nBefore = out.paired_pt2pt.size();

            // matcher implementation:
            implMatchOneLayer(*glLayer, *lcLayer, localPose, ms, glLayerName, localLayerName, out);

            const size_t nAfter = out.paired_pt2pt.size();

            if (hasWeight && nAfter != nBefore)
            {
                const double w = localWeight.second.value();
                out.point_weights.emplace_back(nAfter - nBefore, w);
            }
        }
    }
    return true;
    MRPT_END
}

void Matcher_Cov2Cov::initialize(const mrpt::containers::yaml& params)
{
    Matcher::initialize(params);

    if (params.has("pointLayerMatches"))
    {
        auto& p = params["pointLayerMatches"];

        weight_pt2pt_layers.clear();
        ASSERT_(p.isSequence());

        // - {global: "raw", local: "decimated", weight: 1.0}
        // - {global: "raw", local: "decimated", weight: 1.0}
        // ...

        for (const auto& entry : p.asSequence())
        {
            ASSERT_(entry.isMap());
            const auto& em = entry.asMap();

            ASSERT_(em.count("global"));
            ASSERT_(em.count("local"));

            const std::string globalLayer = em.at("global").as<std::string>();
            const std::string localLayer  = em.at("local").as<std::string>();
            const double      w = em.count("weight") != 0 ? em.at("weight").as<double>() : 1.0;

            weight_pt2pt_layers[globalLayer][localLayer] = w;
        }
    }

    bounding_box_intersection_check_epsilon_ = params.getOrDefault(
        "bounding_box_intersection_check_epsilon", bounding_box_intersection_check_epsilon_);
}
