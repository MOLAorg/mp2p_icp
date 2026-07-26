/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Matcher_Points_Base.cpp
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */

#include <mp2p_icp/Matcher_Points_Base.h>
#include <mrpt/random/random_shuffle.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

using namespace mp2p_icp;

bool Matcher_Points_Base::impl_match(
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

        // List of local layers to match against this global layer:
        std::set<std::string> localLayers;

        if (!pt2pt_layer_matches.empty())
        {
            const auto itGlob = pt2pt_layer_matches.find(glLayerName);
            // If explicit matches are given and this layer is not listed,
            // skip it:
            if (itGlob == pt2pt_layer_matches.end())
            {
                continue;
            }

            localLayers = itGlob->second;
        }
        else
        {
            // Default: match by identical layer names:
            localLayers = {glLayerName};
        }

        for (const auto& localLayerName : localLayers)
        {
            // Look for a matching layer in "local":
            auto itLocal = pcLocal.layers.find(localLayerName);
            if (itLocal == pcLocal.layers.end())
            {
                // Silently ignore it when relying on the default (implicit)
                // same-name matching; explicit entries must exist:
                if (pt2pt_layer_matches.empty())
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
            const auto lcLayer = mp2p_icp::MapToPointsMap(*lcLayerMap);
            if (!lcLayer)
            {
                THROW_EXCEPTION_FMT(
                    "Local layer map must be a point cloud, but found type "
                    "'%s'",
                    lcLayerMap->GetRuntimeClass()->className);
            }

            // Ensure we have the KD-tree parameters desired by the user:
            if (kdtree_leaf_max_points_.has_value())
            {
                if (auto glLayerPts = mp2p_icp::MapToPointsMap(*glLayer);
                    glLayerPts &&
                    glLayerPts->kdtree_search_params.leaf_max_size != *kdtree_leaf_max_points_)
                {
                    glLayerPts->kdtree_search_params.leaf_max_size = *kdtree_leaf_max_points_;
                    glLayerPts->mark_as_modified();  // rebuild kd-tree index
                }
            }

            // matcher implementation:
            implMatchOneLayer(*glLayer, *lcLayer, localPose, ms, glLayerName, localLayerName, out);
        }
    }
    return true;
    MRPT_END
}

void Matcher_Points_Base::initialize(const mrpt::containers::yaml& params)
{
    Matcher::initialize(params);

    if (params.has("pointLayerMatches"))
    {
        auto& p = params["pointLayerMatches"];

        pt2pt_layer_matches.clear();
        ASSERT_(p.isSequence());

        // - {global: "raw", local: "decimated"}
        // - {global: "raw", local: "decimated"}
        // ...
        // Note: a "weight" key is also accepted for backward compatibility
        // with older configuration files, but is ignored: per-layer pt2pt
        // weighting was removed, use PairWeights::pt2pt instead.

        for (const auto& entry : p.asSequence())
        {
            ASSERT_(entry.isMap());
            const auto& em = entry.asMap();

            ASSERT_(em.count("global"));
            ASSERT_(em.count("local"));

            const std::string globalLayer = em.at("global").as<std::string>();
            const std::string localLayer  = em.at("local").as<std::string>();

            pt2pt_layer_matches[globalLayer].insert(localLayer);
        }
    }

    allowMatchAlreadyMatchedPoints_ =
        params.getOrDefault("allowMatchAlreadyMatchedPoints", allowMatchAlreadyMatchedPoints_);

    allowMatchAlreadyMatchedGlobalPoints_ = params.getOrDefault(
        "allowMatchAlreadyMatchedGlobalPoints", allowMatchAlreadyMatchedGlobalPoints_);

    if (auto val = params.getOrDefault("kdtree_leaf_max_points", 0); val > 0)
    {
        kdtree_leaf_max_points_ = val;
    }

    bounding_box_intersection_check_epsilon_ = params.getOrDefault(
        "bounding_box_intersection_check_epsilon", bounding_box_intersection_check_epsilon_);
}

Matcher_Points_Base::TransformedLocalPointCloud Matcher_Points_Base::transform_local_to_global(
    const mrpt::maps::CPointsMap& pcLocal, const mrpt::poses::CPose3D& localPose)
{
    MRPT_START
    TransformedLocalPointCloud r;

    const auto lambdaKeepBBox = [&](float x, float y, float z)
    {
        mrpt::keep_max(r.localMax.x, x);
        mrpt::keep_max(r.localMax.y, y);
        mrpt::keep_max(r.localMax.z, z);

        mrpt::keep_min(r.localMin.x, x);
        mrpt::keep_min(r.localMin.y, y);
        mrpt::keep_min(r.localMin.z, z);
    };

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    const size_t nLocalPoints = pcLocal.size();

    r.x_locals.resize(nLocalPoints);
    r.y_locals.resize(nLocalPoints);
    r.z_locals.resize(nLocalPoints);

#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        static_cast<size_t>(0), nLocalPoints,
        [&](size_t i)
        {
#else
    for (size_t i = 0; i < nLocalPoints; i++)
    {
#endif
            localPose.composePoint(
                lxs[i], lys[i], lzs[i], r.x_locals[i], r.y_locals[i], r.z_locals[i]);
        }
#if defined(MP2P_HAS_TBB)
    );
#endif

    for (size_t i = 0; i < nLocalPoints; i++)
    {
        lambdaKeepBBox(r.x_locals[i], r.y_locals[i], r.z_locals[i]);
    }

    return r;
    MRPT_END
}
