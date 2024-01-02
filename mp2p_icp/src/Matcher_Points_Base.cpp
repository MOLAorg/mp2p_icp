/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_Base.cpp
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */

#include <mp2p_icp/Matcher_Points_Base.h>
#include <mrpt/random/random_shuffle.h>

#include <chrono>
#include <numeric>  // iota
#include <random>

using namespace mp2p_icp;

bool Matcher_Points_Base::impl_match(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D&          localPose,
    [[maybe_unused]] const MatchContext& mc, MatchState& ms,
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
            if (itGlob == weight_pt2pt_layers.end()) continue;

            for (const auto& kv : itGlob->second)
                localLayers[kv.first] = kv.second;
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
                    continue;
                else
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
                THROW_EXCEPTION_FMT(
                    "Local layer map must be a point cloud, but found type "
                    "'%s'",
                    lcLayerMap->GetRuntimeClass()->className);

            const size_t nBefore = out.paired_pt2pt.size();

            // Ensure we have the KD-tree parameters desired by the user:
            if (kdtree_leaf_max_points_.has_value())
            {
                if (auto glLayerPts = mp2p_icp::MapToPointsMap(*glLayer);
                    glLayerPts &&
                    glLayerPts->kdtree_search_params.leaf_max_size !=
                        *kdtree_leaf_max_points_)
                {
                    glLayerPts->kdtree_search_params.leaf_max_size =
                        *kdtree_leaf_max_points_;
                    glLayerPts->mark_as_modified();  // rebuild kd-tree index
                }
            }

            // matcher implementation:
            implMatchOneLayer(
                *glLayer, *lcLayer, localPose, ms, glLayerName, localLayerName,
                out);

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

void Matcher_Points_Base::initialize(const mrpt::containers::yaml& params)
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
            const double      w =
                em.count("weight") != 0 ? em.at("weight").as<double>() : 1.0;

            weight_pt2pt_layers[globalLayer][localLayer] = w;
        }
    }

    maxLocalPointsPerLayer_ =
        params.getOrDefault("maxLocalPointsPerLayer", maxLocalPointsPerLayer_);

    localPointsSampleSeed_ =
        params.getOrDefault("localPointsSampleSeed", localPointsSampleSeed_);

    allowMatchAlreadyMatchedPoints_ = params.getOrDefault(
        "allowMatchAlreadyMatchedPoints", allowMatchAlreadyMatchedPoints_);

    allowMatchAlreadyMatchedGlobalPoints_ = params.getOrDefault(
        "allowMatchAlreadyMatchedGlobalPoints",
        allowMatchAlreadyMatchedGlobalPoints_);

    if (auto val = params.getOrDefault("kdtree_leaf_max_points", 0); val > 0)
        kdtree_leaf_max_points_ = val;

    bounding_box_intersection_check_epsilon_ = params.getOrDefault(
        "bounding_box_intersection_check_epsilon",
        bounding_box_intersection_check_epsilon_);
}

Matcher_Points_Base::TransformedLocalPointCloud
    Matcher_Points_Base::transform_local_to_global(
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, const std::size_t maxLocalPoints,
        const uint64_t localPointsSampleSeed)
{
    MRPT_START
    TransformedLocalPointCloud r;

    const auto lambdaKeepBBox = [&](float x, float y, float z) {
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

    if (maxLocalPoints == 0 || nLocalPoints <= maxLocalPoints)
    {
        // All points:
        r.x_locals.resize(nLocalPoints);
        r.y_locals.resize(nLocalPoints);
        r.z_locals.resize(nLocalPoints);

        for (size_t i = 0; i < nLocalPoints; i++)
        {
            localPose.composePoint(
                lxs[i], lys[i], lzs[i], r.x_locals[i], r.y_locals[i],
                r.z_locals[i]);
            lambdaKeepBBox(r.x_locals[i], r.y_locals[i], r.z_locals[i]);
        }
    }
    else
    {
        // random subset:
        r.idxs.emplace(maxLocalPoints);
        std::iota(r.idxs->begin(), r.idxs->end(), 0);

        const unsigned int seed =
            localPointsSampleSeed != 0
                ? localPointsSampleSeed
                : std::chrono::system_clock::now().time_since_epoch().count();

        mrpt::random::partial_shuffle(
            r.idxs->begin(), r.idxs->end(), std::default_random_engine(seed),
            maxLocalPoints);

        r.x_locals.resize(maxLocalPoints);
        r.y_locals.resize(maxLocalPoints);
        r.z_locals.resize(maxLocalPoints);

        for (size_t ri = 0; ri < maxLocalPoints; ri++)
        {
            const auto i = (*r.idxs)[ri];
            localPose.composePoint(
                lxs[i], lys[i], lzs[i], r.x_locals[ri], r.y_locals[ri],
                r.z_locals[ri]);
            lambdaKeepBBox(r.x_locals[ri], r.y_locals[ri], r.z_locals[ri]);
        }
    }

    return r;
    MRPT_END
}
