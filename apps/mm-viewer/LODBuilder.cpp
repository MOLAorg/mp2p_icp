/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   mm-viewer/LODBuilder.cpp
 * @brief  Builds an LODTree from a CPointsMap (sequential build).
 * @author Jose Luis Blanco Claraco
 * @date   Apr 27, 2026
 */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <random>

#include "LODTree.h"

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

namespace
{
// Thread-safe counter for progress reporting.
std::atomic<uint64_t> g_nodesBuilt{0};

struct BuildContext
{
    const mrpt::maps::CPointsMap* src    = nullptr;
    const LODTree::BuildParams*   params = nullptr;
    LODTree::ProgressCallback     onProgress;
    std::mt19937                  rng{42};
    uint32_t                      maxDepthSeen = 0;
};

/** Fill glRepPoints of a node from its repIndices.
 *
 *  Colors are read from the source map's named fields when available
 *  (color_r/g/b uint8 or color_rf/gf/bf float). Falls back to red otherwise.
 *  Recolorization via field-based colormaps is applied later by LODRenderer
 *  through mrpt::obs::recolorize3Dpc on each node cloud.
 */
void fillGLCloud(LODNode& node, const mrpt::maps::CPointsMap& src)
{
    const size_t N     = node.repIndices.size();
    auto         cloud = mrpt::opengl::CPointCloudColoured::Create();
    cloud->reserve(N);

    // Check for native color fields.
    const bool hasRGBu8 = src.hasColor_u8();
    const bool hasRGBf  = src.hasColor_f();

    const mrpt::aligned_std_vector<uint8_t>* buf_ru8 = nullptr;
    const mrpt::aligned_std_vector<uint8_t>* buf_gu8 = nullptr;
    const mrpt::aligned_std_vector<uint8_t>* buf_bu8 = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_rf  = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_gf  = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_bf  = nullptr;

    if (hasRGBu8)
    {
        buf_ru8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
        buf_gu8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
        buf_bu8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);
    }
    if (hasRGBf && !(buf_ru8 && buf_gu8 && buf_bu8))
    {
        buf_rf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf);
        buf_gf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf);
        buf_bf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf);
    }

    for (const uint32_t idx : node.repIndices)
    {
        float x = 0;
        float y = 0;
        float z = 0;
        src.getPoint(idx, x, y, z);

        uint8_t cr = 0xff;
        uint8_t cg = 0x00;
        uint8_t cb = 0x00;
        uint8_t ca = 0x80;

        if (buf_ru8 && buf_gu8 && buf_bu8 && idx < buf_ru8->size())
        {
            cr = (*buf_ru8)[idx];
            cg = (*buf_gu8)[idx];
            cb = (*buf_bu8)[idx];
            ca = 0xff;
        }
        else if (buf_rf && buf_gf && buf_bf && idx < buf_rf->size())
        {
            cr = mrpt::f2u8((*buf_rf)[idx]);
            cg = mrpt::f2u8((*buf_gf)[idx]);
            cb = mrpt::f2u8((*buf_bf)[idx]);
            ca = 0xff;
        }

        cloud->insertPoint(mrpt::math::TPointXYZfRGBAu8(x, y, z, cr, cg, cb, ca));
    }
    node.glRepPoints = cloud;
}

// Select a uniform random subsample of 'count' indices from 'pool'.
std::vector<uint32_t> randomSubsample(
    const std::vector<uint32_t>& pool, size_t count, std::mt19937& rng)
{
    if (pool.size() <= count)
    {
        return pool;
    }
    // Fisher-Yates partial shuffle to pick 'count' from pool.
    std::vector<uint32_t> tmp = pool;
    for (size_t i = 0; i < count; ++i)
    {
        std::uniform_int_distribution<size_t> dist(i, tmp.size() - 1);
        std::swap(tmp[i], tmp[dist(rng)]);
    }
    tmp.resize(count);
    return tmp;
}

void buildRecursive(
    std::unique_ptr<LODNode>& nodePtr, std::vector<uint32_t>& indices,
    const mrpt::math::TBoundingBoxf& bbox, uint32_t level, BuildContext& ctx)
{
    nodePtr       = std::make_unique<LODNode>();
    LODNode& node = *nodePtr;

    node.level       = level;
    node.totalPoints = static_cast<uint64_t>(indices.size());
    node.bbox        = bbox;

    if (level > ctx.maxDepthSeen)
    {
        ctx.maxDepthSeen = level;
    }

    const bool isLeaf =
        (indices.size() <= ctx.params->leafPointTarget) || (level >= ctx.params->maxDepth);

    if (isLeaf)
    {
        // Leaf: rep set is all points.
        node.repIndices = std::move(indices);
        fillGLCloud(node, *ctx.src);

        const uint64_t built = ++g_nodesBuilt;
        if (ctx.onProgress && (built % 500 == 0))
        {
            ctx.onProgress(level, built);
        }
        return;
    }

    // Partition indices into 8 octants.
    const mrpt::math::TPoint3Df center(
        0.5f * (bbox.min.x + bbox.max.x), 0.5f * (bbox.min.y + bbox.max.y),
        0.5f * (bbox.min.z + bbox.max.z));

    std::array<std::vector<uint32_t>, 8> childIdx;

    for (const uint32_t i : indices)
    {
        float x = 0;
        float y = 0;
        float z = 0;
        ctx.src->getPoint(i, x, y, z);
        const int ox = (x >= center.x) ? 1 : 0;
        const int oy = (y >= center.y) ? 1 : 0;
        const int oz = (z >= center.z) ? 1 : 0;
        childIdx[ox + 2 * oy + 4 * oz].push_back(i);
    }

    // Free the parent index memory early before recursing.
    indices.clear();
    indices.shrink_to_fit();

    // Build child bboxes and recurse.
    std::array<mrpt::math::TBoundingBoxf, 8> childBBox;
    for (int ox = 0; ox < 2; ++ox)
    {
        for (int oy = 0; oy < 2; ++oy)
        {
            for (int oz = 0; oz < 2; ++oz)
            {
                const int ci  = ox + 2 * oy + 4 * oz;
                childBBox[ci] = mrpt::math::TBoundingBoxf(
                    {ox ? center.x : bbox.min.x, oy ? center.y : bbox.min.y,
                     oz ? center.z : bbox.min.z},
                    {ox ? bbox.max.x : center.x, oy ? bbox.max.y : center.y,
                     oz ? bbox.max.z : center.z});
            }
        }
    }

    // Recurse into children.
#if defined(MP2P_HAS_TBB)
    if (ctx.params->useParallelBuild && level <= 2)
    {
        // Only parallelize top levels to avoid TBB overhead.
        tbb::parallel_for(
            size_t{0}, size_t{8},
            [&](size_t ci)
            {
                if (!childIdx[ci].empty())
                {
                    buildRecursive(node.children[ci], childIdx[ci], childBBox[ci], level + 1, ctx);
                }
            });
    }
    else
#endif
    {
        for (int ci = 0; ci < 8; ++ci)
        {
            if (!childIdx[ci].empty())
            {
                buildRecursive(node.children[ci], childIdx[ci], childBBox[ci], level + 1, ctx);
            }
        }
    }

    // Compute this node's rep set from children's rep sets, drawing from each
    // child proportionally to that child's TOTAL point count (not to its
    // rep-set size).  Sampling uniformly over the pooled rep indices would
    // over-represent children that subdivided more (their rep set equals the
    // leaf cap regardless of how many points actually live there), which
    // produces visible density bias near the root.
    uint64_t totalSubtreePts = 0;
    for (int ci = 0; ci < 8; ++ci)
    {
        if (node.children[ci])
        {
            totalSubtreePts += node.children[ci]->totalPoints;
        }
    }

    node.repIndices.clear();
    if (totalSubtreePts > 0)
    {
        const size_t target = ctx.params->pointsPerNode;
        node.repIndices.reserve(target);
        size_t remaining = target;
        for (int ci = 0; ci < 8; ++ci)
        {
            if (!node.children[ci])
            {
                continue;
            }
            const auto& crep = node.children[ci]->repIndices;
            if (crep.empty())
            {
                continue;
            }
            // Quota for this child, proportional to its total points.
            const double frac =
                static_cast<double>(node.children[ci]->totalPoints) /
                static_cast<double>(totalSubtreePts);
            size_t quota = static_cast<size_t>(std::round(frac * target));
            // Cap by child's rep size and by remaining budget.
            quota = std::min(quota, crep.size());
            quota = std::min(quota, remaining);
            if (quota == 0)
            {
                continue;
            }
            auto picked = randomSubsample(crep, quota, ctx.rng);
            for (uint32_t idx : picked)
            {
                node.repIndices.push_back(idx);
            }
            remaining -= quota;
            if (remaining == 0)
            {
                break;
            }
        }
    }
    fillGLCloud(node, *ctx.src);

    const uint64_t built = ++g_nodesBuilt;
    if (ctx.onProgress && (built % 500 == 0))
    {
        ctx.onProgress(level, built);
    }
}

}  // namespace

void LODTree::build(
    const mrpt::maps::CPointsMap& src, const BuildParams& p, ProgressCallback onProgress)
{
    m_glRoot = mrpt::opengl::CSetOfObjects::Create();
    m_root.reset();
    m_depth = 0;

    const size_t N = src.size();
    if (N == 0)
    {
        return;
    }

    const auto bbox = src.boundingBox();

    std::vector<uint32_t> allIndices(N);
    std::iota(allIndices.begin(), allIndices.end(), 0u);

    g_nodesBuilt.store(0);

    BuildContext ctx;
    ctx.src        = &src;
    ctx.params     = &p;
    ctx.onProgress = onProgress;

    buildRecursive(m_root, allIndices, bbox, 0, ctx);

    m_depth = ctx.maxDepthSeen;

    if (onProgress)
    {
        onProgress(m_depth, g_nodesBuilt.load());
    }
}

void LODTree::rebuildColors(const mrpt::maps::CPointsMap& src)
{
    ASSERT_EQUAL_(src.size(), totalPoints());

    // Gather color buffer references once for the whole map.
    const bool hasRGBu8 = src.hasColor_u8();
    const bool hasRGBf  = src.hasColor_f();

    const mrpt::aligned_std_vector<uint8_t>* buf_ru8 = nullptr;
    const mrpt::aligned_std_vector<uint8_t>* buf_gu8 = nullptr;
    const mrpt::aligned_std_vector<uint8_t>* buf_bu8 = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_rf  = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_gf  = nullptr;
    const mrpt::aligned_std_vector<float>*   buf_bf  = nullptr;

    if (hasRGBu8)
    {
        buf_ru8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
        buf_gu8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
        buf_bu8 = src.getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);
    }
    if (hasRGBf && !(buf_ru8 && buf_gu8 && buf_bu8))
    {
        buf_rf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf);
        buf_gf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf);
        buf_bf = src.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf);
    }

    auto f2u8 = [](float v) -> float { return std::max(0.0f, std::min(1.0f, v)); };

    // DFS traversal to update all node clouds.
    std::vector<LODNode*> stack;
    if (m_root)
    {
        stack.push_back(m_root.get());
    }

    while (!stack.empty())
    {
        LODNode* node = stack.back();
        stack.pop_back();

        if (node->glRepPoints && !node->repIndices.empty())
        {
            auto& cloud = *node->glRepPoints;
            ASSERT_EQUAL_(cloud.size(), node->repIndices.size());

            for (size_t i = 0; i < node->repIndices.size(); ++i)
            {
                const uint32_t srcIdx = node->repIndices[i];

                if (buf_ru8 && buf_gu8 && buf_bu8 && srcIdx < buf_ru8->size())
                {
                    // uint8 RGB: setPointColor_fast expects floats in [0,1].
                    cloud.setPointColor_fast(
                        i, mrpt::u8tof((*buf_ru8)[srcIdx]), mrpt::u8tof((*buf_gu8)[srcIdx]),
                        mrpt::u8tof((*buf_bu8)[srcIdx]), 1.0f);
                }
                else if (buf_rf && buf_gf && buf_bf && srcIdx < buf_rf->size())
                {
                    cloud.setPointColor_fast(
                        i, f2u8((*buf_rf)[srcIdx]), f2u8((*buf_gf)[srcIdx]),
                        f2u8((*buf_bf)[srcIdx]), 1.0f);
                }
                else
                {
                    cloud.setPointColor_fast(i, 1.0f, 0.0f, 0.0f, 0.5f);
                }
            }
            cloud.markAllPointsAsNew();
            cloud.notifyChange();
        }

        for (auto& child : node->children)
        {
            if (child)
            {
                stack.push_back(child.get());
            }
        }
    }
}
