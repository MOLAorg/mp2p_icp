/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   mm-viewer/LODTree.h
 * @brief  Octree LOD structure for progressive point cloud rendering.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 27, 2026
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

/** One node of the octree LOD structure (additive refinement).
 *
 *  Each node owns a DISJOINT subsample of the points inside its cell: showing
 *  a node adds detail on top of its ancestors, it never replaces them.  The
 *  union of repIndices over a node and all its descendants is exactly the set
 *  of source points inside the node's cell.
 */
struct LODNode
{
    mrpt::math::TBoundingBoxf bbox;
    uint32_t                  level       = 0;
    uint64_t                  totalPoints = 0;  // count of points in this subtree

    /** GL cloud holding exactly this node's own (disjoint) point subset. */
    mrpt::opengl::CPointCloudColoured::Ptr glRepPoints;

    /** Original point indices (into the source CPointsMap) owned by this node. */
    std::vector<uint32_t> repIndices;

    /** 8 octants; index == nullptr means absent / leaf. */
    std::array<std::unique_ptr<LODNode>, 8> children{};

    // --- per-frame runtime state ---
    bool gpuResident  = false;  // has glRepPoints been inserted into the scene?
    bool subtreeShown = false;  // any rep cloud visible in this subtree?
    bool wasDescended = false;  // descended last frame (for LOD hysteresis)

    bool isLeaf() const
    {
        for (const auto& c : children)
        {
            if (c)
            {
                return false;
            }
        }
        return true;
    }
};

/** Octree-LOD tree for one point-cloud layer.
 *
 *  Build once (off the GL thread) with build(), then use sceneNode() to
 *  obtain the CSetOfObjects that LODRenderer populates each frame.
 */
class LODTree
{
   public:
    struct BuildParams
    {
        // Additive scheme: each inner node keeps pointsPerNode points sampled
        // uniformly from its cell and pushes the rest down to its octants.
        // Leaves keep everything that reaches them.
        size_t   leafPointTarget  = 32000;  // stop splitting when count <= this
        uint32_t maxDepth         = 12;
        size_t   pointsPerNode    = 16000;  // points OWNED per inner node
        bool     useParallelBuild = true;  // TBB if available
    };

    using ProgressCallback = std::function<void(uint32_t depth, uint64_t nodesBuilt)>;

    /** Build the tree from a point map. Safe to run off the GL thread. */
    void build(
        const mrpt::maps::CPointsMap& src, const BuildParams& p, ProgressCallback onProgress);

    /** Build with default parameters and no progress callback. */
    void build(const mrpt::maps::CPointsMap& src) { build(src, BuildParams{}, ProgressCallback{}); }

    LODNode&       root() { return *m_root; }
    const LODNode& root() const { return *m_root; }

    uint64_t totalPoints() const { return m_root ? m_root->totalPoints : 0; }
    uint32_t depth() const { return m_depth; }

    /** CSetOfObjects inserted into glVizMap; LODRenderer manages its children. */
    mrpt::opengl::CSetOfObjects::Ptr sceneNode() const { return m_glRoot; }

    /** Update colors for all rep-set nodes from a freshly recolorized source map.
     *  The source map must have the same number of points as when build() was called.
     */
    void rebuildColors(const mrpt::maps::CPointsMap& src);

   private:
    std::unique_ptr<LODNode>         m_root;
    mrpt::opengl::CSetOfObjects::Ptr m_glRoot;
    uint32_t                         m_depth = 0;
};
