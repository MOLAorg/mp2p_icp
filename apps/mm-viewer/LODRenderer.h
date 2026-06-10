/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   mm-viewer/LODRenderer.h
 * @brief  Per-frame LOD traversal and upload budget manager.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 27, 2026
 */
#pragma once

#include <mp2p_icp/render_params.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "LODTree.h"

struct LODRendererConfig
{
    // Additive refinement: each visited node ADDS its own (disjoint) points on
    // top of its ancestors' points.  Crossing a LOD threshold only adds or
    // removes the finest sprinkle of points, so there is no parent/child swap
    // and no visible popping of whole blocks.

    // Hard cap on points rendered per frame; descent simply stops when the
    // budget is exhausted (safe: ancestors already cover the region).
    size_t frameBudgetPoints = 50000000;

    // Hide a subtree when its bbox projects smaller than this (pixels).
    float cullScreenSizePx = 2.0f;

    // Descend into children while the node's bbox projects larger than this
    // (pixels).  With ~16k points per node, a node at this size has a mean
    // on-screen point spacing of descendScreenSizePx/sqrt(16k) ~ px/126.
    float descendScreenSizePx = 600.0f;

    // While the camera moves, descend threshold is scaled by this (> 1 means
    // slightly coarser rendering; with additive refinement only the finest
    // detail drops out, which is barely noticeable).
    float interactingDescendScale = 1.5f;

    // Per-node hysteresis: a node that descended last frame keeps descending
    // until its projected size falls below descend*this (< 1), preventing
    // flicker right at the threshold.
    float descendHysteresis = 0.85f;

    bool interacting      = false;  // set by camera-motion detector
    int  movingFramesHyst = 5;  // frames of hysteresis for interacting flag
};

/** Manages all LOD trees for one loaded metric map.
 *
 *  One instance lives for the duration of the app. Call clear() before loading
 *  a new map so that scene-graph proxies are torn down cleanly.
 *
 *  Thread model:
 *   - addLayer() spawns a background thread that builds the LODTree.
 *   - When the build completes the finished tree is pushed onto m_pending.
 *   - update() (GL thread) splices pending trees into the active map each frame.
 */
class LODRenderer
{
   public:
    LODRenderer() = default;
    ~LODRenderer();
    LODRenderer(const LODRenderer&)            = delete;
    LODRenderer& operator=(const LODRenderer&) = delete;
    LODRenderer(LODRenderer&&)                 = delete;
    LODRenderer& operator=(LODRenderer&&)      = delete;

    /** Start building an LODTree for the given layer in a background thread.
     *  The result is inserted into glVizMap the next time update() runs.
     *  Thread-safe: may be called from any thread. */
    void addLayer(
        const std::string& layerName, const mrpt::maps::CPointsMap& pts,
        mrpt::opengl::CSetOfObjects& glVizMap, float initialPointSize = 2.0f);

    /** Remove all trees and cancel pending background builds.
     *  Must be called before the next loadMapFile() so the previous
     *  scene-node pointers can be released. */
    void clear(mrpt::opengl::CSetOfObjects& glVizMap);

    /** Show or hide the entire LOD layer without touching the tree.
     *  Call this from the layer-checkbox callback. */
    void setLayerVisible(const std::string& layerName, bool visible);

    /** Called each frame (from addLoopCallback) before nanogui repaints.
     *  cam must be win->camera() (a CGlCanvasBase). */
    void update(const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH);

    /** Push updated render params (point size) into all LOD node clouds. */
    void applyRenderParams(
        const std::string& layerName, const mp2p_icp::render_params_point_layer_t& rp);

    /** Recolorize all rep clouds for one layer from a freshly-recolorized source map.
     *  The source map must have the same point count as when the tree was built. */
    void rebuildColors(const std::string& layerName, const mrpt::maps::CPointsMap& pts);

    /** Re-insert a layer's sceneNode into glVizMap after a glVizMap->clear(). */
    void reattachSceneNode(const std::string& layerName, mrpt::opengl::CSetOfObjects& glVizMap);

    /** Returns true if layerName is actively managed by LOD. */
    bool hasLayer(const std::string& layerName) const;

    /** Returns true if a background build is still running for layerName. */
    bool isBuildingLayer(const std::string& layerName) const;

    /** Returns true (and clears the flag) if a layer was spliced in since the
     *  last call.  The caller should trigger a scene rebuild so any full-cloud
     *  GL objects created before the LOD tree was ready are removed. */
    bool takeRebuildNeeded() { return m_rebuildNeeded.exchange(false); }

    LODRendererConfig config;

   private:
    struct LayerEntry
    {
        std::unique_ptr<LODTree> tree;
        float                    pointSize = 2.0f;
        bool                     visible   = true;
    };

    std::map<std::string, LayerEntry> m_layers;

    // Camera-motion detection.
    struct CameraState
    {
        float px = 0, py = 0, pz = 0;
        float az = 0, el = 0;
        float zoom = 0;
    };
    CameraState       m_prevCam;
    bool              m_firstUpdate  = true;
    int               m_movingFrames = 0;
    std::atomic<bool> m_rebuildNeeded{false};

    // Pending trees: built in background, spliced in on GL tick.
    struct PendingLayer
    {
        std::string                  layerName;
        std::unique_ptr<LODTree>     tree;
        mrpt::opengl::CSetOfObjects* glVizMap         = nullptr;
        float                        initialPointSize = 2.0f;
    };
    std::vector<PendingLayer> m_pending;
    mutable std::mutex        m_pendingMutex;

    // Background build threads, each paired with a done flag set by the
    // thread itself just before it exits.  Joined and erased by
    // joinFinishedBuildThreads() or clear().
    struct BuildThread
    {
        std::thread                        thread;
        std::shared_ptr<std::atomic<bool>> done;  // set to true by the thread
    };
    std::vector<BuildThread> m_buildThreads;
    mutable std::mutex       m_buildThreadsMutex;

    // Layers whose build is in progress (name -> done flag).
    std::map<std::string, std::shared_ptr<std::atomic<bool>>> m_building;
    mutable std::mutex                                        m_buildingMutex;

    // Visibility requested for a layer before its tree is ready.
    // Applied when the pending tree is spliced in.
    std::map<std::string, bool> m_pendingVisibility;
    mutable std::mutex          m_pendingVisibilityMutex;

    // Per-frame budget and debug counters.
    struct BudgetState
    {
        size_t frameBudget    = 0;
        size_t usedPoints     = 0;
        size_t nodesShown     = 0;
        size_t nodesCulled    = 0;
        size_t nodesDescended = 0;
    };

    void visit(
        LODNode& node, LODTree& tree, const mrpt::gui::CGlCanvasBase& cam, int viewportW,
        int viewportH, BudgetState& budget);

    /** Hide every visible rep cloud in this subtree (early-outs on the
     *  subtreeShown flag, so repeated calls on hidden subtrees are free). */
    static void hideSubtree(LODNode& node);

    static float approxProjectedDiameter(
        const mrpt::math::TBoundingBoxf& bbox, const mrpt::gui::CGlCanvasBase& cam, int viewportH);

    void joinFinishedBuildThreads();
};
