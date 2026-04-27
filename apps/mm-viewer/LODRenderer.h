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

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "LODTree.h"

struct LODRendererConfig
{
    size_t frameBudgetPoints  = 5000000;  // max pts rendered per frame
    size_t uploadBudgetPoints = 1000000;  // new GL bytes per frame (idle)
    float  coarseScreenSizePx = 64.0f;  // smaller: use rep points, stop
    float  fineScreenSizePx   = 512.0f;  // larger: descend into children
    bool   interacting        = false;
    float  interactingRatio   = 0.25f;  // shrink budget while moving
};

/** Manages all LOD trees for one loaded metric map.
 *
 *  One instance lives for the duration of the app. Call clear() before loading
 *  a new map so that scene-graph proxies are torn down cleanly.
 */
class LODRenderer
{
   public:
    LODRenderer() = default;

    /** Build an LODTree for the given layer and schedule insertion into
     *  glVizMap on the next GL tick. Thread-safe: build runs on the caller's
     *  thread, insertion happens in update(). */
    void addLayer(
        const std::string& layerName, const mrpt::maps::CPointsMap& pts,
        mrpt::opengl::CSetOfObjects& glVizMap);

    /** Remove all trees and detach their scene nodes from glVizMap. */
    void clear(mrpt::opengl::CSetOfObjects& glVizMap);

    /** Called each frame (from addLoopCallback) before nanogui repaints.
     *
     *  cam must be win->camera() (a CGlCanvasBase).
     */
    void update(const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH);

    /** Push updated render params (point size, colorMode) into all LOD clouds.
     *  Call this whenever the user changes a color/size slider. */
    void applyRenderParams(
        const std::string& layerName, const mp2p_icp::render_params_point_layer_t& rp);

    /** Recolorize all rep clouds for one layer from a freshly-rendered source. */
    void rebuildColors(const std::string& layerName, const mrpt::maps::CPointsMap& pts);

    /** Re-insert a layer's sceneNode into glVizMap after a glVizMap->clear(). */
    void reattachSceneNode(const std::string& layerName, mrpt::opengl::CSetOfObjects& glVizMap);

    /** Returns true if layerName is managed by LOD (i.e. was added via addLayer). */
    bool hasLayer(const std::string& layerName) const;

    LODRendererConfig config;

   private:
    struct LayerEntry
    {
        std::unique_ptr<LODTree> tree;
        float                    pointSize = 2.0f;
        bool                     visible   = true;
    };

    std::map<std::string, LayerEntry> m_layers;

    // Camera-motion detection (previous frame snapshot).
    struct CameraState
    {
        float px = 0, py = 0, pz = 0;  // pointing-at
        float az = 0, el = 0;  // azimuth / elevation deg
        float zoom = 0;
    };
    CameraState m_prevCam;
    int         m_movingFrames = 0;

    // Pending ready trees waiting for GL-thread insertion.
    struct PendingLayer
    {
        std::string                  layerName;
        std::unique_ptr<LODTree>     tree;
        mrpt::opengl::CSetOfObjects* glVizMap = nullptr;
    };
    std::vector<PendingLayer> m_pending;
    std::mutex                m_pendingMutex;

    // Traversal helpers.
    struct BudgetState
    {
        size_t frameBudget  = 0;
        size_t uploadBudget = 0;
        size_t usedPoints   = 0;
        size_t uploaded     = 0;
    };

    void visit(
        LODNode& node, LODTree& tree, const mrpt::gui::CGlCanvasBase& cam, int viewportW,
        int viewportH, BudgetState& budget);

    static float approxProjectedDiameter(
        const mrpt::math::TBoundingBoxf& bbox, const mrpt::gui::CGlCanvasBase& cam, int viewportH);
};
