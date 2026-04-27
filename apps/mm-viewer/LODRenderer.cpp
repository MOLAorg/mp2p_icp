/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   mm-viewer/LODRenderer.cpp
 * @brief  Per-frame LOD traversal and upload budget manager.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 27, 2026
 */

#include "LODRenderer.h"

#include <mrpt/core/exceptions.h>

#include <cmath>

namespace
{
// Compute a crude "projected diameter in pixels" for a bounding box.
// Uses the standard pinhole approximation and the camera zoom distance.
float calcProjectedDiameter(
    const mrpt::math::TBoundingBoxf& bbox, const mrpt::gui::CGlCanvasBase& cam, int viewportH)
{
    // Diagonal length of the bbox (world units).
    const float dx   = bbox.max.x - bbox.min.x;
    const float dy   = bbox.max.y - bbox.min.y;
    const float dz   = bbox.max.z - bbox.min.z;
    const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Camera pointing-at gives an approximate distance from bbox center.
    const float cx = 0.5f * (bbox.min.x + bbox.max.x);
    const float cy = 0.5f * (bbox.min.y + bbox.max.y);
    const float cz = 0.5f * (bbox.min.z + bbox.max.z);

    const float ddx  = cx - cam.getCameraPointingX();
    const float ddy  = cy - cam.getCameraPointingY();
    const float ddz  = cz - cam.getCameraPointingZ();
    float       dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);

    // Add zoom distance so we don't divide by zero when inside the bbox.
    dist += cam.getZoomDistance();

    const float fovRad  = cam.cameraFOV() * (3.14159265f / 180.0f);
    const float halfTan = std::tan(0.5f * fovRad);

    if (halfTan < 1e-6f || dist < 1e-6f)
    {
        return 0.0f;
    }
    return (diag * static_cast<float>(viewportH)) / (2.0f * dist * halfTan);
}

}  // namespace

// ----------------------------------------------------------------------------

void LODRenderer::addLayer(
    const std::string& layerName, const mrpt::maps::CPointsMap& pts,
    mrpt::opengl::CSetOfObjects& glVizMap)
{
    auto tree = std::make_unique<LODTree>();

    LODTree::BuildParams bp;

    std::cout << "[LODRenderer] Building LODTree for layer '" << layerName << "' (" << pts.size()
              << " pts)..." << std::flush;

    tree->build(
        pts, bp,
        [&layerName](uint32_t depth, uint64_t nodesBuilt)
        {
            std::cout << "\r[LODRenderer] Layer '" << layerName << "': " << nodesBuilt
                      << " nodes built (depth " << depth << ")   " << std::flush;
        });

    std::cout << "\n[LODRenderer] LODTree done for layer '" << layerName
              << "': " << tree->totalPoints() << " pts, depth=" << tree->depth() << "\n";

    // Queue the finished tree for GL-thread insertion.
    std::lock_guard<std::mutex> lk(m_pendingMutex);
    PendingLayer                pl;
    pl.layerName = layerName;
    pl.tree      = std::move(tree);
    pl.glVizMap  = &glVizMap;
    m_pending.push_back(std::move(pl));
}

void LODRenderer::clear(mrpt::opengl::CSetOfObjects& glVizMap)
{
    // Remove each layer's sceneNode from glVizMap.
    // CSetOfObjects does not expose individual removal; we rebuild after clearing.
    // We just call glVizMap.clear() from the caller; here we clean local state.
    (void)glVizMap;

    m_layers.clear();

    {
        std::lock_guard<std::mutex> lk(m_pendingMutex);
        m_pending.clear();
    }
}

bool LODRenderer::hasLayer(const std::string& layerName) const
{
    return m_layers.count(layerName) > 0;
}

void LODRenderer::applyRenderParams(
    const std::string& layerName, const mp2p_icp::render_params_point_layer_t& rp)
{
    auto it = m_layers.find(layerName);
    if (it == m_layers.end())
    {
        return;
    }
    it->second.pointSize = rp.pointSize;
    // Propagate point size to all node clouds (DFS).
    if (!it->second.tree)
    {
        return;
    }
    std::vector<LODNode*> stack;
    stack.push_back(&it->second.tree->root());
    while (!stack.empty())
    {
        LODNode* node = stack.back();
        stack.pop_back();
        if (node->glRepPoints)
        {
            node->glRepPoints->setPointSize(rp.pointSize);
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

void LODRenderer::reattachSceneNode(
    const std::string& layerName, mrpt::opengl::CSetOfObjects& glVizMap)
{
    auto it = m_layers.find(layerName);
    if (it == m_layers.end() || !it->second.tree)
    {
        return;
    }
    glVizMap.insert(it->second.tree->sceneNode());
}

void LODRenderer::rebuildColors(const std::string& layerName, const mrpt::maps::CPointsMap& pts)
{
    auto it = m_layers.find(layerName);
    if (it == m_layers.end() || !it->second.tree)
    {
        return;
    }
    it->second.tree->rebuildColors(pts);
}

// ----------------------------------------------------------------------------

float LODRenderer::approxProjectedDiameter(
    const mrpt::math::TBoundingBoxf& bbox, const mrpt::gui::CGlCanvasBase& cam, int viewportH)
{
    return calcProjectedDiameter(bbox, cam, viewportH);
}

void LODRenderer::visit(
    LODNode& node, LODTree& tree, const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH,
    BudgetState& budget)
{
    (void)viewportW;

    if (!node.glRepPoints)
    {
        return;
    }

    // Frustum cull: rough AABB check against camera distance.
    // (Full frustum cull requires projection matrices; use distance heuristic for now.)

    const float screenSz = approxProjectedDiameter(node.bbox, cam, viewportH);

    const bool  interacting  = config.interacting;
    const float coarseThresh = config.coarseScreenSizePx;
    const float fineThresh   = config.fineScreenSizePx;

    const bool shouldDescend =
        !node.isLeaf() && (screenSz >= fineThresh) &&
        (budget.usedPoints + static_cast<size_t>(node.totalPoints) < budget.frameBudget);

    if (!shouldDescend || screenSz < coarseThresh)
    {
        // Render this node's rep cloud.
        if (!node.gpuResident)
        {
            // Upload when not interacting and we have upload budget.
            if (!interacting && budget.uploaded + node.repIndices.size() <= budget.uploadBudget)
            {
                tree.sceneNode()->insert(node.glRepPoints);
                node.gpuResident = true;
                budget.uploaded += node.repIndices.size();
            }
            else
            {
                // Not yet resident -- skip this node this frame.
                return;
            }
        }
        node.glRepPoints->setVisibility(true);
        budget.usedPoints += node.repIndices.size();

        // Hide all children below this node.
        std::vector<LODNode*> hideStack;
        for (auto& child : node.children)
        {
            if (child)
            {
                hideStack.push_back(child.get());
            }
        }
        while (!hideStack.empty())
        {
            LODNode* n = hideStack.back();
            hideStack.pop_back();
            if (n->glRepPoints && n->gpuResident)
            {
                n->glRepPoints->setVisibility(false);
            }
            for (auto& child : n->children)
            {
                if (child)
                {
                    hideStack.push_back(child.get());
                }
            }
        }
        return;
    }

    // Descend: hide this node's rep cloud and recurse into children.
    if (node.gpuResident)
    {
        node.glRepPoints->setVisibility(false);
    }

    for (auto& child : node.children)
    {
        if (child)
        {
            visit(*child, tree, cam, viewportW, viewportH, budget);
        }
    }
}

void LODRenderer::update(const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH)
{
    // Splice in any trees that finished building since last frame.
    {
        std::lock_guard<std::mutex> lk(m_pendingMutex);
        for (auto& pl : m_pending)
        {
            // Insert the layer's scene node into glVizMap.
            pl.glVizMap->insert(pl.tree->sceneNode());

            LayerEntry entry;
            entry.tree             = std::move(pl.tree);
            entry.pointSize        = 2.0f;
            entry.visible          = true;
            m_layers[pl.layerName] = std::move(entry);
        }
        m_pending.clear();
    }

    // Camera motion detection.
    CameraState cur;
    cur.px   = cam.getCameraPointingX();
    cur.py   = cam.getCameraPointingY();
    cur.pz   = cam.getCameraPointingZ();
    cur.az   = cam.getAzimuthDegrees();
    cur.el   = cam.getElevationDegrees();
    cur.zoom = cam.getZoomDistance();
    // Note: CGlCanvasBase::getAzimuthDegrees / getElevationDegrees / getZoomDistance
    // are available in mrpt 2.x CGlCanvasBase.

    const bool moved =
        (cur.px != m_prevCam.px || cur.py != m_prevCam.py || cur.pz != m_prevCam.pz ||
         cur.az != m_prevCam.az || cur.el != m_prevCam.el || cur.zoom != m_prevCam.zoom);

    if (moved)
    {
        m_movingFrames = 3;
    }
    else if (m_movingFrames > 0)
    {
        --m_movingFrames;
    }
    config.interacting = (m_movingFrames > 0);
    m_prevCam          = cur;

    // Per-frame budget.
    BudgetState budget;
    budget.frameBudget =
        config.interacting ? static_cast<size_t>(config.frameBudgetPoints * config.interactingRatio)
                           : config.frameBudgetPoints;
    budget.uploadBudget = config.interacting ? 0 : config.uploadBudgetPoints;
    budget.usedPoints   = 0;
    budget.uploaded     = 0;

    for (auto& [layerName, entry] : m_layers)
    {
        if (!entry.tree || !entry.visible)
        {
            continue;
        }
        visit(entry.tree->root(), *entry.tree, cam, viewportW, viewportH, budget);
    }
}
