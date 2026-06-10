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

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
// Compute eye (camera) world position from MRPT canvas (pointing + az/el/zoom).
// MRPT convention: azimuth rotates around +Z, elevation tilts from XY plane.
struct EyePos
{
    float x = 0, y = 0, z = 0;
};

EyePos computeEyePos(const mrpt::gui::CGlCanvasBase& cam)
{
    constexpr float kDeg2Rad = 3.14159265f / 180.0f;
    const float     az       = cam.getAzimuthDegrees() * kDeg2Rad;
    const float     el       = cam.getElevationDegrees() * kDeg2Rad;
    const float     zoom     = cam.getZoomDistance();

    EyePos e;
    e.x = cam.getCameraPointingX() + zoom * std::cos(el) * std::cos(az);
    e.y = cam.getCameraPointingY() + zoom * std::cos(el) * std::sin(az);
    e.z = cam.getCameraPointingZ() + zoom * std::sin(el);
    return e;
}

// Approximate projected diameter of a bounding box in pixels.
// Distance is measured from the camera EYE to the CLOSEST point of the bbox:
// using the bbox center underestimates the size of large nearby nodes (their
// center can be far away while the cell extends right up to the camera),
// which made them refuse to refine and look blocky.
float calcProjectedDiameter(
    const mrpt::math::TBoundingBoxf& bbox, const mrpt::gui::CGlCanvasBase& cam, int viewportH)
{
    const float dx   = bbox.max.x - bbox.min.x;
    const float dy   = bbox.max.y - bbox.min.y;
    const float dz   = bbox.max.z - bbox.min.z;
    const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);

    const EyePos eye = computeEyePos(cam);
    const float  qx  = std::clamp(eye.x, bbox.min.x, bbox.max.x);
    const float  qy  = std::clamp(eye.y, bbox.min.y, bbox.max.y);
    const float  qz  = std::clamp(eye.z, bbox.min.z, bbox.max.z);

    const float ddx  = qx - eye.x;
    const float ddy  = qy - eye.y;
    const float ddz  = qz - eye.z;
    float       dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);

    // Eye inside (or touching) the bbox: force maximum detail.
    if (dist < 1e-3f)
    {
        dist = 1e-3f;
    }

    const float fovRad  = cam.cameraFOV() * (3.14159265f / 180.0f);
    const float halfTan = std::tan(0.5f * fovRad);

    if (halfTan < 1e-6f)
    {
        return 0.0f;
    }
    return (diag * static_cast<float>(viewportH)) / (2.0f * dist * halfTan);
}

}  // namespace

namespace
{
// Upload rep clouds for the top maxLevel levels of the tree so that a coarse
// preview is always GPU-resident and can be shown immediately during camera
// movement, before any idle-frame lazy uploads have occurred.
void eagerUploadTopLevels(LODNode& root, mrpt::opengl::CSetOfObjects& sceneNode, int maxLevel)
{
    std::vector<LODNode*> stack;
    stack.push_back(&root);
    while (!stack.empty())
    {
        LODNode* node = stack.back();
        stack.pop_back();
        if (!node->glRepPoints || node->gpuResident)
        {
            continue;
        }
        sceneNode.insert(node->glRepPoints);
        node->gpuResident = true;
        node->glRepPoints->setVisibility(false);  // visit() controls actual visibility

        if (static_cast<int>(node->level) < maxLevel)
        {
            for (auto& child : node->children)
            {
                if (child)
                {
                    stack.push_back(child.get());
                }
            }
        }
    }
}
}  // namespace

// ----------------------------------------------------------------------------

LODRenderer::~LODRenderer()
{
    std::lock_guard<std::mutex> lk(m_buildThreadsMutex);
    for (auto& bt : m_buildThreads)
    {
        if (bt.thread.joinable())
        {
            bt.thread.join();
        }
    }
}

// ----------------------------------------------------------------------------

void LODRenderer::addLayer(
    const std::string& layerName, const mrpt::maps::CPointsMap& pts,
    mrpt::opengl::CSetOfObjects& glVizMap, float initialPointSize)
{
    auto done = std::make_shared<std::atomic<bool>>(false);

    {
        std::lock_guard<std::mutex> lk(m_buildingMutex);
        m_building[layerName] = done;
    }

    std::cout << "[LODRenderer] Starting background build for layer '" << layerName << "' ("
              << pts.size() << " pts).\n"
              << std::flush;

    auto* pGlVizMap = &glVizMap;

    std::lock_guard<std::mutex> lkT(m_buildThreadsMutex);
    BuildThread                 bt;
    bt.done   = done;
    bt.thread = std::thread(
        [this, layerName, &pts, pGlVizMap, initialPointSize, done]()
        {
            auto tree = std::make_unique<LODTree>();

            LODTree::BuildParams bp;

            tree->build(
                pts, bp,
                [&layerName](uint32_t depth, uint64_t nodesBuilt)
                {
                    if (nodesBuilt % 1000 == 0)
                    {
                        std::cout << "\r[LODRenderer] '" << layerName << "': " << nodesBuilt
                                  << " nodes (depth " << depth << ")   " << std::flush;
                    }
                });

            std::cout << "\n[LODRenderer] Build done for layer '" << layerName
                      << "': " << tree->totalPoints() << " pts, depth=" << tree->depth() << "\n"
                      << std::flush;

            {
                std::lock_guard<std::mutex> lk(m_pendingMutex);
                PendingLayer                pl;
                pl.layerName        = layerName;
                pl.tree             = std::move(tree);
                pl.glVizMap         = pGlVizMap;
                pl.initialPointSize = initialPointSize;
                m_pending.push_back(std::move(pl));
            }

            done->store(true);
        });
    m_buildThreads.push_back(std::move(bt));
}

// ----------------------------------------------------------------------------

void LODRenderer::clear(mrpt::opengl::CSetOfObjects& /*glVizMap*/)
{
    // Join all background builds before destroying state so that the pts
    // reference captured by the thread lambda remains valid until then.
    {
        std::lock_guard<std::mutex> lk(m_buildThreadsMutex);
        for (auto& bt : m_buildThreads)
        {
            if (bt.thread.joinable())
            {
                bt.thread.join();
            }
        }
        m_buildThreads.clear();
    }

    {
        std::lock_guard<std::mutex> lk(m_buildingMutex);
        m_building.clear();
    }

    m_layers.clear();

    {
        std::lock_guard<std::mutex> lk(m_pendingMutex);
        m_pending.clear();
    }

    {
        std::lock_guard<std::mutex> lk(m_pendingVisibilityMutex);
        m_pendingVisibility.clear();
    }
}

// ----------------------------------------------------------------------------

bool LODRenderer::hasLayer(const std::string& layerName) const
{
    return m_layers.count(layerName) > 0;
}

bool LODRenderer::isBuildingLayer(const std::string& layerName) const
{
    std::lock_guard<std::mutex> lk(m_buildingMutex);
    auto                        it = m_building.find(layerName);
    return it != m_building.end() && !it->second->load();
}

// ----------------------------------------------------------------------------

void LODRenderer::setLayerVisible(const std::string& layerName, bool visible)
{
    auto it = m_layers.find(layerName);
    if (it == m_layers.end())
    {
        // Layer not yet spliced in — store for later application.
        std::lock_guard<std::mutex> lk(m_pendingVisibilityMutex);
        m_pendingVisibility[layerName] = visible;
        return;
    }
    it->second.visible = visible;
    if (it->second.tree)
    {
        it->second.tree->sceneNode()->setVisibility(visible);
    }
}

// ----------------------------------------------------------------------------

void LODRenderer::applyRenderParams(
    const std::string& layerName, const mp2p_icp::render_params_point_layer_t& rp)
{
    auto it = m_layers.find(layerName);
    if (it == m_layers.end() || !it->second.tree)
    {
        return;
    }
    it->second.pointSize = rp.pointSize;

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

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------

void LODRenderer::hideSubtree(LODNode& node)
{
    if (!node.subtreeShown)
    {
        return;
    }
    node.subtreeShown = false;
    node.wasDescended = false;
    if (node.gpuResident && node.glRepPoints)
    {
        node.glRepPoints->setVisibility(false);
    }
    for (auto& child : node.children)
    {
        if (child)
        {
            hideSubtree(*child);
        }
    }
}

void LODRenderer::visit(
    LODNode& node, LODTree& tree, const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH,
    BudgetState& budget)
{
    (void)viewportW;
    (void)tree;

    const float screenSz = approxProjectedDiameter(node.bbox, cam, viewportH);

    // Projected-size cull: hide the whole subtree.
    if (screenSz < config.cullScreenSizePx)
    {
        hideSubtree(node);
        ++budget.nodesCulled;
        return;
    }

    // Additive refinement: ALWAYS show this node's own points; descendants
    // only add finer detail on top, they never replace these points.
    if (node.gpuResident && node.glRepPoints)
    {
        node.glRepPoints->setVisibility(true);
        budget.usedPoints += node.repIndices.size();
        ++budget.nodesShown;
    }
    node.subtreeShown = true;

    float descendThresh = config.descendScreenSizePx;
    if (config.interacting)
    {
        descendThresh *= config.interactingDescendScale;
    }
    if (node.wasDescended)
    {
        descendThresh *= config.descendHysteresis;
    }

    const bool budgetOk = (budget.usedPoints < budget.frameBudget);
    const bool descend  = !node.isLeaf() && budgetOk && (screenSz > descendThresh);

    if (descend)
    {
        node.wasDescended = true;
        ++budget.nodesDescended;
        for (auto& child : node.children)
        {
            if (child)
            {
                visit(*child, tree, cam, viewportW, viewportH, budget);
            }
        }
    }
    else
    {
        node.wasDescended = false;
        for (auto& child : node.children)
        {
            if (child)
            {
                hideSubtree(*child);
            }
        }
    }
}

// ----------------------------------------------------------------------------

void LODRenderer::joinFinishedBuildThreads()
{
    std::lock_guard<std::mutex> lk(m_buildThreadsMutex);
    auto                        it = m_buildThreads.begin();
    while (it != m_buildThreads.end())
    {
        if (it->done->load() && it->thread.joinable())
        {
            it->thread.join();
            it = m_buildThreads.erase(it);
        }
        else if (!it->thread.joinable())
        {
            it = m_buildThreads.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

// ----------------------------------------------------------------------------

void LODRenderer::update(const mrpt::gui::CGlCanvasBase& cam, int viewportW, int viewportH)
{
    // Splice completed trees into the active layer map (GL thread).
    {
        std::lock_guard<std::mutex> lk(m_pendingMutex);
        for (auto& pl : m_pending)
        {
            pl.glVizMap->insert(pl.tree->sceneNode());

            // Apply point size to every node cloud.
            {
                std::vector<LODNode*> stack;
                stack.push_back(&pl.tree->root());
                while (!stack.empty())
                {
                    LODNode* node = stack.back();
                    stack.pop_back();
                    if (node->glRepPoints)
                    {
                        node->glRepPoints->setPointSize(pl.initialPointSize);
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

            LayerEntry entry;
            entry.tree      = std::move(pl.tree);
            entry.pointSize = pl.initialPointSize;
            entry.visible   = true;

            // Apply any pending visibility override.
            {
                std::lock_guard<std::mutex> lkV(m_pendingVisibilityMutex);
                auto                        vit = m_pendingVisibility.find(pl.layerName);
                if (vit != m_pendingVisibility.end())
                {
                    entry.visible = vit->second;
                    m_pendingVisibility.erase(vit);
                }
            }

            entry.tree->sceneNode()->setVisibility(entry.visible);

            // Eagerly upload ALL nodes so every level is GPU-resident.
            // Without this, deeper nodes (level > maxLevel) cannot render
            // when the camera selects them, producing missing regions and
            // a hard step from "coarse only" to "everything visible" instead
            // of progressive refinement.  Total node count for a 40M-point
            // map with default build params is a few thousand — acceptable.
            // TODO: revisit if this hurts load time on huge maps; reinstate
            // an upload budget with a fallback that flips parent visibility
            // back on when a child can't be shown.
            eagerUploadTopLevels(
                entry.tree->root(), *entry.tree->sceneNode(),
                /*maxLevel=*/std::numeric_limits<int>::max());

            m_layers[pl.layerName] = std::move(entry);
            m_rebuildNeeded.store(true);
        }
        m_pending.clear();
    }

    joinFinishedBuildThreads();

    // --- Camera motion detection (epsilon-based) ---
    CameraState cur;
    cur.px   = cam.getCameraPointingX();
    cur.py   = cam.getCameraPointingY();
    cur.pz   = cam.getCameraPointingZ();
    cur.az   = cam.getAzimuthDegrees();
    cur.el   = cam.getElevationDegrees();
    cur.zoom = cam.getZoomDistance();

    // Seed prevCam on the very first frame to avoid a false "moved" event
    // caused by comparing the real camera position against the zero-initialized state.
    if (m_firstUpdate)
    {
        m_prevCam     = cur;
        m_firstUpdate = false;
    }

    constexpr float kEpsPos  = 1e-4f;
    constexpr float kEpsAng  = 1e-3f;
    constexpr float kEpsZoom = 1e-4f;

    const bool moved =
        (std::abs(cur.px - m_prevCam.px) > kEpsPos || std::abs(cur.py - m_prevCam.py) > kEpsPos ||
         std::abs(cur.pz - m_prevCam.pz) > kEpsPos || std::abs(cur.az - m_prevCam.az) > kEpsAng ||
         std::abs(cur.el - m_prevCam.el) > kEpsAng ||
         std::abs(cur.zoom - m_prevCam.zoom) > kEpsZoom);

    const bool wasInteracting = config.interacting;
    if (moved)
    {
        m_movingFrames = config.movingFramesHyst;
    }
    else if (m_movingFrames > 0)
    {
        --m_movingFrames;
    }
    config.interacting = (m_movingFrames > 0);
    m_prevCam          = cur;

    if (config.interacting != wasInteracting)
    {
        std::cout << "[LODRenderer] interacting=" << config.interacting
                  << "  layers=" << m_layers.size() << "\n"
                  << std::flush;
    }

    // --- Per-frame budget ---
    BudgetState budget;
    budget.frameBudget = config.frameBudgetPoints;

    for (auto& [layerName, entry] : m_layers)
    {
        if (!entry.tree || !entry.visible)
        {
            continue;
        }
        visit(entry.tree->root(), *entry.tree, cam, viewportW, viewportH, budget);
    }

    // Throttled debug summary: print once per ~60 frames.
    static uint64_t s_frameCount = 0;
    if ((++s_frameCount % 60) == 0)
    {
        std::cout << "[LODRenderer] frame=" << s_frameCount << " interacting=" << config.interacting
                  << " shown=" << budget.nodesShown << " descended=" << budget.nodesDescended
                  << " culled=" << budget.nodesCulled << " usedPts=" << budget.usedPoints
                  << " budget=" << budget.frameBudget << " layers=" << m_layers.size() << "\n"
                  << std::flush;
    }
}
