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
 * @file   test-mp2p_viz_non_finite_points.cpp
 * @brief  Unit tests for get_visualization() against maps holding blank slots
 * @author Jose Luis Blanco Claraco
 * @date   Jul 28, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#include <cmath>
#include <limits>

namespace
{
constexpr size_t NUM_GOOD_POINTS = 100;
constexpr size_t NUM_BLANK_SLOTS = 10;

/** A points map with some slots blanked to NaN, mimicking what map classes
 *  with recycled storage slots (e.g. mola::IncrementalPointCloud) expose
 *  through the inherited CPointsMap buffers.
 */
mrpt::maps::CSimplePointsMap::Ptr mapWithBlankSlots()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    for (size_t i = 0; i < NUM_GOOD_POINTS; i++)
    {
        const auto f = static_cast<float>(i);
        pts->insertPointFast(f, 2.0f * f, 3.0f * f);
    }

    constexpr float kNoPoint = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < NUM_BLANK_SLOTS; i++)
    {
        pts->insertPointFast(kNoPoint, kNoPoint, kNoPoint);
    }
    pts->mark_as_modified();

    ASSERT_EQUAL_(pts->size(), NUM_GOOD_POINTS + NUM_BLANK_SLOTS);

    return pts;
}

void checkAllPointsFinite(const mrpt::opengl::CSetOfObjects& o, size_t expectedCount)
{
    size_t totalPoints = 0;

    for (size_t ith = 0;; ith++)
    {
        auto glPts = o.getByClass<mrpt::opengl::CPointCloudColoured>(ith);
        if (!glPts) break;

        for (size_t i = 0; i < glPts->size(); i++)
        {
            const auto& p = glPts->getPoint3Df(i);
            ASSERTMSG_(
                std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z),
                "A non-finite point reached a CPointCloudColoured render object");
            totalPoints++;
        }
    }

    for (size_t ith = 0;; ith++)
    {
        auto glPts = o.getByClass<mrpt::opengl::CPointCloud>(ith);
        if (!glPts) break;

        for (size_t i = 0; i < glPts->size(); i++)
        {
            const auto& p = (*glPts)[i];
            ASSERTMSG_(
                std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z),
                "A non-finite point reached a CPointCloud render object");
            totalPoints++;
        }
    }

    ASSERT_EQUAL_(totalPoints, expectedCount);
}

/// Uniform color path: metric_map_t::get_visualization() without colorMode.
void test_uniform_color_render()
{
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = mapWithBlankSlots();

    mp2p_icp::render_params_t rp;
    // no rp.points.allLayers.colorMode: exercises the CPointCloud branch

    const auto glObj = map.get_visualization(rp);
    ASSERT_(glObj);

    checkAllPointsFinite(*glObj, NUM_GOOD_POINTS);
}

/// Colorized path: this is the one mola_lidar_odometry's local map viz uses.
void test_colorized_render()
{
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = mapWithBlankSlots();

    mp2p_icp::render_params_t rp;
    auto&                     cm = rp.points.allLayers.colorMode.emplace();
    cm.colorMap                  = mrpt::img::cmHOT;
    cm.recolorizeByField         = "z";

    const auto glObj = map.get_visualization(rp);
    ASSERT_(glObj);

    checkAllPointsFinite(*glObj, NUM_GOOD_POINTS);
}

/// keep_original_cloud_color delegates to the map's own getVisualizationInto(),
/// bypassing the source-map filtering the two paths above rely on.
void test_keep_original_cloud_color_render()
{
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = mapWithBlankSlots();

    mp2p_icp::render_params_t rp;
    auto&                     cm = rp.points.allLayers.colorMode.emplace();
    cm.keep_original_cloud_color = true;

    const auto glObj = map.get_visualization(rp);
    ASSERT_(glObj);

    checkAllPointsFinite(*glObj, NUM_GOOD_POINTS);
}

/// Same path, but with the per-point colors the map itself assigned: the
/// surviving points must keep them, i.e. filtering must not shift colors.
void test_keep_original_cloud_color_preserves_colors()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    // Interleave blank slots with the good points, so that a filtering bug
    // that shifts colors by one slot cannot go unnoticed:
    constexpr float kNoPoint = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < NUM_GOOD_POINTS; i++)
    {
        const auto f = static_cast<float>(i);
        pts->insertPointFast(f, 2.0f * f, 3.0f * f);
        if (i % 3 == 0) pts->insertPointFast(kNoPoint, kNoPoint, kNoPoint);
    }
    pts->mark_as_modified();

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pts;

    mp2p_icp::render_params_t rp;
    auto&                     cm = rp.points.allLayers.colorMode.emplace();
    cm.keep_original_cloud_color = true;

    const auto glObj = map.get_visualization(rp);
    ASSERT_(glObj);

    checkAllPointsFinite(*glObj, NUM_GOOD_POINTS);

    // The survivors must be exactly the good points, in their original order:
    size_t checked = 0;
    for (size_t ith = 0;; ith++)
    {
        auto glPts = glObj->getByClass<mrpt::opengl::CPointCloud>(ith);
        if (!glPts) break;

        for (size_t i = 0; i < glPts->size(); i++)
        {
            const auto& q = (*glPts)[i];
            const auto  f = static_cast<float>(checked);
            ASSERT_EQUAL_(q.x, f);
            ASSERT_EQUAL_(q.y, 2.0f * f);
            ASSERT_EQUAL_(q.z, 3.0f * f);
            checked++;
        }
    }
    for (size_t ith = 0;; ith++)
    {
        auto glPts = glObj->getByClass<mrpt::opengl::CPointCloudColoured>(ith);
        if (!glPts) break;

        for (size_t i = 0; i < glPts->size(); i++)
        {
            const auto& q = glPts->getPoint3Df(i);
            const auto  f = static_cast<float>(checked);
            ASSERT_EQUAL_(q.x, f);
            ASSERT_EQUAL_(q.y, 2.0f * f);
            ASSERT_EQUAL_(q.z, 3.0f * f);
            checked++;
        }
    }
    ASSERT_EQUAL_(checked, NUM_GOOD_POINTS);
}

/// A map with no blank slots at all must be rendered unchanged.
void test_all_finite_is_untouched()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (size_t i = 0; i < NUM_GOOD_POINTS; i++)
    {
        const auto f = static_cast<float>(i);
        pts->insertPointFast(f, 2.0f * f, 3.0f * f);
    }
    pts->mark_as_modified();

    mp2p_icp::metric_map_t map;
    map.layers["raw"] = pts;

    mp2p_icp::render_params_t rp;
    auto&                     cm = rp.points.allLayers.colorMode.emplace();
    cm.colorMap                  = mrpt::img::cmHOT;
    cm.recolorizeByField         = "z";

    const auto glObj = map.get_visualization(rp);
    ASSERT_(glObj);

    checkAllPointsFinite(*glObj, NUM_GOOD_POINTS);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_uniform_color_render();
        test_colorized_render();
        test_keep_original_cloud_color_render();
        test_keep_original_cloud_color_preserves_colors();
        test_all_finite_is_untouched();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
