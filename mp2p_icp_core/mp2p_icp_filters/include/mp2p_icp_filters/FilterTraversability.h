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
 * @file   FilterTraversability.h
 * @brief  2D obstacles for ground vehicles from a 3D point cloud (2.5D ground model)
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Finds the obstacles for a ground vehicle in a 3D point cloud (e.g. a whole map):
 *  steep slopes, steps and drop-offs, and anything at the height of the vehicle
 *  body above the local ground. Gentle slopes remain traversable.
 *
 * The input cloud (map frame, z up) is analyzed on a 2D grid of `resolution`:
 *  1. **Ground model**: the lowest point of each cell is its ground, if it is not too
 *     far above the lowest points around it (`ground_envelope_size`,
 *     `ground_envelope_slope_deg`): cells where only treetops, wall or table tops
 *     were seen get their ground interpolated from neighboring ground, up to
 *     `ground_fill_distance`. Never observed cells within that distance get
 *     interpolated ground too, so sparse ground stays continuous: the edge of a
 *     drop-off whose lower side was never seen is placed up to
 *     `ground_fill_distance` past the real edge (reduce it if that matters).
 *  2. **Per cell checks**: a cell is not traversable if the slope of a plane fitted to
 *     the ground within `slope_window_radius` exceeds `max_slope_deg`, if the
 *     ground height changes by more than `max_step` to a neighbor cell, or if it
 *     has at least `body_min_points` points between `body_min_height` and
 *     `body_max_height` above its ground (overhangs higher than the vehicle are
 *     ignored).
 *  3. **Reachability** (only if `seeds_pointcloud_layer` is given, e.g. the
 *     trajectory followed while mapping, see FilterInsertPoint): the traversable
 *     region is grown from the seeds, which are traversable by definition within
 *     `seeds_radius`, without crossing steps. The obstacles are then the cells
 *     around the reachable region: its border (including never observed cells if
 *     `unknown_is_obstacle`, e.g. the hidden side of a drop-off) and all non
 *     traversable cells up to `obstacle_distance` from it.
 *     Without seeds, all non traversable observed cells are obstacles.
 *
 * Outputs are points at the cell centers, at their ground height:
 * `output_layer_obstacles` and, optionally, `output_layer_traversable` (the
 * reachable cells, or the traversable ones without seeds).
 *
 * The grid is dense over the bounding box of the input: use a coarser resolution
 * for very large maps.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterTraversability : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterTraversability, mp2p_icp_filters)
   public:
    FilterTraversability();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterTraversability& parent);

        /** Input 3D point cloud, typically without dynamic objects */
        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Optional layer with seed points known to be traversable (e.g. the
         *  vehicle trajectory). Enables the reachability analysis. */
        std::string seeds_pointcloud_layer;

        /** Output layer for the obstacle cells */
        std::string output_layer_obstacles;

        /** Optional output layer for the traversable (reachable) cells */
        std::string output_layer_traversable;

        /** Grid cell size [m] */
        float resolution = 0.20f;

        /** Maximum traversable slope [deg] */
        float max_slope_deg = 15.0f;

        /** Half size of the window to fit the ground plane for the slope [m] */
        float slope_window_radius = 0.40f;

        /** Maximum ground height change between neighbor cells [m] */
        float max_step = 0.15f;

        /** Height band of the vehicle body above the ground [m] */
        float body_min_height = 0.25f;
        float body_max_height = 1.20f;

        /** Minimum points in the body band to mark a cell as obstacle */
        uint32_t body_min_points = 2;

        /** Window for the lower envelope of the ground [m] */
        float ground_envelope_size = 1.0f;

        /** Slope tolerated within the ground envelope window [deg] */
        float ground_envelope_slope_deg = 30.0f;

        /** Maximum distance to interpolate ground into cells where it was not
         *  seen (only non-ground points, or no points at all) [m] */
        float ground_fill_distance = 1.0f;

        /** Radius around each seed point that is traversable [m] */
        float seeds_radius = 0.30f;

        /** Whether never observed cells bordering the reachable region are obstacles */
        bool unknown_is_obstacle = true;

        /** Non traversable cells up to this distance from the reachable region are
         *  obstacles [m] */
        float obstacle_distance = 2.0f;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
