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
 * @file   FilterPlanePatches.h
 * @brief  Extracts large planar patches from a point layer into `planes`
 * @author Jose Luis Blanco Claraco
 * @date   Sep 1, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Extracts large planar patches from a point layer and appends them to
 *  `metric_map_t::planes`, each with the surface area it was fitted from.
 *
 * Unlike FilterEdgesPlanes, which labels individual *points* as planar in the
 * LOAM sense, this produces actual patches: a plane equation, a centroid, and
 * an extent. That is what an estimator needs when it has to weigh one surface
 * against another, e.g. to read gravity off the assumption that large patches
 * in a built environment are plumb or level.
 *
 * The extraction is greedy and seeded by per-point normals: voxel-downsample,
 * estimate a normal for every remaining point once from its k nearest
 * neighbors, then repeatedly take the seed whose plane holds the most points,
 * refit that plane by PCA on its inliers, and remove them. A patch is kept
 * only if its smaller in-plane extent reaches `min_span`, which is what
 * separates a wall from a sliver fitted across a cluttered surface.
 *
 * Deterministic by construction: the seeds are a fixed stride through the
 * surviving points rather than a random sample, so two runs over the same
 * scan produce byte-identical patches regardless of thread count or of how
 * the work was scheduled.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterPlanePatches : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterPlanePatches, mp2p_icp_filters)
   public:
    FilterPlanePatches();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterPlanePatches& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** If true, `planes` is emptied before appending. Leave it on for a
         *  per-scan observation map, whose patches describe only this scan. */
        bool clear_previous = true;

        /** Downsampling applied before anything else [m]. It also sets the
         *  area credited to a patch: one point stands for one voxel face. */
        double voxel_size = 0.10;

        /** Maximum point-to-plane distance for an inlier [m]. */
        double distance_threshold = 0.06;

        /** Maximum angle between a point's own normal and the candidate
         *  plane's [deg]. Rejects points that merely lie near the plane
         *  while belonging to a surface crossing it. */
        double normal_agreement_deg = 12.0;

        /** Minimum inlier count for a patch. With the default voxel size, 300
         *  points is 3 m2. */
        uint32_t min_points = 300;

        /** Minimum extent along the *shorter* in-plane axis [m]. */
        double min_span = 1.5;

        /** Points outside this range from the layer's origin are ignored [m].
         *  The near limit drops the vehicle body, the far one drops returns
         *  whose angular resolution no longer supports a plane fit. */
        double range_min = 1.0;
        double range_max = 60.0;

        /** Stop after this many patches. */
        uint32_t max_patches = 12;

        /** Neighbors used to estimate each point's normal. */
        uint32_t normal_knn = 13;

        /** How many seeds to try per patch. Cost is linear in this. */
        uint32_t seed_candidates = 120;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
