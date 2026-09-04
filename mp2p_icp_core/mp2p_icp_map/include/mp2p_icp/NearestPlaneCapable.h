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
 * @file   NearestPlaneCapable.h
 * @brief  Defines a virtual interface for maps capable of finding pt-plane
 * pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2024
 */
#pragma once

#include <mp2p_icp/point_plane_pair_t.h>

#include <functional>
#include <optional>

/** Feature-detection macro for `mp2p_icp::NearestPlaneCapable::nn_visit_pt2pl_candidates()`
 *  and the associated `PlaneCandidate`/`plane_candidate_visitor_t` types. Downstream code
 *  that must build against both this and older mp2p_icp versions can
 *  `#if defined(MP2P_ICP_HAS_NN_VISIT_PT2PL_CANDIDATES)`. */
#define MP2P_ICP_HAS_NN_VISIT_PT2PL_CANDIDATES 1

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** Virtual interface for nearest plane to a point search algorithms */
class NearestPlaneCapable
{
   public:
    NearestPlaneCapable() = default;
    virtual ~NearestPlaneCapable();

    NearestPlaneCapable(const NearestPlaneCapable&)            = default;
    NearestPlaneCapable& operator=(const NearestPlaneCapable&) = default;
    NearestPlaneCapable(NearestPlaneCapable&&)                 = default;
    NearestPlaneCapable& operator=(NearestPlaneCapable&&)      = default;

    struct NearestPlaneResult
    {
        NearestPlaneResult() = default;

        /// Found pairing:
        std::optional<point_plane_pair_t> pairing{};

        /// Absolute value of plane-point distance, if a pairing is found:
        float distance = 0;
    };

    virtual NearestPlaneResult nn_search_pt2pl(
        const mrpt::math::TPoint3Df& point, const float max_search_distance) const = 0;

    /** One of the candidate planes examined while answering nn_search_pt2pl().
     */
    struct PlaneCandidate
    {
        PlaneCandidate() = default;

        /// The candidate plane, with the centroid of its supporting elements:
        point_plane_pair_t pairing{};

        /// Absolute value of the plane-point distance:
        float distance = 0;

        /// Distance from the query point to the plane patch centroid.
        /// Matchers that blend several candidates need this to fade the
        /// contribution of distant ones smoothly to zero: the candidate set
        /// itself is discrete, so without such a fade a candidate entering or
        /// leaving it makes the result jump.
        float centroidDistance = 0;
    };

    using plane_candidate_visitor_t = std::function<void(const PlaneCandidate&)>;

    /** Visits all the plane candidates that nn_search_pt2pl() would examine for
     *  the same query, in an unspecified but deterministic order.
     *
     *  Intended for matchers that combine the candidates instead of selecting
     *  one of them. The default implementation reports at most the single best
     *  candidate, so maps able to expose only an argmin remain usable.
     */
    virtual void nn_visit_pt2pl_candidates(
        const mrpt::math::TPoint3Df& point, float max_search_distance,
        const plane_candidate_visitor_t& visitor) const;
};

/** @} */

}  // namespace mp2p_icp
