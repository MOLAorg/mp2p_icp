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
 * @file   Matcher_NDT_Blend.h
 * @brief  Pointcloud matcher: point to a likelihood-weighted blend of nearby
 *         plane models
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: point to a likelihood-weighted blend of nearby planes.
 *
 * Like Matcher_Point2Plane, this matcher pairs each `local` point against a
 * plane taken from a `global` layer implementing mp2p_icp::NearestPlaneCapable,
 * and emits one point-to-plane pairing per local point, so solvers consume it
 * unchanged.
 *
 * It differs in how the target plane is chosen. Matcher_Point2Plane keeps the
 * single closest candidate, which makes the residual a discontinuous function
 * of the sensor pose: an arbitrarily small pose change can swap the winning
 * candidate and move the residual by the full disagreement between the two.
 * This matcher instead combines all the candidates in the window with weights
 *
 *     w_i = taper(|q - c_i|) * exp(-0.5 * d_i^2 / temperature^2)
 *
 * where `d_i` is the point-to-plane distance to candidate `i`, `c_i` its
 * centroid and `q` the query point. The resulting plane is the weighted mean of
 * the candidate centroids together with the dominant direction of the
 * weighted normal structure tensor.
 *
 * Two properties are deliberate:
 *
 * - `temperature = 0` reproduces Matcher_Point2Plane exactly, by taking the
 *   very same code path, so it is an exact control rather than an approximate
 *   one.
 * - `taper()` reaches zero with zero derivative at `searchRadius`, so a
 *   candidate entering or leaving the window does so with vanishing weight. A
 *   hard cutoff would reintroduce precisely the discontinuity the blend
 *   exists to remove.
 *
 * Normals are accumulated as outer products rather than as vectors. Plane
 * normals recovered by PCA carry an arbitrary sign, and deciding that sign per
 * candidate against the query would itself be a discrete flip, triggered
 * exactly where the candidate matters most. The outer product is invariant to
 * it.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_NDT_Blend : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_NDT_Blend, mp2p_icp)

   public:
    Matcher_NDT_Blend();

    /** Parameters:
     * - `distanceThreshold`: Max. inliers pt-plane distance [meters][mandatory]
     * - `temperature`: Blending scale for the point-plane distance [meters].
     *   `0` (the default) means "keep the closest candidate", i.e. exactly
     *   Matcher_Point2Plane.
     * - `searchRadius`: Radius [meters] around the query point within which
     *   candidates are blended, measured to the candidate centroid. Defaults
     *   to `distanceThreshold`, which is only sensible for maps whose plane
     *   models sit at the query's own scale; for cell-based maps set it to the
     *   cell size or above, or every candidate falls outside the taper.
     * - `smoothCutoff`: Optional (Default=true). Fade weights smoothly to zero
     *   at `searchRadius` instead of truncating there.
     * - `minWeightSum`: Optional. Emit no pairing if the accumulated weight
     *   falls below this.
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    double distanceThreshold = 0.50;
    double temperature       = 0;
    double searchRadius      = 0;
    double minWeightSum      = 0;
    bool   smoothCutoff      = true;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
