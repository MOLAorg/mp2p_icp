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
 * @file   Matcher_Points_Blend.h
 * @brief  Pointcloud matcher: point to a distance-weighted blend of nearby
 *         map points
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: point to a distance-weighted blend of nearby map points.
 *
 * This is the point-based counterpart of Matcher_NDT_Blend. Like
 * Matcher_Points_DistanceThreshold it emits one point-to-point pairing per
 * local point, so solvers consume it unchanged, but the target is not the
 * single nearest map point: it is the weighted mean
 *
 *     m(q) = sum_i w_i p_i / sum_i w_i,
 *     w_i  = taper(|q - p_i|) * exp(-0.5 * |q - p_i|^2 / temperature^2)
 *
 * over the map points within `searchRadius` of the query `q`.
 *
 * The motivation is that a nearest-neighbor target is a discontinuous function
 * of the sensor pose: as the query crosses the perpendicular bisector between
 * two map points the winner swaps and the residual moves by the full
 * separation between them. The weighted mean crosses that bisector
 * continuously.
 *
 * Three properties are deliberate:
 *
 * - `temperature = 0` runs the very same nn_single_search() query as
 *   Matcher_Points_DistanceThreshold with `pairingsPerPoint: 1`, so it
 *   reproduces it exactly and is an exact control rather than an approximate
 *   one.
 * - The neighborhood is a radius query, never a fixed-k one. A map point
 *   entering or leaving a top-k list is itself a hard flip, and a harder one
 *   than the nearest-neighbor swap this matcher exists to remove, since `k` is
 *   a count rather than a geometric boundary.
 * - `taper()` reaches zero with zero derivative at `searchRadius`, so a map
 *   point entering or leaving the neighborhood does so with vanishing weight.
 *
 * A known cost, stated because it is intrinsic rather than incidental: a
 * weighted mean of points sampled from a surface shrinks toward the interior
 * of the neighborhood. At large `temperature` the target tends to the local
 * centroid rather than to the surface, which biases the residual. The
 * temperature should be chosen against the map's point spacing.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_Blend : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Points_Blend, mp2p_icp)

   public:
    Matcher_Points_Blend();

    /** Parameters:
     * - `threshold`: Max. distance between the local point and the blended
     *   target [meters][mandatory]
     * - `thresholdAngularDeg`: Additional range-proportional term added to
     *   `threshold`, as in Matcher_Points_DistanceThreshold [degrees][mandatory]
     * - `temperature`: Blending scale for the point-to-point distance [meters].
     *   `0` (the default) means "keep the nearest map point", i.e. exactly
     *   Matcher_Points_DistanceThreshold with `pairingsPerPoint: 1`.
     * - `searchRadius`: Radius [meters] around the query point within which map
     *   points are blended. Defaults to `threshold`.
     * - `smoothCutoff`: Optional (Default=true). Fade weights smoothly to zero
     *   at `searchRadius` instead of truncating there.
     * - `minWeightSum`: Optional. Emit no pairing if the accumulated weight
     *   falls below this.
     * - `maxNeighbors`: Optional (Default=0, meaning every point in the
     *   radius). **Leave it at 0.** A nonzero value makes the underlying
     *   `nn_radius_search()` rank its candidates and return only the best few,
     *   which is a top-k rule and reintroduces exactly the discreteness this
     *   matcher exists to remove. Some map classes cap that list at a handful
     *   of points regardless of the value asked for.
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    double   threshold           = 0.50;
    double   thresholdAngularDeg = 0;
    double   temperature         = 0;
    double   searchRadius        = 0;
    double   minWeightSum        = 0;
    uint32_t maxNeighbors        = 0;
    bool     smoothCutoff        = true;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
