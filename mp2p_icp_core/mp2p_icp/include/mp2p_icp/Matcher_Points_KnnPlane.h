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
 * @file   Matcher_Points_KnnPlane.h
 * @brief  Pointcloud matcher: k nearest map points, a least-squares plane, and
 *         fixed acceptance gates
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: k nearest map points, a least-squares plane through
 *  them, and a chain of fixed acceptance gates.
 *
 * This is the correspondence protocol used by Fast-LIO2, expressed as an
 * `mp2p_icp` matcher so that it can be run inside an otherwise unchanged
 * pipeline. Per local point, per iteration:
 *
 * 1. Query the `knn` nearest map points; reject if fewer are found.
 * 2. Reject if the farthest of them is beyond `maxNeighborDistance`.
 * 3. Fit a least-squares plane through them, and reject the fit if **any** of
 *    the points lies further than `planeFitMaxDeviation` from it.
 * 4. Accept the pairing only if
 *    `1 - residualGateScale * |d| / sqrt(range) > residualGateAccept`, where
 *    `d` is the point-to-plane distance and `range` the local point's own norm
 *    in the sensor frame. With `rangeNormalized: false` the division is
 *    dropped.
 *
 * It emits `paired_pt2pl`, so `Solver_GaussNewton` consumes it unchanged.
 *
 * What makes this matcher interesting as an experiment is that every gate is
 * either a constant or a function of a static property of the measurement:
 * none of them reads back the registration's own quality, unlike the
 * `adaptive_threshold` controller that drives the shipped matchers. It is also
 * the least smooth correspondence rule in this library -- a fixed count, three
 * hard gates -- which is why it is worth measuring against the blending
 * matchers rather than assuming which way the result must go.
 *
 * \note The plane is fitted by solving `A x = -1` with a column-pivoting
 *       Householder QR, in single precision, which is what the reference
 *       implementation does. That is deliberately **not** an eigen/PCA fit:
 *       the two are not numerically the same, and the point of this matcher is
 *       fidelity to the protocol.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_KnnPlane : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Points_KnnPlane, mp2p_icp)

   public:
    Matcher_Points_KnnPlane();

    /** Parameters, defaulting throughout to the reference implementation's
     * values:
     * - `knn`: Optional (Default=5). Number of nearest map points to fit the
     *   plane through.
     * - `maxNeighborDistance`: Optional (Default=2.236). Reject the pairing if
     *   the farthest of the `knn` neighbors is beyond this [meters]. The
     *   default is `sqrt(5)`, matching the reference's squared-distance
     *   literal.
     * - `planeFitMaxDeviation`: Optional (Default=0.1). Reject the plane fit if
     *   any supporting point lies further than this from it [meters].
     * - `residualGateScale`: Optional (Default=0.9).
     * - `residualGateAccept`: Optional (Default=0.9).
     * - `rangeNormalized`: Optional (Default=true). Divide the residual gate by
     *   the square root of the local point's range.
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    uint32_t knn                  = 5;
    double   maxNeighborDistance  = 2.236;
    double   planeFitMaxDeviation = 0.1;
    double   residualGateScale    = 0.9;
    double   residualGateAccept   = 0.9;
    bool     rangeNormalized      = true;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
