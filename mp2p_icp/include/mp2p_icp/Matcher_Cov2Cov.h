/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Matcher_Cov2Cov.h
 * @brief  Point-to-point with associated local covariance matcher
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2025
 */
#pragma once

#include <mp2p_icp/Matcher.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdlib>

namespace mp2p_icp
{
/** Point-to-point with associated local covariance matcher.
 *
 * Both maps (local and global layers) must implement the mp2p_icp::NearestPointWithCovCapable
 * interface.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Cov2Cov : public Matcher
{
   public:
    Matcher_Cov2Cov() = default;

    /** Weights for each potential Local->Global layer matching.
     * If empty, the output Pairings::point_weights
     * will left empty (=all points have equal weight).
     * \note Note: this field can be loaded from a configuration file via
     * initializeLayerWeights().
     *
     * \note Map is: w["globalLayer"]["localLayer"]=weight;
     *
     */
    std::map<std::string, std::map<std::string, double>> weight_pt2pt_layers;

    /** The additional "margin" in all axes (x,y,z) that bounding box is
     * enlarged for checking the feasibility of pairings to exist. */
    float bounding_box_intersection_check_epsilon_ = 0.20f;

    /** Common parameters to all derived classes:
     *
     * - `pointLayerMatches`: Optional map of layer names to relative weights.
     *  Refer to example YAML files.
     *
     * - `bounding_box_intersection_check_epsilon`: Optional (Default=0.20). The
     * additional "margin" in all axes (x,y,z) that bounding box is enlarged for
     * checking the feasibility of pairings to exist.
     */
    void initialize(const mrpt::containers::yaml& params) override;

   protected:
    bool impl_match(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc, MatchState& ms,
        Pairings& out) const override final;
};

}  // namespace mp2p_icp
