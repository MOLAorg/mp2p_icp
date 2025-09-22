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
    DEFINE_MRPT_OBJECT(Matcher_Cov2Cov, mp2p_icp)

   public:
    Matcher_Cov2Cov() = default;

    /** Pairs of Local->Global layers to match.
     * \note Note: this field can be loaded from a configuration file via initialize().
     *
     * \note Map is: { {"globalLayer", "localLayer"} [,...] }
     *
     */
    std::vector<std::pair<std::string, std::string>> layer_matches;

    /** The additional "margin" in all axes (x,y,z) that bounding box is
     * enlarged for checking the feasibility of pairings to exist. */
    float bounding_box_intersection_check_epsilon = 0.20f;

    /** Inliers distance threshold [meters] */
    float threshold = 0.40f;

    /** Common parameters to all derived classes:
     *
     * - `threshold`: Inliers distance threshold [meters][mandatory]
     *
     * - `layerMatches`: Optional map of layer names to match.
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
        Pairings& out) const override;
};

}  // namespace mp2p_icp
