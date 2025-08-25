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
 * @file   Matcher_Point2Plane.h
 * @brief  Pointcloud matcher: point to plane-fit of nearby points
 * @author Jose Luis Blanco Claraco
 * @date   July 21, 2020
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: point to plane-fit of nearby points
 *
 * Finds point-to-plane pairings between `local` point layers and points fitting
 * a plane in layers of the `global` input metric map.
 *
 * By default, each `local` point layer is matched against the layer with the
 * same name in the `global` map, unless specified otherwise in the base class
 * member `weight_pt2pt_layers`. Refer to example configuration YAML files for
 * example configurations.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Point2Plane : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Point2Plane, mp2p_icp)

   public:
    Matcher_Point2Plane();

    /** Parameters:
     * - `distanceThreshold`: Max. inliers pt-plane distance [meters][mandatory]
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    double distanceThreshold = 0.50;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
