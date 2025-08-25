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
 * @file   Matcher_Points_DistanceThreshold.h
 * @brief  Pointcloud matcher: fixed distance thresholds
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: fixed distance thresholds.
 *
 * Finds point-to-point pairings between the `local` and `global` input metric
 * maps.
 *
 * By default, each `local` point layer is matched against the layer with the
 * same name in the `global` map, unless especified otherwise in the base class
 * member `weight_pt2pt_layers`. Refer to example configuration YAML files for
 * example configurations.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_DistanceThreshold : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Points_DistanceThreshold, mp2p_icp)

   public:
    Matcher_Points_DistanceThreshold();

    Matcher_Points_DistanceThreshold(double distThreshold) : Matcher_Points_DistanceThreshold()
    {
        threshold = distThreshold;
    }

    /** Parameters:
     * - `threshold`: Inliers distance threshold [meters][mandatory]
     * - `pairingsPerPoint`: Number of pairings in "global" for each "local"
     * points. Default=1. If more than one, they will be picked in ascending
     * order of distance, up to `threshold`. [optional].
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

    double   threshold           = 0.50;  // m
    double   thresholdAngularDeg = 0.50;  // deg
    uint32_t pairingsPerPoint    = 1;

   private:
    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
