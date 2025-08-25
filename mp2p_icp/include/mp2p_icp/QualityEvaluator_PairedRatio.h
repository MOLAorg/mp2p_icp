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
 * @file   QualityEvaluator_PairedRatio.h
 * @brief  Matching quality evaluator: simple ratio [0,1] of paired entities.
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */
#pragma once

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/QualityEvaluator.h>

namespace mp2p_icp
{
/** Matching quality evaluator: simple ratio [0,1] of paired entities,
 *  using either an independent Matcher_Points_DistanceThreshold object,
 *  or (faster) directly from the ratio of found pairings in the last ICP step
 *  if `reuse_icp_pairings` is `true`, the default.
 *
 * \ingroup mp2p_icp_grp
 */
class QualityEvaluator_PairedRatio : public QualityEvaluator
{
    DEFINE_MRPT_OBJECT(QualityEvaluator_PairedRatio, mp2p_icp)

   public:
    /** See base class. Parameters:
     *
     * \code
     * reuse_icp_pairings: true # Default=true (no more params then required)
     * #thresholdDistance: 0.10
     * #thresholdAngularDeg: 0
     * \endcode
     */
    void initialize(const mrpt::containers::yaml& params) override;

    Result evaluate(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const Pairings& pairingsFromICP) const override;

    void attachToParameterSource(ParameterSource& source) override
    {
        source.attach(*this);
        source.attach(matcher_);
    }

   private:
    Matcher_Points_DistanceThreshold matcher_;
    bool                             reuse_icp_pairings = true;

    double absolute_minimum_pairing_ratio = 0.20;
};

}  // namespace mp2p_icp
