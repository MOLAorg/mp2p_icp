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
 * @file   QualityEvaluator.h
 * @brief  Matching quality evaluator (virtual base class)
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mp2p_icp
{
/** Matching quality evaluator (virtual base class)
 *
 * \ingroup mp2p_icp_grp
 */
class QualityEvaluator : public mrpt::system::COutputLogger,
                         public mrpt::rtti::CObject,
                         public mp2p_icp::Parameterizable
{
    DEFINE_VIRTUAL_MRPT_OBJECT(QualityEvaluator, mp2p_icp)

   public:
    struct Result
    {
        /// The resulting quality measure, in the range [0,1]
        double quality = .0;

        /// If true, the match is bad and all other quality measurements should
        /// be discarded
        bool hard_discard = false;
    };

    /** Check each derived class to see required and optional parameters. */
    virtual void initialize(const mrpt::containers::yaml& params) = 0;

    /** Finds correspondences between the two point clouds. */
    virtual Result evaluate(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const Pairings& pairingsFromICP) const = 0;
};

}  // namespace mp2p_icp
