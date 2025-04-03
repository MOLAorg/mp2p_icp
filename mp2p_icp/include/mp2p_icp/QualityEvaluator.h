/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
#include <mrpt/version.h>

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
#if MRPT_VERSION < 0x020e00
    DEFINE_VIRTUAL_MRPT_OBJECT(QualityEvaluator)
#else
    DEFINE_VIRTUAL_MRPT_OBJECT(QualityEvaluator, mp2p_icp)
#endif

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
