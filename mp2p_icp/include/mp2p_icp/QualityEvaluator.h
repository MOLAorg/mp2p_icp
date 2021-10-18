/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
                         public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(QualityEvaluator)

   public:
    /** Check each derived class to see required and optional parameters. */
    virtual void initialize(const mrpt::containers::yaml& params) = 0;

    /** Finds correspondences between the two point clouds. */
    virtual double evaluate(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose,
        const Pairings&             pairingsFromICP) const = 0;
};

}  // namespace mp2p_icp
