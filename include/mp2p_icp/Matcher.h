/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher.h
 * @brief  Pointcloud matching generic base class
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/containers/Parameters.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mp2p_icp
{
/** Defines the context of a match operation.
 *
 * \ingroup mp2p_icp_grp
 */
struct MatchContext
{
    MatchContext() = default;

    uint32_t icpIteration = 0;
};

/** Pointcloud matching generic base class.
 * Each "matcher" implementation takes a global ("reference") `pointcloud_t` and
 * another local ("mobile") `pointcloud_t` which is assumed to be placed in a
 * hypothetical SE(3) pose in the global frame, and generates pairings between
 * the geometric entities (points, planes, etc.) of both groups.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher : public mrpt::system::COutputLogger, public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(Matcher);

   public:
    /** Check each derived class to see required and optional parameters. */
    virtual void initialize(const mrpt::containers::Parameters& params);

    /** Finds correspondences between the two point clouds.
     * "out" is not cleared, but new pairings added to it.
     */
    virtual void match(
        const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc,
        Pairings& out) const;

    uint32_t runFromIteration = 0;
    uint32_t runUpToIteration = 0;  //!< 0: no limit

   protected:
    virtual void impl_match(
        const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc,
        Pairings& out) const = 0;
};

}  // namespace mp2p_icp
