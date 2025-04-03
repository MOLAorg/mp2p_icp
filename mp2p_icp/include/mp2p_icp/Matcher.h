/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/pointcloud_bitfield.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/version.h>

namespace mp2p_icp
{
/** Defines the static part of a match operation.
 *
 * \ingroup mp2p_icp_grp
 */
struct MatchContext
{
    MatchContext() = default;

    /// The ICP iteration number we are in:
    uint32_t icpIteration = 0;
};

struct MatchState
{
    MatchState(const metric_map_t& pcGlobal, const metric_map_t& pcLocal)
        : pcGlobal_(pcGlobal), pcLocal_(pcLocal)
    {
        initialize();
    }

    /// The pairings already assigned by former matches in the pipeline
    /// indexed by the pcLocal entities (true=already have a pairing).
    pointcloud_bitfield_t localPairedBitField;

    /// Like localPairedBitField for the global map
    pointcloud_bitfield_t globalPairedBitField;

    /** Initialize all bit fields to their correct length and default value
     * (false) */
    void initialize()
    {
        localPairedBitField.initialize_from(pcLocal_);
        globalPairedBitField.initialize_from(pcGlobal_);
    }

   private:
    const metric_map_t& pcGlobal_;
    const metric_map_t& pcLocal_;
};

/** Pointcloud matching generic base class.
 * Each "matcher" implementation takes a global ("reference") `metric_map_t` and
 * another local ("mobile") `metric_map_t` which is assumed to be placed in a
 * hypothetical SE(3) pose in the global frame, and generates pairings between
 * the geometric entities (points, planes, etc.) of both groups.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher : public mrpt::system::COutputLogger,
                public mrpt::rtti::CObject,
                public mp2p_icp::Parameterizable
{
#if MRPT_VERSION < 0x020e00
    DEFINE_VIRTUAL_MRPT_OBJECT(Matcher)
#else
    DEFINE_VIRTUAL_MRPT_OBJECT(Matcher, mp2p_icp)
#endif

   public:
    /** Check each derived class to see required and optional parameters. */
    virtual void initialize(const mrpt::containers::yaml& params);

    /** Finds correspondences between the two point clouds.
     * "out" is not cleared, but new pairings added to it.
     * \return false if the matcher is disabled and was not actually run.
     */
    virtual bool match(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc, MatchState& ms,
        Pairings& out) const;

    uint32_t runFromIteration = 0;
    uint32_t runUpToIteration = 0;  //!< 0: no limit
    bool     enabled          = true;

   protected:
    /// \return true if the mather is actually invoked, false if disabled.
    virtual bool impl_match(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc, MatchState& ms,
        Pairings& out) const = 0;
};

using matcher_list_t = std::vector<mp2p_icp::Matcher::Ptr>;

/** Runs a sequence of matcher between two metric_map_t objects.
 *
 * This is normally invoked by mp2p_icp::ICP, but users can use it as a
 * standalone module as needed.
 *
 * \ingroup mp2p_icp_grp
 */
Pairings run_matchers(
    const matcher_list_t& matchers, const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& local_wrt_global, const MatchContext& mc,
    const mrpt::optional_ref<MatchState>& userProvidedMS = std::nullopt);

}  // namespace mp2p_icp
