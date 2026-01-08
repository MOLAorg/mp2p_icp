/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterBase.h
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/version.h>

namespace mrpt::system
{
class CTimeLogger;
}

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/** Pure virtual base class of filters, modifying an input/output metric_map_t.
 *
 * Filters can be used to remove noisy points from a point cloud, decimate it,
 * or to detect planes, lines, or split points into different layers.
 *
 * \sa Generator
 */
class FilterBase : public mrpt::rtti::CObject,  // RTTI support
                   public mrpt::system::COutputLogger,  // Logging support
                   public mp2p_icp::Parameterizable  // Dynamic parameters
{
    DEFINE_VIRTUAL_MRPT_OBJECT(FilterBase, mp2p_icp_filters)

   public:
    FilterBase();
    virtual ~FilterBase();

    // Delete copy and move constructors and assignment operators
    FilterBase(const FilterBase&)            = default;
    FilterBase& operator=(const FilterBase&) = default;
    FilterBase(FilterBase&&)                 = default;
    FilterBase& operator=(FilterBase&&)      = default;

    /** \name API for all filtering/segmentation algorithms
     *  @{ */

    /** Loads, from a YAML configuration block, all the common, and
     * implementation-specific parameters. */
    void initialize(const mrpt::containers::yaml& cfg_block);

   protected:
    virtual void initialize_filter(const mrpt::containers::yaml& cfg_block) = 0;

   public:
    /** See docs above for FilterBase.
     */
    virtual void filter(mp2p_icp::metric_map_t& inOut) const = 0;

    /** @} */

    // Loads common parameters for all filters. Must be called from initialize.
    void initializeFilterBase(const mrpt::containers::yaml& cfg_block);

    /** If not empty, it will be used instead of class name in Logger and Profiler.
     *  This is loaded from the `name` key in the YAML configuration block.
     */
    std::string name;
};

/** A sequence of filters */
using FilterPipeline = std::vector<FilterBase::Ptr>;

/** Applies a pipeline of filters to a given metric_map_t  */
void apply_filter_pipeline(
    const FilterPipeline& filters, mp2p_icp::metric_map_t& inOut,
    const mrpt::optional_ref<mrpt::system::CTimeLogger>& profiler = std::nullopt);

/** Creates a pipeline of filters from a YAML configuration block (a sequence).
 *  Refer to YAML file examples. Returns an empty pipeline for an empty or null
 * yaml node.
 * Returned filters are already initialize()'d.
 */
[[nodiscard]] FilterPipeline filter_pipeline_from_yaml(
    const mrpt::containers::yaml&       c,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

/** \overload Taking a YAML filename as input.
 *  The file must contain with a top entry named `filters` with the sequence of
 *  filter descriptors.
 *  Returns an empty pipeline for an empty or null yaml node.
 *  Refer to YAML file examples.
 * Returned filters are already initialize()'d.
 */
[[nodiscard]] FilterPipeline filter_pipeline_from_yaml_file(
    const std::string&                  filename,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

// For convenience, define shortcut names for common point cloud field names:
#if MRPT_VERSION >= 0x20f03  // 2.15.3
constexpr auto POINT_FIELD_INTENSITY = mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY;
constexpr auto POINT_FIELD_RING_ID   = mrpt::maps::CPointsMap::POINT_FIELD_RING_ID;
constexpr auto POINT_FIELD_TIMESTAMP = mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP;
#else
// Define here locally, until MRPT 2.15.3 is the minimum required version:
constexpr static std::string_view POINT_FIELD_INTENSITY = "intensity";
constexpr static std::string_view POINT_FIELD_RING_ID   = "ring";
constexpr static std::string_view POINT_FIELD_TIMESTAMP = "t";
#endif

/** @} */

}  // namespace mp2p_icp_filters
