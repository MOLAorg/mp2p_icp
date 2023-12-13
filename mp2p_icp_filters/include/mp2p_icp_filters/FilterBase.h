/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBase.h
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdint>
#include <stdexcept>

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
    DEFINE_VIRTUAL_MRPT_OBJECT(FilterBase)

   public:
    FilterBase();
    virtual ~FilterBase();

    /** \name API for all filtering/segmentation algorithms
     *  @{ */

    /** Loads, from a YAML configuration block, all the common, and
     * implementation-specific parameters. */
    virtual void initialize(const mrpt::containers::yaml& cfg_block) = 0;

    /** See docs above for FilterBase.
     */
    virtual void filter(mp2p_icp::metric_map_t& inOut) const = 0;

    /** @} */
   protected:
    static mrpt::maps::CPointsMap* GetOrCreatePointLayer(
        mp2p_icp::metric_map_t& m, const std::string& layerName,
        bool               allowEmptyName = true,
        const std::string& classForLayerCreation =
            "mrpt::maps::CSimplePointsMap");
};

/** A sequence of filters */
using FilterPipeline = std::vector<FilterBase::Ptr>;

/** Applies a pipeline of filters to a given metric_map_t  */
void apply_filter_pipeline(
    const FilterPipeline& filters, mp2p_icp::metric_map_t& inOut);

/** Creates a pipeline of filters from a YAML configuration block (a sequence).
 *  Refer to YAML file examples. Returns an empty pipeline for an empty or null
 * yaml node.
 * Returned filters are already initialize()'d.
 */
FilterPipeline filter_pipeline_from_yaml(
    const mrpt::containers::yaml&       c,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

/** \overload Taking a YAML filename as input.
 *  The file must contain with a top entry named `filters` with the sequence of
 *  filter descriptors.
 *  Returns an empty pipeline for an empty or null yaml node.
 *  Refer to YAML file examples.
 * Returned filters are already initialize()'d.
 */
FilterPipeline filter_pipeline_from_yaml_file(
    const std::string&                  filename,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

/** @} */

}  // namespace mp2p_icp_filters
