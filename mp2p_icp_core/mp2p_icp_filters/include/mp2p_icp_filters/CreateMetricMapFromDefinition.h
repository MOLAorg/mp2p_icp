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
 * @file   CreateMetricMapFromDefinition.h
 * @brief  Auxiliary function CreateMetricMapFromDefinition
 * @author Jose Luis Blanco Claraco
 * @date   Aug 25, 2026
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/system/COutputLogger.h>

#include <string>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/** Instantiates an empty mrpt::maps::CMetricMap object of the class described by a
 *  `metric_map_definition` YAML block, using the same schema accepted by Generator's
 *  `metric_map_definition` parameter (a `class` key, plus optional `plugin`, `creationOpts`,
 *  `insertOpts`, `likelihoodOpts`, `renderOpts`, etc. sub-blocks), or from an external `.ini`
 *  file in the format expected by mrpt::maps::TSetOfMetricMapInitializers.
 *
 *  Internally builds a one-element mrpt::maps::CMultiMetricMap from the given initializer and
 *  returns its only submap. This is the very same mechanism Generator uses to create a
 *  custom-class layer the first time it processes a matching observation; it is exposed here as a
 *  standalone function so it can also be used without any observation at hand, e.g. to pre-create
 *  a layer for an `mm-filter` pipeline (see Generator::createTargetLayerIfNeeded()).
 *
 *  At least one of `metricMapDefinition` or `metricMapDefinitionIniFile` must be non-empty. If
 *  both are given, `metricMapDefinitionIniFile` takes precedence and `metricMapDefinition` is
 *  ignored.
 *
 * \param[in] metricMapDefinition YAML map with, at least, a `class` key. Ignored if
 *            `metricMapDefinitionIniFile` is not empty.
 * \param[in] metricMapDefinitionIniFile Alternative to the above: a path to an external `.ini`
 *            file, taking precedence over `metricMapDefinition` if both are given.
 * \param[in] logger Optional logger, forwarded to mp2p_icp::load_plugin() if a `plugin` key is
 *            given.
 *
 * \ingroup mp2p_icp_filters_grp
 */
[[nodiscard]] mrpt::maps::CMetricMap::Ptr CreateMetricMapFromDefinition(
    const mrpt::containers::yaml&      metricMapDefinition,
    const std::string&                 metricMapDefinitionIniFile = std::string(),
    const mrpt::system::COutputLogger* logger                     = nullptr);

/** @} */

}  // namespace mp2p_icp_filters
