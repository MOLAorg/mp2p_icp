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
 * @file   MetricMapMergeCapable.h
 * @brief  Virtual interface for map layers capable of being merged with others
 * @author Jose Luis Blanco Claraco
 * @date   Sep 24, 2025
 */
#pragma once

#include <mrpt/poses/CPose3D.h>

#include <optional>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/**
 * Virtual interface for map layers capable of being merged with others.
 * This merge behavior is already implemented upstream in mrpt::maps::CPointsMap,
 * but mp2p_icp does not know how to "merge" other metric map classes, hence this interface.
 *
 * This interface is used from mp2p_icp::metric_map_t::merge_with()
 *
 */
class MetricMapMergeCapable
{
   public:
    MetricMapMergeCapable()                                        = default;
    MetricMapMergeCapable(const MetricMapMergeCapable&)            = default;
    MetricMapMergeCapable& operator=(const MetricMapMergeCapable&) = default;
    MetricMapMergeCapable(MetricMapMergeCapable&&)                 = default;
    MetricMapMergeCapable& operator=(MetricMapMergeCapable&&)      = default;
    virtual ~MetricMapMergeCapable();

    /** Merge  */
    virtual void merge_with(
        const MetricMapMergeCapable&               source,
        const std::optional<mrpt::poses::CPose3D>& otherRelativePose = std::nullopt) = 0;

    /** Change the map such as each entity \f$ p_i \f$ becomes \f$ p'_i = b \oplus p_i \f$
     * (pose compounding operator).
     */
    virtual void transform_map_left_multiply([[maybe_unused]] const mrpt::poses::CPose3D& b)
    {
        throw std::runtime_error("changeCoordinatesReference() not implemented for this map type");
    }
};
/** @} */

}  // namespace mp2p_icp
