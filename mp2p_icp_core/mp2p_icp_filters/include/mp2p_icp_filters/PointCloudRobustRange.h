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
 * @file   PointCloudRobustRange.h
 * @brief  Outlier-robust estimate of a point cloud's maximum range.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 30, 2026
 */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>

namespace mp2p_icp_filters
{
/** Outlier-robust estimate of a point cloud's maximum range (distance from
 * a `center` point, default the origin).
 *
 * Plain `boundingBox().max.norm()` is dominated by even a single spurious
 * far return (e.g. specular-reflection artifacts observed on Livox sensors,
 * hundreds of meters away in an otherwise <20 m scene): consumers that use
 * such an estimate to size a downstream voxel grid or range filter can end
 * up starving the real, near-field points. This function instead returns
 * the given `percentile` of the per-point range distribution, so a small
 * minority of extreme points cannot dominate it.
 *
 * Implemented with `std::nth_element` (average O(n)), not a full sort.
 *
 * \param pc The point cloud. May be empty, in which case 0 is returned.
 * \param percentile In (0,1]. Ranges are sorted and the value at this
 *        fraction is returned; e.g. 0.95 (the default) discards the top 5%
 *        of ranges as potential outliers. 1.0 reproduces the plain maximum.
 * \param center Ranges are measured from this point.
 *
 * \ingroup mp2p_icp_filters_grp
 */
double robust_max_range(
    const mrpt::maps::CPointsMap& pc, double percentile = 0.95,
    const mrpt::math::TPoint3Df& center = {0, 0, 0});

}  // namespace mp2p_icp_filters
