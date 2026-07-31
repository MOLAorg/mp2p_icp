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
 * @file   PointCloudRobustRange.cpp
 * @brief  Outlier-robust estimate of a point cloud's maximum range.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 30, 2026
 */

#include <mp2p_icp_filters/PointCloudRobustRange.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace mp2p_icp_filters;

double mp2p_icp_filters::robust_max_range(
    const mrpt::maps::CPointsMap& pc, double percentile, const mrpt::math::TPoint3Df& center)
{
    ASSERT_GT_(percentile, 0.0);
    ASSERT_LE_(percentile, 1.0);

    const size_t n = pc.size();
    if (n == 0)
    {
        return 0.0;
    }

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    std::vector<float> ranges(n);
    for (size_t i = 0; i < n; i++)
    {
        const float dx = xs[i] - center.x;
        const float dy = ys[i] - center.y;
        const float dz = zs[i] - center.z;
        ranges[i]      = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // percentile==1.0 must still map to the last (largest) element, not one
    // past the end.
    const size_t rank = std::min(n - 1, static_cast<size_t>(percentile * static_cast<double>(n)));

    std::nth_element(ranges.begin(), ranges.begin() + rank, ranges.end());
    return static_cast<double>(ranges[rank]);
}
