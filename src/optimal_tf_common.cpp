/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   OptimalTF_common.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_common.h>

using namespace mp2p_icp;

std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D>
    mp2p_icp::eval_centroids_robust(
        const PairingsCommon& in, const OutlierIndices& outliers)
{
    using mrpt::math::TPoint3D;

    const auto nPoints = in.paired_points.size();

    // We need more points than outliers (!)
    ASSERT_ABOVE_(nPoints, outliers.point2point.size());

    // Normalized weights for centroids.
    // Discount outliers.
    const double wcPoints = 1.0 / (nPoints - outliers.point2point.size());

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);
    {
        std::size_t cnt             = 0;
        auto        it_next_outlier = outliers.point2point.begin();
        for (std::size_t i = 0; i < in.paired_points.size(); i++)
        {
            // Skip outlier?
            if (it_next_outlier != outliers.point2point.end() &&
                i == *it_next_outlier)
            {
                ++it_next_outlier;
                continue;
            }
            const auto& pair = in.paired_points[i];

            ct_this += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
            cnt++;
        }
        // Sanity check:
        ASSERT_EQUAL_(cnt, nPoints - outliers.point2point.size());

        ct_other *= wcPoints;
        ct_this *= wcPoints;
    }

    return {ct_other, ct_this};
}
