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
 * @file   VoxelRepresentativePoint.cpp
 * @brief  DecimateMethod enum and the shared per-voxel point-picking logic.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#include <mp2p_icp_filters/VoxelRepresentativePoint.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/random/RandomGenerators.h>

using namespace mp2p_icp_filters;

VoxelRepresentativePoint mp2p_icp_filters::pickVoxelRepresentativePoint(
    const DecimateMethod method, const PointCloudToVoxelGrid::voxel_t& voxel,
    const mrpt::aligned_std_vector<float>& xs, const mrpt::aligned_std_vector<float>& ys,
    const mrpt::aligned_std_vector<float>& zs, mrpt::random::CRandomGenerator& rng)
{
    VoxelRepresentativePoint out;

    switch (method)
    {
        case DecimateMethod::VoxelAverage:
        case DecimateMethod::ClosestToAverage:
        {
            mrpt::math::TPoint3Df mean(0, 0, 0);
            const float           inv_n = 1.0f / static_cast<float>(voxel.size());
            for (size_t i = 0; i < voxel.size(); i++)
            {
                const auto pt_idx = voxel[i];
                mean.x += xs[pt_idx];
                mean.y += ys[pt_idx];
                mean.z += zs[pt_idx];
            }
            mean *= inv_n;

            if (method == DecimateMethod::VoxelAverage)
            {
                out.point = mean;
                break;
            }

            std::optional<float>  minSqrErr;
            std::optional<size_t> bestIdx;
            for (size_t i = 0; i < voxel.size(); i++)
            {
                const auto  pt_idx = voxel[i];
                const float sqrErr = mrpt::square(xs[pt_idx] - mean.x) +
                                     mrpt::square(ys[pt_idx] - mean.y) +
                                     mrpt::square(zs[pt_idx] - mean.z);
                if (!minSqrErr.has_value() || sqrErr < *minSqrErr)
                {
                    minSqrErr = sqrErr;
                    bestIdx   = pt_idx;
                }
            }
            out.pointIndex = *bestIdx;
            break;
        }
        case DecimateMethod::RandomPoint:
        {
            const auto idxInVoxel = rng.drawUniform64bit() % voxel.size();
            out.pointIndex        = voxel[idxInVoxel];
            break;
        }
        case DecimateMethod::FirstPoint:
        default:
            THROW_EXCEPTION(
                "pickVoxelRepresentativePoint() does not handle DecimateMethod::FirstPoint; "
                "callers must special-case it.");
    }

    return out;
}
