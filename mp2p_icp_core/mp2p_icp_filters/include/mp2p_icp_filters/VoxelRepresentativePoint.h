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
 * @file   VoxelRepresentativePoint.h
 * @brief  DecimateMethod enum and the shared per-voxel point-picking logic,
 *         used by both FilterDecimateVoxels and FilterDecimateAdaptive.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#pragma once

#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace mrpt::random
{
class CRandomGenerator;
}

namespace mp2p_icp_filters
{
/** Enum to select what method to use to pick the downsampled point for each
 *  voxel, shared by FilterDecimateVoxels and FilterDecimateAdaptive.
 *
 * \ingroup mp2p_icp_filters_grp
 */
enum class DecimateMethod : uint8_t
{
    /** Pick the first point that was put int the voxel */
    FirstPoint = 0,
    /** Closest to the average of all voxel points */
    ClosestToAverage,
    /** Average of all voxel points */
    VoxelAverage,
    /** Pick one of the voxel points at random */
    RandomPoint
};

/** The outcome of picking one representative point for an occupied voxel,
 *  for any DecimateMethod except FirstPoint (callers normally special-case
 *  FirstPoint themselves, since it does not need to touch point coordinates
 *  at all). Exactly one of the two optionals is set:
 *  - `point`: a synthesized coordinate (DecimateMethod::VoxelAverage).
 *  - `pointIndex`: the index, into the source cloud, of a real point
 *    (DecimateMethod::ClosestToAverage, DecimateMethod::RandomPoint).
 */
struct VoxelRepresentativePoint
{
    std::optional<mrpt::math::TPoint3Df> point;
    std::optional<std::size_t>           pointIndex;
};

/** Computes the representative point of an occupied voxel for `method`,
 *  which must not be DecimateMethod::FirstPoint (that case is cheaper to
 *  special-case at the call site: it never needs to look at coordinates).
 *  `rng` is only touched for DecimateMethod::RandomPoint.
 */
VoxelRepresentativePoint pickVoxelRepresentativePoint(
    DecimateMethod method, const PointCloudToVoxelGrid::voxel_t& voxel,
    const mrpt::aligned_std_vector<float>& xs, const mrpt::aligned_std_vector<float>& ys,
    const mrpt::aligned_std_vector<float>& zs, mrpt::random::CRandomGenerator& rng);

}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::DecimateMethod)
MRPT_FILL_ENUM(DecimateMethod::FirstPoint);
MRPT_FILL_ENUM(DecimateMethod::ClosestToAverage);
MRPT_FILL_ENUM(DecimateMethod::VoxelAverage);
MRPT_FILL_ENUM(DecimateMethod::RandomPoint);
MRPT_ENUM_TYPE_END()
