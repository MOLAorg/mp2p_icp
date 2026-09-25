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
 * @file   DecimateMethod.h
 * @brief  Method to pick the representative point of a voxel
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#pragma once

#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mp2p_icp_filters
{
/** Enum to select what method to use to pick the downsampled point for each
 *  voxel, in FilterDecimateVoxels and FilterDecimateAdaptive.
 *
 * \ingroup mp2p_icp_filters_grp
 */
enum class DecimateMethod : uint8_t
{
    /** Pick the first point that was put int the voxel */
    FirstPoint = 0,
    /** Pick the k-th point of a voxel, with k rotating from one voxel to the
     *  next, so that whatever the sensor's scan order does to the choice is
     *  spread over the voxels instead of displacing all of them alike.
     *  k comes from the voxel's own integer coordinates, so the result does
     *  not depend on traversal order, on the number of threads, or on the
     *  underlying map type. */
    RotatingIndex,
    /** Closest to the average of all voxel points */
    ClosestToAverage,
    /** Average of all voxel points. Other per-point fields are taken from the
     * point closest to the average. */
    VoxelAverage,
    /** Pick one of the voxel points at random */
    RandomPoint
};

}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::DecimateMethod)
MRPT_FILL_ENUM(DecimateMethod::FirstPoint);
MRPT_FILL_ENUM(DecimateMethod::RotatingIndex);
MRPT_FILL_ENUM(DecimateMethod::ClosestToAverage);
MRPT_FILL_ENUM(DecimateMethod::VoxelAverage);
MRPT_FILL_ENUM(DecimateMethod::RandomPoint);
MRPT_ENUM_TYPE_END()
