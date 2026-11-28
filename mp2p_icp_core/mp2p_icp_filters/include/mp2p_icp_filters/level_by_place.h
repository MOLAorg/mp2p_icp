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
 * @file   level_by_place.h
 * @brief  Levels a trajectory with a vertical correction that depends on place.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#pragma once

#include <mp2p_icp_filters/graph_partition.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Parameters for level_poses_by_place()
struct PlaceLevelingParams
{
    /// Weight of the edge between each pair of consecutive keyframes
    double sequential_weight = 1.0;
};

/// Output of level_poses_by_place()
struct PlaceLevelingResult
{
    std::vector<mrpt::poses::CPose3D> poses;  //!< Leveled poses

    /// Misfit of the leveled positions on each edge kind [m]: the part of the
    /// correction field that no position change can follow (it is not a
    /// gradient), plus the effect of the edge weights.
    double sequential_residual_rms = 0;
    double sequential_residual_max = 0;
    double link_residual_rms       = 0;
    double link_residual_max       = 0;
};

/** Levels poses with one "up" vector per pose, which may vary from place to
 *  place. Each attitude is rotated by its own correction
 *  C_k = rotation_aligning_up_to_z(ups[k]). Positions are not integrated in
 *  time order: they are the least-squares solution, on a graph of
 *  consecutive keyframes plus `links` (e.g. revisits of a place), of
 *
 *      p'_j - p'_i = C_ij (p_j - p_i),   with p'_0 = p_0,
 *
 *  where C_ij aligns the mean of ups[i] and ups[j]. Two visits of a place,
 *  linked and corrected alike, keep their relative position (rotated): the
 *  correction cannot separate them as an integration in time order does.
 *  Link weights are relative to PlaceLevelingParams::sequential_weight.
 *
 *  One sparse solve (SimplicialLDLT) for the three coordinates.
 */
[[nodiscard]] PlaceLevelingResult level_poses_by_place(
    const std::vector<mrpt::poses::CPose3D>& poses, const std::vector<mrpt::math::TVector3D>& ups,
    const std::vector<WeightedEdge>& links, const PlaceLevelingParams& params = {});

/** Levels a dense path with the correction of the keyframes, given before and
 *  after leveling (`kfBefore`, `kfAfter`) with their `kfUps`, all at the
 *  sorted times `kfTimes`. Each pose between two keyframes moves with them,
 *  in proportion to its time between the two:
 *
 *      p' = (1-f) (p'_a + C_a (p - p_a)) + f (p'_b + C_b (p - p_b))
 *
 *  and its attitude is rotated by the interpolated up. Poses outside the
 *  keyframe times move with the nearest keyframe. Since the keyframe
 *  correction depends on place, so does this one.
 */
[[nodiscard]] std::vector<mrpt::poses::CPose3D> level_path_by_place(
    const std::vector<double>& pathTimes, const std::vector<mrpt::poses::CPose3D>& pathPoses,
    const std::vector<double>& kfTimes, const std::vector<mrpt::poses::CPose3D>& kfBefore,
    const std::vector<mrpt::poses::CPose3D>&  kfAfter,
    const std::vector<mrpt::math::TVector3D>& kfUps);

/** @} */

}  // namespace mp2p_icp_filters
