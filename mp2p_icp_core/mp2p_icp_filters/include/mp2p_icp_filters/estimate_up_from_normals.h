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
 * @file   estimate_up_from_normals.h
 * @brief  Estimates the "up" direction of a map from wall and floor normals.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 8, 2026
 */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <array>
#include <cstddef>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Parameters for estimate_planar_normals()
struct PlanarNormalsParams
{
    /// Neighborhood radius for each local plane fit [m]
    double search_radius = 0.4;

    /// Points with fewer neighbors (including itself) get no normal
    std::size_t min_neighbors = 8;

    /// A neighborhood is planar if eigVal[0] < max_planarity_ratio * eigVal[1]
    double max_planarity_ratio = 0.1;

    std::size_t parallelization_grain_size = 1024;
};

/** Estimates one unit normal per point whose neighborhood is planar, as the
 *  eigenvector of the smallest eigenvalue of the covariance of the neighbors
 *  within `search_radius`. The sign of the normals is arbitrary.
 *
 *  The output is in point index order and independent of the number of
 *  threads.
 */
[[nodiscard]] std::vector<mrpt::math::TVector3D> estimate_planar_normals(
    const mrpt::maps::CPointsMap& pc, const PlanarNormalsParams& params = {});

/// Parameters for estimate_up_from_normals()
struct EstimateUpParams
{
    /// Normals with |n.up| below this are walls
    double wall_nz = 0.25;

    /// Normals with |n.up| above this are floors or ceilings ("flats")
    double flat_nz = 0.95;

    /// Number of re-classification iterations
    std::size_t iterations = 5;

    /// A population (walls or flats) with fewer normals than this is ignored,
    /// so a handful of outliers can not weigh as much as the other population.
    std::size_t min_population = 100;

    /// Minimum gap between the two smallest eigenvalues of the cost matrix.
    /// Below it, "up" is not observable (e.g. one wall direction and no floor).
    double min_eigen_gap = 0.05;
};

/// Output of estimate_up_from_normals()
struct EstimateUpResult
{
    /// Estimated "up" unit vector, with up.z > 0
    mrpt::math::TVector3D up = {0, 0, 1};

    std::size_t wall_count = 0;  //!< Wall normals used in the last iteration
    std::size_t flat_count = 0;  //!< Floor/ceiling normals used in the last iteration

    /// Eigenvalues (ascending) of the full cost matrix
    std::array<double, 3> eigVals = {0, 0, 0};

    /// Eigenvalues (ascending) of the wall term alone. If the second one is
    /// close to zero, all walls share a single direction.
    std::array<double, 3> wallEigVals = {0, 0, 0};

    /// True if walls alone do not constrain both pitch and roll (one wall
    /// direction): the result then relies on the floors and ceilings.
    bool single_wall_direction = false;

    /// Angle between `up` and +Z [deg]
    [[nodiscard]] double tilt_deg() const;
};

/** Estimates the "up" direction that makes walls as vertical, and floors and
 *  ceilings as horizontal, as possible.
 *
 *  Starting from u=+Z, normals are classified as walls (|n.u| < wall_nz) or
 *  flats (|n.u| > flat_nz), and `u` is updated to the eigenvector of the
 *  smallest eigenvalue of:
 *
 *      M = (1/|W|) sum_W n n^T + (1/|F|) sum_F (I - n n^T)
 *
 *  Walls contribute (n.u)^2 and flats |n x u|^2, each term normalized so
 *  neither population dominates.
 *
 *  The result is deterministic: normals are accumulated sequentially.
 *
 *  \exception std::exception If there are not enough normals, or "up" is not
 *             observable from them.
 */
[[nodiscard]] EstimateUpResult estimate_up_from_normals(
    const std::vector<mrpt::math::TVector3D>& normals, const EstimateUpParams& params = {});

/** Returns the minimal rotation R (as a pose with zero translation) such that
 *  R * up = +Z. Its rotation axis is horizontal, so the map heading is kept.
 */
[[nodiscard]] mrpt::poses::CPose3D rotation_aligning_up_to_z(const mrpt::math::TVector3D& up);

/** @} */

}  // namespace mp2p_icp_filters
