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
#include <mrpt/math/CMatrixFixed.h>
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
 *  threads. If `outPoints` is given, it receives the point of each normal, in
 *  the same order (see the `points` argument of estimate_up_from_normals()).
 */
[[nodiscard]] std::vector<mrpt::math::TVector3D> estimate_planar_normals(
    const mrpt::maps::CPointsMap& pc, const PlanarNormalsParams& params = {},
    std::vector<mrpt::math::TPoint3Df>* outPoints = nullptr);

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

    /// Weight of the floor/ceiling term against the wall term (each one is
    /// first normalized by its population). 0 uses the walls only: they must
    /// then have at least two non-parallel directions. Floors and ceilings
    /// are not vertical references where a floor slopes (ramps), walls are.
    double flats_weight = 1.0;

    /// Side [m] of the cubic cells that group the normals for `up_cov`, when
    /// their points are given: errors shared by nearby normals (a leaning
    /// wall, a noisy patch, overlapping neighborhoods) then count once per
    /// cell. 0, or no points, treats every normal as independent.
    double cluster_size = 2.0;

    /// Fewest groups (cells) for the grouped `up_cov`. The scores of all
    /// groups sum to zero at the optimum, so a handful of groups cannot
    /// measure their spread (one group gives a null covariance, two a rank-1
    /// one): below this, or with a rank-deficient score covariance, `up_cov`
    /// falls back to the per-normal covariance and `cov_reliable` is false.
    std::size_t min_cov_groups = 8;
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

    /** Covariance [rad^2] of the horizontal components (x, y) of `up`: for a
     *  small tilt, of the lean of the normals' frame towards +X and +Y. It is
     *  the sandwich (robust) covariance of the estimator, built from the
     *  spread of the residuals actually observed (n.up for walls, n x up for
     *  floors and ceilings) and the curvature of the cost, so it needs no
     *  noise model. Its one assumption is independence between the groups of
     *  normals (cells of EstimateUpParams::cluster_size, or single normals):
     *  normals of one plane share its error, so per-normal independence is
     *  optimistic on real clouds. The grouped covariance is never smaller
     *  than the per-normal one. It does not account for references that are
     *  not plumb or level (a sloped floor, a leaning facade).
     */
    mrpt::math::CMatrixDouble22 up_cov;

    /// Number of groups found: cells of EstimateUpParams::cluster_size if
    /// points were given, otherwise single normals.
    std::size_t cov_groups = 0;

    /// True if `up_cov` comes from those groups: at least
    /// EstimateUpParams::min_cov_groups of them, with a full-rank score
    /// covariance. False means it fell back to the per-normal covariance,
    /// a lower bound of the real uncertainty: do not use it as a weight.
    bool cov_reliable = false;

    /// Angle between `up` and +Z [deg]
    [[nodiscard]] double tilt_deg() const;

    /// Standard deviations of up.x and up.y [deg], from `up_cov`
    [[nodiscard]] std::array<double, 2> up_sigmas_deg() const;
};

/** Estimates the "up" direction that makes walls as vertical, and floors and
 *  ceilings as horizontal, as possible.
 *
 *  Starting from u=+Z, normals are classified as walls (|n.u| < wall_nz) or
 *  flats (|n.u| > flat_nz), and `u` is updated to the eigenvector of the
 *  smallest eigenvalue of:
 *
 *      M = (1/|W|) sum_W n n^T + w_F (1/|F|) sum_F (I - n n^T)
 *
 *  Walls contribute (n.u)^2 and flats |n x u|^2, each term normalized so
 *  neither population dominates; w_F is EstimateUpParams::flats_weight.
 *
 *  `points`, if given, holds the point of each normal (same size and order,
 *  see estimate_planar_normals()) and is only used to group the normals for
 *  EstimateUpResult::up_cov.
 *
 *  The result is deterministic: normals are accumulated sequentially.
 *
 *  \exception std::exception If there are not enough normals, or "up" is not
 *             observable from them.
 */
[[nodiscard]] EstimateUpResult estimate_up_from_normals(
    const std::vector<mrpt::math::TVector3D>& normals, const EstimateUpParams& params = {},
    const std::vector<mrpt::math::TPoint3Df>* points = nullptr);

/** Returns the minimal rotation R (as a pose with zero translation) such that
 *  R * up = +Z. Its rotation axis is horizontal, so the map heading is kept.
 */
[[nodiscard]] mrpt::poses::CPose3D rotation_aligning_up_to_z(const mrpt::math::TVector3D& up);

/// An "up" estimate at a time, e.g. from the keyframes of one time window
struct UpSample
{
    double                t = 0;  //!< Time [s]
    mrpt::math::TVector3D up{0, 0, 1};  //!< Unit up vector, in the map frame
};

/** Up direction at time `t`: linear interpolation of the horizontal
 *  components of `samples` (sorted by time), held constant before the first
 *  and after the last one, and normalized. +Z if `samples` is empty.
 */
[[nodiscard]] mrpt::math::TVector3D interpolate_up(const std::vector<UpSample>& samples, double t);

/** Levels a trajectory whose vertical drifts slowly: each pose is rotated by
 *  R_k = rotation_aligning_up_to_z(up(t_k)), and the positions are integrated
 *  again from the first one with each step rotated by the correction at its
 *  mid time, so the trajectory stays continuous:
 *
 *      p'_0 = p_0,  p'_k = p'_{k-1} + R(t_mid) (p_k - p_{k-1}),  R'_k = R(t_k) R_k
 *
 *  `up(t)` is interpolate_up() of `samples`. `times` and `poses` must have
 *  the same size, with times sorted.
 */
[[nodiscard]] std::vector<mrpt::poses::CPose3D> level_poses_from_up(
    const std::vector<double>& times, const std::vector<mrpt::poses::CPose3D>& poses,
    const std::vector<UpSample>& samples);

/** @} */

}  // namespace mp2p_icp_filters

/** Feature macro: EstimateUpParams::flats_weight, EstimateUpResult::up_cov,
 *  EstimateUpResult::cov_reliable and level_poses_from_up() are available */
#define MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE 1
