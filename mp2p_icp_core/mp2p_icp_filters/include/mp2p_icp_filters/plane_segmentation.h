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
 * @file   plane_segmentation.h
 * @brief  Segmentation of large planes in point clouds and maps
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#pragma once

#include <mp2p_icp/plane_patch.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Parameters for segment_planes()
struct PlaneSegmentationParams
{
    /// Sampling resolution [m]: the cloud is reduced to one representative
    /// (the mean) per voxel for normal estimation and region growing, and the
    /// area of each plane is measured on an in-plane grid of this size.
    /// Parallel surfaces closer than about this distance cannot be told apart.
    double voxel_size = 0.10;

    /// Neighbors (including the point itself) used to estimate the normal of
    /// each representative. They also define the region growing graph.
    uint32_t normal_knn = 16;

    /// Neighbors farther than this are ignored [m]. Larger values bridge the
    /// gaps between rings of sparse LiDAR scans.
    double normal_radius = 0.50;

    /// Region growing: maximum angle between a representative's normal and the
    /// normal of the region it joins [deg].
    double normal_agreement_deg = 10.0;

    /// Region growing: a representative joins a region if its distance to the
    /// current plane of the region is below three times the noise measured
    /// in the region, but never less than this [m] (nor more than
    /// `inlier_distance`). Keep it below half the distance between parallel
    /// surfaces that must stay apart.
    double distance_threshold = 0.03;

    /// Final refinement: an input point is an inlier of its plane if its
    /// distance to the refined plane is at most this [m]. Make it wider than
    /// any systematic error (e.g. a per-beam range offset) to be measured
    /// against the plane, or those points are excluded from it, but narrower
    /// than the gap to a parallel surface that is not itself a plane.
    double inlier_distance = 0.08;

    /// Planes whose RMS inlier distance is larger are rejected [m]. Also the
    /// flatness that a merged pair must keep.
    double max_thickness = 0.04;

    /// Planes with a covered area smaller than this are rejected [m2].
    double min_area = 2.0;

    /// Planes with fewer inliers than this are rejected.
    uint32_t min_points = 100;

    /// Merging: two segments merge only if their normals agree within this
    /// angle [deg], ...
    double merge_normal_deg = 3.0;

    /// ... the centroid of the smaller one lies within this distance of the
    /// plane of the larger one [m], ...
    double merge_offset = 0.05;

    /// ... and they are spatially adjacent: some representatives of both lie
    /// within (roughly) this distance [m].
    double merge_max_gap = 0.50;

    /// Normals are oriented to point towards this point (i.e. the viewpoint is
    /// on the positive side, `n.v > d`). For a map, the origin is usually the
    /// first sensor pose.
    mrpt::math::TPoint3D viewpoint = {0, 0, 0};

    /// When TBB is enabled, the grain size for parallel loops.
    std::size_t parallelization_grain_size = 4096;
};

/// One plane found by segment_planes()
struct SegmentedPlane
{
    /// Unit normal, pointing towards PlaneSegmentationParams::viewpoint
    mrpt::math::TVector3D normal = {0, 0, 1};

    /// Plane equation: `normal.p = offset` for every point `p` on the plane
    double offset = 0;

    /// Mean of the inliers, projected onto the plane
    mrpt::math::TPoint3D centroid = {0, 0, 0};

    /// Indices of the inliers in the input cloud, in ascending order
    std::vector<uint32_t> inliers;

    /// Covered area [m2]: occupied cells of an in-plane grid of `voxel_size`
    double area = 0;

    /// RMS of the inlier point-to-plane distances [m]
    double thickness_rms = 0;

    /// 95th percentile of the absolute inlier point-to-plane distances [m]
    double thickness_p95 = 0;

    /// In-plane principal directions (largest spread first). `axis_u`,
    /// `axis_v`, `normal` form a right-handed orthonormal basis.
    mrpt::math::TVector3D axis_u = {1, 0, 0};
    mrpt::math::TVector3D axis_v = {0, 1, 0};

    /// Oriented bounding box: inlier coordinates along `axis_u` and `axis_v`
    /// span `[u_min, u_max] x [v_min, v_max]`, relative to `centroid` [m].
    double u_min = 0;
    double u_max = 0;
    double v_min = 0;
    double v_max = 0;

    /** Variances of the minimal plane parameters `[alpha, beta, gamma]`: the
     *  plane through `centroid + gamma * normal` with normal
     *  `normal + alpha * axis_u + beta * axis_v` (to first order). They are
     *  uncorrelated in this basis: `max(thickness_rms, 1e-4 m)^2` divided by
     *  the sum of squared `u` (resp. `v`) coordinates, and by the number of
     *  inliers. The 0.1 mm floor keeps them positive for exact synthetic
     *  planes. This assumes independent residuals, which overlapping scans of
     *  a map are not: treat it as a lower bound.
     */
    std::array<double, 3> tangent_variances = {0, 0, 0};

    /// Covariance of `[nx, ny, nz, offset]`, the same uncertainty mapped to
    /// the 4-parameter form (rank 3).
    mrpt::math::CMatrixDouble44 covariance;

    std::size_t num_points() const { return inliers.size(); }

    /// The plane as an MRPT object
    [[nodiscard]] mrpt::math::TPlane plane() const;

    /// The plane, centroid, area and number of points as a plane_patch_t,
    /// e.g. for metric_map_t::planes
    [[nodiscard]] mp2p_icp::plane_patch_t as_plane_patch() const;
};

/// Output of segment_planes()
struct PlaneSegmentationResult
{
    /// The planes, by decreasing number of inliers
    std::vector<SegmentedPlane> planes;

    std::size_t num_input_points = 0;  //!< Finite input points
    std::size_t num_voxels       = 0;  //!< Representatives (occupied voxels)
    std::size_t num_normals      = 0;  //!< Representatives with a planar neighborhood
    std::size_t num_regions      = 0;  //!< Segments after region growing
    std::size_t num_merged       = 0;  //!< Segments after merging
};

/** Segments the large planar surfaces (ground, walls, facades, ceilings...)
 *  of a point cloud or map.
 *
 *  1. The cloud is reduced to one representative per voxel (`voxel_size`).
 *     Its normal comes from the voxel's own points if they are enough and
 *     planar (robust to a few points of a surface next to it), otherwise
 *     from the nearest neighbor representatives (at a wider scale if the map
 *     is noisy). Neighborhoods that are not planar (edges, clutter) or that
 *     are degenerate (a single scan ring) get no normal.
 *  2. Region growing, from the flattest representatives first: a neighbor
 *     joins a region if it lies close to the region's plane, which is
 *     refitted as the region grows, and its normal (if it has one) agrees
 *     with the region's. On a curved surface, a region stops where the surface
 *     leaves the plane, so curved surfaces are split into near-planar pieces.
 *  3. Conservative merging of segments of the same surface (split by
 *     occlusions or by the order of growth): only if the normals agree, the
 *     smaller one lies on the plane of the larger one, the segments are
 *     adjacent, and their union is still as flat as `max_thickness`. Two
 *     close parallel walls stay apart. Large segments then absorb the
 *     unassigned or small neighbor segments that lie on their plane.
 *  4. Multi-plane refinement on the original points: each point of a voxel
 *     of a segment goes to the closest plane (within `inlier_distance`)
 *     among the segments of its voxel and of the neighbor voxels, so a voxel
 *     shared by two surfaces is split between them; then each plane is
 *     refitted by robust total least squares (Cauchy kernel, MAD scale), and
 *     merged again if they turned out to be one. The planes are then
 *     measured and rejected by `min_points`, `min_area`, `max_thickness`,
 *     and if most of their points are in voxels whose own normal disagrees
 *     with them (a slab cut through vegetation).
 *
 *  No orientation is assumed: vertical, horizontal and slanted planes are
 *  treated alike. Non-finite points are ignored.
 *
 *  The result is deterministic, independent of the number of threads.
 */
[[nodiscard]] PlaneSegmentationResult segment_planes(
    const mrpt::maps::CPointsMap& pc, const PlaneSegmentationParams& params = {});

/** @} */

}  // namespace mp2p_icp_filters
