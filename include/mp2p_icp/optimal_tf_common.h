/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   OptimalTF_common.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/pointcloud.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct matched_plane_t
{
    /** \note "this"=global, "other"=local, while finding the transformation
     * local wrt global
     */
    plane_patch_t p_this, p_other;

    matched_plane_t() = default;
    matched_plane_t(const plane_patch_t& pl_this, const plane_patch_t& pl_other)
        : p_this(pl_this), p_other(pl_other)
    {
    }
};
using TMatchedPlaneList = std::vector<matched_plane_t>;

struct matched_line_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    mrpt::math::TLine3D l_this, l_other;
};
using TMatchedLineList = std::vector<matched_line_t>;

/** Vector of pairings that are considered outliers, from those in the
 * corresponding `PairingsCommon` structure.
 *
 * \note Indices are always assumed to be sorted in these containers.
 */
struct OutlierIndices
{
    std::vector<std::size_t> point2point;
    std::vector<std::size_t> line2line;
    std::vector<std::size_t> plane2plane;

    inline bool empty() const
    {
        return point2point.empty() && line2line.empty() && plane2plane.empty();
    }
    inline std::size_t size() const
    {
        return point2point.size() + line2line.size() + plane2plane.size();
    }
};

/** The is the output structure for all optimal transformation methods.
 */
struct OptimalTF_Result
{
    mrpt::poses::CPose3D optimal_pose;
    double               optimal_scale{1.0};

    /** Correspondence that were detected as outliers. */
    OutlierIndices outliers;
};

/** Common pairing input data for OLAE and Horn's solvers. */
struct PairingsCommon
{
    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_points;
    TMatchedLineList               paired_lines;
    TMatchedPlaneList              paired_planes;

    virtual bool empty() const
    {
        return paired_points.empty() && paired_planes.empty() &&
               paired_lines.empty();
    }
};

/** Common weight parameters for OLAE and Horn's solvers. */
struct WeightParameters
{
    /** Enables the use of the scale-based outlier detector. Refer to the
     * technical report.  This robustness feature is independent from
     * use_robust_kernel.
     */
    bool use_scale_outlier_detector{true};

    /** If use_scale_outlier_detector==true, discard a potential point-to-point
     * pairing if the ratio between the norm of their final vectors is larger
     * than this value. A value of "1.0" will only allow numerically perfect
     * pairings, so a slightly larger value is required. The closer to 1, the
     * stricter. A much larger threshold (e.g. 5.0) would only reject the
     * most obvious outliers. Refer to the technical report. */
    double scale_outlier_threshold{1.20};

    /** *Individual* weights for paired_points: each entry specifies how many
     * points have the given (mapped second value) weight, in the same order as
     * stored in paired_points. If empty, all points will have equal weights.
     */
    std::vector<std::pair<std::size_t, double>> point_weights;

    /** Relative weight of points, lines, and planes. They will be automatically
     * normalized to sum the unity, so feel free of setting weights at any
     * convenient scale. Weights are used in two steps: in the orientation cost
     * function, and in the evaluation of the centroids to find the final
     * translation. At present, only point-to-point correspondences are used (in
     * the family of methods inheriting from this structure) to solve for
     * translation, hence there are only weights for the attitude solution here.
     * Note that finer control of weights can be achieved with `point_weights`.
     */
    struct AttitudeWeights
    {
        double pt2pt{1.0};  //!< Weight of point-to-point pairs
        double l2l{1.0};  //!< Weight of line-to-line pairs
        double pl2pl{1.0};  //!< Weight of plane-to-plane pairs
    };

    /// See docs for Weights
    AttitudeWeights attitude_weights;

    /** The current guess for the sought transformation. Must be supplied if
     * use_robust_kernel==true. */
    mrpt::poses::CPose3D current_estimate_for_robust;
    bool                 use_robust_kernel{false};
    double robust_kernel_param{mrpt::DEG2RAD(0.1)}, robust_kernel_scale{400.0};
};

struct WeightedPairings : public PairingsCommon, public WeightParameters
{
};

/** Evaluates the centroids [ct_other, ct_this] for point-to-point
 * correspondences only, taking into account the current guess for outliers
 */
std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D> eval_centroids_robust(
    const PairingsCommon& in, const OutlierIndices& outliers);

/** @} */

}  // namespace mp2p_icp
