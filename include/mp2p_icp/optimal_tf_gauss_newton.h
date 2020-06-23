/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_gauss_newton.h
 * @brief  Simple non-linear optimizer to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/optimal_tf_common.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct point_plane_pair_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    plane_patch_t         pl_this;
    mrpt::math::TPoint3Df pt_other;

    point_plane_pair_t() = default;
    point_plane_pair_t(
        const plane_patch_t& p_this, const mrpt::math::TPoint3Df& p_other)
        : pl_this(p_this), pt_other(p_other)
    {
    }
};
using TMatchedPointPlaneList = std::vector<point_plane_pair_t>;

struct point_line_pair_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    mrpt::math::TLine3D     ln_this;
    mrpt::math::TPoint3D    pt_other;

    point_line_pair_t() = default;
    point_line_pair_t(
            const mrpt::math::TLine3D& l_this, const mrpt::math::TPoint3D& p_other)
            : ln_this(l_this),pt_other(p_other)
    {
    }
};

using TMatchedPointLineList = std::vector<point_line_pair_t>;

struct Pairings_GaussNewton : public PairingsCommon
{
    /** In addition to base members in PairingsCommon, the Gauss-Newton method
     * supports point-to-line pairings:
     */
    TMatchedPointLineList paired_pt2ln;

    /** In addition to base members in PairingsCommon, the Gauss-Newton method
     * supports point-to-plane pairings:
     */
    TMatchedPointPlaneList paired_pt2pl;

    /** Default weight (i.e. scalar information matrix) of point-to-point
     * pairings. \sa point_weights */
    double weight_point2point{1.0};

    /** Weight (i.e. scalar information matrix) of point-to-line pairings */
    double weight_point2line{1.0};

    /** Weight (i.e. scalar information matrix) of point-to-plane pairings */
    double weight_point2plane{1.0};

    /** Weight (i.e. scalar information matrix) of plane-to-plane pairings */
    double weight_plane2plane{1.0};

    /** If non-empty, overrides weight_point2point.
     * This contains weights for paired_points: each entry specifies how many
     * points have the given (mapped second value) weight, in the same order as
     * stored in paired_points. */
    std::vector<std::pair<std::size_t, double>> point_weights;

    bool                 use_robust_kernel{true};
    double               robust_kernel_param{0.05};  /// [meters]
    std::size_t          max_iterations{20};
    double               min_delta{1e-6};
    mrpt::poses::CPose3D initial_guess;

    bool verbose{false};

    bool empty() const { return paired_points.empty() && paired_pt2pl.empty() && paired_pt2ln.empty(); }
};

/** Gauss-Newton non-linear, iterative optimizer to find the SE(3) optimal
 * transformation between a set of correspondences.
 */
void optimal_tf_gauss_newton(
    const Pairings_GaussNewton& in, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
