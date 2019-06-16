/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
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

struct Pairings_GaussNewton
{
    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_points;

    /** Weights for paired_points: each entry specifies how many points have the
     * given (mapped second value) weight, in the same order as stored in
     * paired_points */
    std::vector<std::pair<std::size_t, double>> point_weights;

    TMatchedPointPlaneList paired_pt2pl;

    bool                 use_robust_kernel{true};
    double               robust_kernel_param{0.05};  /// [meters]
    std::size_t          max_iterations{25};
    double               min_delta{1e-9};
    mrpt::poses::CPose3D initial_guess;

    bool empty() const { return paired_points.empty() && paired_pt2pl.empty(); }
};

/** Gauss-Newton non-linear, iterative optimizer to find the SE(3) optimal
 * transformation between a set of correspondences.
 */
void optimal_tf_gauss_newton(
    const Pairings_GaussNewton& in, OptimalTF_Result& result);

}  // namespace mp2p_icp
