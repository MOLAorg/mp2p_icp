/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_OLAE.h
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */
#pragma once

#include <mp2p_icp/ICP_Base.h>
#include <mp2p_icp/pointcloud.h>

namespace mp2p_icp
{
/** ICP registration for points, planes, and lines.
 * Refer to technical report: XXX
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_OLAE : public ICP_Base
{
    DEFINE_MRPT_OBJECT(ICP_OLAE)

   public:
    // See base class docs
    void align(
        const pointcloud_t& pc1, const pointcloud_t& pc2,
        const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
        Results& result) override;

   protected:
};

void align_OLAE(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result);

// TODO: From this point on, refactor to its own OLAE.h header

struct matched_plane_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
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

struct OLAE_Match_Input
{
    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_points;

    /** Weights for paired_points: each entry specifies how many points have the
     * given (mapped second value) weight, in the same order as stored in
     * paired_points.
     * If empty, all points will have equal weights.
     */
    std::vector<std::pair<std::size_t, double>> point_weights;

    TMatchedLineList  paired_lines;
    TMatchedPlaneList paired_planes;

    /** Relative weight of points, lines, and planes. They will be automatically
     * normalized to sum the unity, so feel free of setting weights at any
     * convenient scale. Weights are used in two steps: in the orientation cost
     * function, and in the evaluation of the centroids to find the final
     * translation; hence we have two sets of weights.
     */
    struct Weights
    {
        struct Attitude
        {
            double points{1.0};  //!< Weight for points in attitude estimation
            double lines{1.0};  //!< Weight for lines in attitude estimation
            double planes{1.0};  //!< Weight for planes in attitude estimation
        } attitude;

        struct Translation
        {
            double points{1.0};  //!< Points weight in translation estimation
            double planes{1.0};  //!< Planes weight in translation estimation
        } translation;
    };

    bool   use_robust_kernel{true};
    double robust_kernel_param{mrpt::DEG2RAD(0.5)}, robust_kernel_scale{400.0};

    /// See docs for Weights
    Weights weights;

    bool empty() const
    {
        return paired_points.empty() && paired_planes.empty();
    }
};

struct OLAE_Match_Result
{
    mrpt::poses::CPose3D optimal_pose;
};

/** The points-and-planes optimal pose solver.
 * Refer to technical report: XXX
 */
void olae_match(const OLAE_Match_Input& in, OLAE_Match_Result& result);

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

// TODO: From this point on, refactor to its own GaussNewton solver.
#if 0

struct P2P_Match_Input
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

/** points-and-planes optimal pose solver.
 */
void p2p_match(const P2P_Match_Input& in, P2P_Match_Result& result);
#endif

struct P2P_Match_Result
{
    mrpt::poses::CPose3D optimal_pose;
};

}  // namespace mp2p_icp
