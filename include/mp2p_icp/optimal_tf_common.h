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
#include <mrpt/core/optional_ref.h>

namespace mp2p_icp
{
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

/** The is the output structure for all optimal transformation methods.
 */
struct OptimalTF_Result
{
    mrpt::poses::CPose3D optimal_pose;
    double               optimal_scale{1.0};

    /** If provided, will be filled in with a boolean indicating whether each of
     * the input pairings was detected as an outlier (true) or inlier (false).
     * The vector may be left unmodified if the underlying optimal
     * transformation method does not handle outliers (non-robust estimators).
     */
    mrpt::optional_ref<std::vector<bool>> outliers;
};

}  // namespace mp2p_icp
