/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_olae.h
 * @brief  OLAE algorithm to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/optimal_tf_common.h>
#include <cstdlib>
#include <map>
#include <string>

namespace mp2p_icp
{
struct Pairings_OLAE
{
    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_points;

    /** Weights for paired_points: each entry specifies how many points have the
     * given (mapped second value) weight, in the same order as stored in
     * paired_points.
     * If empty, all point layers will have equal weights. If weights is
     * provided for only some layers, the layers not listed here will be totally
     * ignored.
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

    /** The current guess for the sought transformation. Must be supplied if
     * use_robust_kernel==true. */
    const mrpt::poses::CPose3D current_estimate_for_robust;
    bool                       use_robust_kernel{false};
    double robust_kernel_param{mrpt::DEG2RAD(0.1)}, robust_kernel_scale{400.0};

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

    /** Threshold for doing relinearization (2nd stage). Refer to technical
     * report. */
    double OLAE_relinearize_threshold{mrpt::DEG2RAD(170.0)};

    /// See docs for Weights
    Weights weights;

    bool empty() const
    {
        return paired_points.empty() && paired_planes.empty();
    }
};

/** OLAE algorithm to find the SE(3) optimal transformation given a set of
 * correspondences between points-points, lines-lines, planes-planes. Refer to
 * technical report: Jose-Luis Blanco-Claraco. OLAE-ICP: Robust and fast
 * alignment of geometric features with the optimal linear attitude estimator,
 * Arxiv 2019.
 */
void optimal_tf_olae(const Pairings_OLAE& in, OptimalTF_Result& result);

}  // namespace mp2p_icp
