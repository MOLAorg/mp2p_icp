/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WeightParameters.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** Common weight parameters for OLAE and Horn's solvers. */
struct WeightParameters : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(WeightParameters, mp2p_icp)

   public:
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

    /** Relative weight of points, lines, and planes. They will be automatically
     * normalized to sum the unity, so feel free of setting weights at any
     * convenient scale.
     */
    struct PairWeights
    {
        /** Weight of point-to-point pairs. Note that finer control of weights
         * can be achieved with `Pairings::point_weights`. */
        double pt2pt = 1.0;
        double pt2ln = 1.0;  //!< Weight of point-to-line pairs
        double pt2pl = 1.0;  //!< Weight of point-to-plane pairs

        double ln2ln = 1.0;  //!< Weight of line-to-line pairs
        double pl2pl = 1.0;  //!< Weight of plane-to-plane pairs

        void load_from(const mrpt::containers::yaml& p);
        void save_to(mrpt::containers::yaml& p) const;
        void serializeTo(mrpt::serialization::CArchive& out) const;
        void serializeFrom(mrpt::serialization::CArchive& in);
    };

    /// See docs for PairWeights
    PairWeights pair_weights;

    /** @name An optional "a priori" term.
     * @{ */
    bool use_robust_kernel = false;

    /** The current guess for the sought transformation. Must be supplied if
     * use_robust_kernel==true. */
    std::optional<mrpt::poses::CPose3D> currentEstimateForRobust;

    double robust_kernel_param{mrpt::DEG2RAD(0.1)}, robust_kernel_scale{400.0};

    /** @} */

    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;
};

/** @} */

}  // namespace mp2p_icp
