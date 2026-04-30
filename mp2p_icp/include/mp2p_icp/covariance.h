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
 * @file   covariance.h
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mp2p_icp
{
struct CovarianceParameters
{
    /** Covariance estimation backend. */
    enum class Method : uint8_t
    {
        /** Inverse of the (whitened) Gauss-Newton Hessian, scaled by the
         *  a-posteriori unit-weight variance chi^2/(m-n). Cheap but typically
         *  optimistic, since it ignores correspondence and surface-mismatch
         *  noise. */
        InverseHessian,

        /** Censi-style sandwich  H^{-1} M H^{-1}, with H = sum J_i^T J_i and
         *  M = sum J_i^T Sigma_z_i J_i. For cov2cov pairings the per-pair
         *  Sigma_z = inv(cov_inv) is used directly; for pt2pt without
         *  per-pair noise, an isotropic Sigma_z = defaultPointSigma^2 * I
         *  is assumed. Reference: Censi 2007, Prakhya 2015. */
        Censi3D,
    };

    Method method = Method::InverseHessian;

    /** For pt2pt pairings without per-pair Sigma in Censi3D path:
     *  isotropic point-noise std-dev [m]. */
    double defaultPointSigma = 0.02;

    /** Per-axis floor added to the resulting covariance diagonal. Stored as
     *  std-dev for readability; squared internally. Useful to absorb
     *  unmodelled errors and keep downstream filters numerically stable. */
    double floor_sigma_xyz    = 0.0;  ///< [m]
    double floor_sigma_angles = 0.0;  ///< [rad]

    // Finite difference deltas (only used by InverseHessian path):
    double finDif_xyz    = 1e-7;
    double finDif_angles = 1e-7;

    /** Loads the parameters from a YAML map node. All entries are optional;
     *  unspecified fields keep their default values. Recognized keys:
     *
     *    method:               "InverseHessian" | "Censi3D"
     *    defaultPointSigma:    [m]
     *    floor_sigma_xyz:      [m]
     *    floor_sigma_angles:   [rad]   (use floor_sigma_angles_deg for degrees)
     *    floor_sigma_angles_deg: [deg] (alternative to the radian form)
     *    finDif_xyz, finDif_angles
     */
    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;
};

/** Covariance estimation methods for an ICP result.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 covariance(
    const Pairings& finalPairings, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& p);

}  // namespace mp2p_icp

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp, mp2p_icp::CovarianceParameters::Method)
MRPT_FILL_ENUM(CovarianceParameters::Method::InverseHessian);
MRPT_FILL_ENUM_CUSTOM_NAME(CovarianceParameters::Method::InverseHessian, "InverseHessian");
MRPT_FILL_ENUM(CovarianceParameters::Method::Censi3D);
MRPT_FILL_ENUM_CUSTOM_NAME(CovarianceParameters::Method::Censi3D, "Censi3D");
MRPT_ENUM_TYPE_END()
