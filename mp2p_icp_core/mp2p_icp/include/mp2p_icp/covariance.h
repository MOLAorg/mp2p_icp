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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

/** Feature-detection macro for downstream packages: defined only by mp2p_icp
 * releases where every covariance() method, Censi3D included, returns the
 * `mrpt::poses::CPose3DPDFGaussian` layout, and that provide
 * jacobian_ypr_wrt_tangent(), covariance_tangent_to_ypr() and
 * information_ypr_to_tangent(). Before them, Censi3D returned the covariance of
 * the solver's SE(3) tangent instead. */
#define MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS 1

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
         *  is assumed. Reference: Censi 2007, Prakhya 2015.
         *  Computed in the solver's SE(3) tangent, then converted to the
         *  [x y z yaw pitch roll] layout with covariance_tangent_to_ypr(). */
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
 * Whatever the method, the result is in the layout of
 * `mrpt::poses::CPose3DPDFGaussian::cov`: `[x y z yaw pitch roll]`, with the
 * translation in the frame `finalAlignSolution` is expressed in (the "global"
 * one), as stored in Results::optimal_tf.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 covariance(
    const Pairings& finalPairings, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& p);

/** Jacobian of the `[x y z yaw pitch roll]` coordinates of `pose · exp(eps)`
 * with respect to `eps`, at `eps = 0`. `eps = [rho, omega]` is the right
 * (body-frame) perturbation of `mrpt::poses::Lie::SE<3>`, translation first:
 * the tangent the ICP solvers linearize in, and the one of the `prior`
 * information of ICP::align().
 *
 * It is block diagonal: the rotation `R` of `pose` turns the body-frame
 * translation into the global one, and the lower block turns body angular
 * rates into rates of yaw, pitch and roll. That block is singular at
 * pitch = +-90 deg, where yaw and roll are undefined; it is kept finite there.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 jacobian_ypr_wrt_tangent(const mrpt::poses::CPose3D& pose);

/** Converts the covariance of the right tangent perturbation of `pose` (see
 * jacobian_ypr_wrt_tangent()) into the `mrpt::poses::CPose3DPDFGaussian` layout
 * `[x y z yaw pitch roll]`: `J * covTangent * J^T`.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 covariance_tangent_to_ypr(
    const mrpt::math::CMatrixDouble66& covTangent, const mrpt::poses::CPose3D& pose);

/** Converts an information matrix in the `mrpt::poses::CPose3DPDFGaussianInf`
 * layout `[x y z yaw pitch roll]` into the right tangent of `pose` (see
 * jacobian_ypr_wrt_tangent()), which is what the `prior` of ICP::align() and
 * OptimalTF_GN_Parameters::prior expect: `J^T * infoYpr * J`.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 information_ypr_to_tangent(
    const mrpt::math::CMatrixDouble66& infoYpr, const mrpt::poses::CPose3D& pose);

}  // namespace mp2p_icp

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp, mp2p_icp::CovarianceParameters::Method)
MRPT_FILL_ENUM(CovarianceParameters::Method::InverseHessian);
MRPT_FILL_ENUM_CUSTOM_NAME(CovarianceParameters::Method::InverseHessian, "InverseHessian");
MRPT_FILL_ENUM(CovarianceParameters::Method::Censi3D);
MRPT_FILL_ENUM_CUSTOM_NAME(CovarianceParameters::Method::Censi3D, "Censi3D");
MRPT_ENUM_TYPE_END()
