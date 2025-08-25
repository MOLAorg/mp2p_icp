/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   errorTerms.h
 * @brief
 * @author Francisco José Mañas Álvarez, Jose Luis Blanco Claraco
 * @date   Aug 4, 2020
 */
#pragma once

#include <mp2p_icp/OptimalTF_Result.h>
#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/WeightParameters.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CVectorFixed.h>

#include <Eigen/Dense>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

mrpt::math::CVectorFixedDouble<3> error_point2point(
    const mrpt::tfest::TMatchingPair& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian = std::nullopt);

mrpt::math::CVectorFixedDouble<3> error_point2line(
    const mp2p_icp::point_line_pair_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian = std::nullopt);

mrpt::math::CVectorFixedDouble<3> error_point2plane(
    const mp2p_icp::point_plane_pair_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian = std::nullopt);

mrpt::math::CVectorFixedDouble<4> error_line2line(
    const mp2p_icp::matched_line_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 4, 12>> jacobian = std::nullopt);

mrpt::math::CVectorFixedDouble<3> error_plane2plane(
    const mp2p_icp::matched_plane_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian = std::nullopt);

}  // namespace mp2p_icp
