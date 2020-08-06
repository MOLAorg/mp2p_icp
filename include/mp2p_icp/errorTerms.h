/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
    const mrpt::tfest::TMatchingPair& pairing, OptimalTF_Result& result,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian);

mrpt::math::CVectorFixedDouble<1> error_point2line(
    const mp2p_icp::point_line_pair_t& pairing, OptimalTF_Result& result,
    Eigen::Matrix<double, 1, 12> jacobian);

mrpt::math::CVectorFixedDouble<1> error_point2plane(
    const mp2p_icp::point_plane_pair_t& pairing, OptimalTF_Result& result,
    Eigen::Matrix<double, 1, 12> jacobian);

mrpt::math::CVectorFixedDouble<4> error_line2line(
    const mp2p_icp::matched_line_t& pairing, OptimalTF_Result& result,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& jacobian, bool jump);

mrpt::math::CVectorFixedDouble<3> error_plane2plane(
    const mp2p_icp::matched_plane_t& pairing, OptimalTF_Result& result,
    Eigen::Matrix<double, 3, 12> jacobian);

}  // namespace mp2p_icp
