/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   estimate_points_eigen.h
 * @brief  Calculate eigenvectors and eigenvalues from a set of points
 * @author Jose Luis Blanco Claraco
 * @date   July 21, 2020
 */
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/math/TPoint3D.h>  // TVector3D
#include <mrpt/poses/CPointPDFGaussian.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace mp2p_icp
{
/**
 * @brief Output of estimate_points_eigen()
 *
 * Eigen vectors and eigen values are sorted from smallest to largest
 * eigenvalue.
 *
 * \ingroup mp2p_icp_map_grp
 */
struct PointCloudEigen
{
    mrpt::poses::CPointPDFGaussian       meanCov;
    std::array<mrpt::math::TVector3D, 3> eigVectors = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
    std::array<double, 3>                eigVals    = {0, 0, 0};  //!< sorted in ascending order
};

/** Calculate mean, covariance, eigenvectors, and eigenvalues from a set of
 * points.
 *
 * It is mandatory to provide one and only one of these two parameters:
 *  - `indices`: only the points with these indices will be considered.
 *  - `totalCount`: all points will be used, ignoring `indices`.
 *
 *
 * @param xs[in] Input x coordinate vector.
 * @param ys[in] Input y coordinate vector.
 * @param zs[in] Input z coordinate vector.
 * @param indices[in] 0-based indices of points to consider from the vectors.
 * @param totalCount[in] Total number of points in the vectors.
 *
 * \exception std::exception If less than 3 points are provided.
 *
 * \ingroup mp2p_icp_map_grp
 */
PointCloudEigen estimate_points_eigen(
    const float* xs, const float* ys, const float* zs,
    mrpt::optional_ref<const std::vector<size_t>> indices,
    std::optional<size_t>                         totalCount = std::nullopt);

/** Auxiliary function that can be used to convert a vector of TPoint3Df into
 * the format expected by estimate_points_eigen() */
void vector_of_points_to_xyz(
    const std::vector<mrpt::math::TPoint3Df>& pts, std::vector<float>& xs, std::vector<float>& ys,
    std::vector<float>& zs);

}  // namespace mp2p_icp
