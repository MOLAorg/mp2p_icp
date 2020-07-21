/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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
 * Eigen vectors and eigen values are sorted from largest to smallest.
 */
struct PointCloudEigen
{
    mrpt::poses::CPointPDFGaussian       meanCov;
    std::array<mrpt::math::TVector3D, 3> eigVectors = {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
    std::array<double, 3> eigVals = {0, 0, 0};
};

/** Calculate eigenvectors and eigenvalues from a set of points.
 *
 * If `indices` is provided, only those set of points will be considered.
 * If `totalCount` is provided, all points will be provided, ignoring `indices`.
 *
 * @param xs[in] Input x coordinate vector.
 * @param ys[in] Input y coordinate vector.
 * @param zs[in] Input z coordinate vector.
 * @param indices[in] 0-based indices of points to consider from the vectors.
 * @param totalCount[in] Total number of points in the vectors.
 *
 * \exception std::exception If less than 3 points are provided.
 *
 * \ingroup mp2p_icp_grp
 */
PointCloudEigen estimate_points_eigen(
    const float* xs, const float* ys, const float* zs,
    mrpt::optional_ref<const std::vector<size_t>> indices,
    std::optional<size_t>                         totalCount = std::nullopt);

}  // namespace mp2p_icp
