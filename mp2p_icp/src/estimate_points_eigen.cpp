/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   estimate_points_eigen.cpp
 * @brief  Calculate eigenvectors and eigenvalues from a set of points
 * @author Jose Luis Blanco Claraco
 * @date   July 21, 2020
 */

#include <mp2p_icp/estimate_points_eigen.h>
#include <mrpt/core/exceptions.h>

#include <stdexcept>

mp2p_icp::PointCloudEigen mp2p_icp::estimate_points_eigen(
    const float* xs, const float* ys, const float* zs,
    mrpt::optional_ref<const std::vector<size_t>> indices,
    std::optional<size_t>                         totalCount)
{
    MRPT_START

    mrpt::math::TPoint3Df                  mean{0, 0, 0};
    mrpt::math::CMatrixFixed<double, 3, 3> mat_a;
    mat_a.setZero();

    // sanity checks:
    if (totalCount)
    {
        if (*totalCount < 3)
            THROW_EXCEPTION("totalCount: at least 3 points required.");

        const float inv_n = (1.0f / *totalCount);
        for (size_t i = 0; i < *totalCount; i++)
        {
            mean.x += xs[i];
            mean.y += ys[i];
            mean.z += zs[i];
        }
        mean *= inv_n;
        for (size_t i = 0; i < *totalCount; i++)
        {
            const auto a = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]) - mean;
            mat_a(0, 0) += a.x * a.x;
            mat_a(1, 0) += a.x * a.y;
            mat_a(2, 0) += a.x * a.z;
            mat_a(1, 1) += a.y * a.y;
            mat_a(2, 1) += a.y * a.z;
            mat_a(2, 2) += a.z * a.z;
        }
        mat_a *= inv_n;
    }
    else
    {
        ASSERTMSG_(indices, "Provide either optional<> indices or totalCount.");
        const auto& idxs = indices->get();

        if (idxs.size() < 3)
            THROW_EXCEPTION("indices: at least 3 points required.");

        const float inv_n = (1.0f / idxs.size());
        for (size_t i = 0; i < idxs.size(); i++)
        {
            const auto pt_idx = idxs[i];
            mean.x += xs[pt_idx];
            mean.y += ys[pt_idx];
            mean.z += zs[pt_idx];
        }
        mean *= inv_n;
        for (size_t i = 0; i < idxs.size(); i++)
        {
            const auto pt_idx = idxs[i];
            const auto a =
                mrpt::math::TPoint3Df(xs[pt_idx], ys[pt_idx], zs[pt_idx]) -
                mean;
            mat_a(0, 0) += a.x * a.x;
            mat_a(1, 0) += a.x * a.y;
            mat_a(2, 0) += a.x * a.z;
            mat_a(1, 1) += a.y * a.y;
            mat_a(2, 1) += a.y * a.z;
            mat_a(2, 2) += a.z * a.z;
        }
        mat_a *= inv_n;
    }

    // Complete the upper-half symmetric part of the matrix:
    mat_a(0, 1) = mat_a(1, 0);
    mat_a(0, 2) = mat_a(2, 0);
    mat_a(1, 2) = mat_a(2, 1);

    mp2p_icp::PointCloudEigen ret;
    ret.meanCov = {mrpt::poses::CPoint3D(mean), mat_a};

    // Find eigenvalues & eigenvectors:
    // This only looks at the lower-triangular part of the cov matrix.
    mrpt::math::CMatrixFixed<double, 3, 3> eigVectors;
    std::vector<double>                    eigVals;

    mat_a.eig_symmetric(eigVectors, eigVals);

    for (int i = 0; i < 3; i++)
    {
        ret.eigVals[i]    = eigVals[i];
        const auto ev     = eigVectors.extractColumn<mrpt::math::TVector3D>(i);
        ret.eigVectors[i] = {ev.x, ev.y, ev.z};
    }

    return ret;

    MRPT_END
}
