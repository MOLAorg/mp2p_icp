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
 * @file   estimate_up_from_normals.cpp
 * @brief  Estimates the "up" direction of a map from wall and floor normals.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 8, 2026
 */

#include <mp2p_icp/estimate_points_eigen.h>
#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/geometry.h>

#include <Eigen/Dense>
#include <cmath>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

namespace mp2p_icp_filters
{
std::vector<mrpt::math::TVector3D> estimate_planar_normals(
    const mrpt::maps::CPointsMap& pc, const PlanarNormalsParams& params)
{
    MRPT_START

    ASSERT_GT_(params.search_radius, 0.0);
    ASSERT_GE_(params.min_neighbors, 3UL);
    ASSERT_GT_(params.parallelization_grain_size, 0UL);

    const size_t N = pc.size();
    if (N == 0)
    {
        return {};
    }

    // A null vector marks "no normal" for that point:
    std::vector<mrpt::math::TVector3D> normals(N, {0, 0, 0});

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    pc.nn_prepare_for_3d_queries();

    const auto radiusSqr = static_cast<float>(params.search_radius * params.search_radius);

    auto lambda_process = [&](size_t i, std::vector<mrpt::math::TPoint3Df>& nnPts,
                              std::vector<float>& nnDistSqr, std::vector<uint64_t>& nnIdxs,
                              std::vector<size_t>& indices)
    {
        pc.nn_radius_search(
            {xs[i], ys[i], zs[i]}, radiusSqr, nnPts, nnDistSqr, nnIdxs, 0 /*no limit*/);

        if (nnIdxs.size() < params.min_neighbors)
        {
            return;
        }
        indices.assign(nnIdxs.begin(), nnIdxs.end());

        const auto  eig = mp2p_icp::estimate_points_eigen(xs.data(), ys.data(), zs.data(), indices);
        const auto& ev  = eig.eigVals;
        if (!(ev[0] < params.max_planarity_ratio * ev[1]))
        {
            return;
        }
        const auto& n    = eig.eigVectors[0];
        const auto  norm = n.norm();
        if (!(norm > 0))
        {
            return;
        }
        normals[i] = n * (1.0 / norm);
    };

#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, N, params.parallelization_grain_size),
        [&](const tbb::blocked_range<size_t>& r)
        {
            std::vector<mrpt::math::TPoint3Df> nnPts;
            std::vector<float>                 nnDistSqr;
            std::vector<uint64_t>              nnIdxs;
            std::vector<size_t>                indices;
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                lambda_process(i, nnPts, nnDistSqr, nnIdxs, indices);
            }
        });
#else
    std::vector<mrpt::math::TPoint3Df> nnPts;
    std::vector<float>                 nnDistSqr;
    std::vector<uint64_t>              nnIdxs;
    std::vector<size_t>                indices;
    for (size_t i = 0; i < N; ++i)
    {
        lambda_process(i, nnPts, nnDistSqr, nnIdxs, indices);
    }
#endif

    // Compact, keeping the point index order:
    std::vector<mrpt::math::TVector3D> out;
    out.reserve(N);
    for (const auto& n : normals)
    {
        if (n.x != 0 || n.y != 0 || n.z != 0)
        {
            out.push_back(n);
        }
    }
    return out;

    MRPT_END
}

double EstimateUpResult::tilt_deg() const
{
    return mrpt::RAD2DEG(std::acos(mrpt::saturate_val(up.z / up.norm(), -1.0, 1.0)));
}

EstimateUpResult estimate_up_from_normals(
    const std::vector<mrpt::math::TVector3D>& normals, const EstimateUpParams& params)
{
    MRPT_START

    ASSERT_GT_(params.iterations, 0UL);
    ASSERT_LT_(params.wall_nz, params.flat_nz);

    EstimateUpResult r;

    for (size_t iter = 0; iter < params.iterations; iter++)
    {
        const auto& u = r.up;

        mrpt::math::CMatrixDouble33 Mw;
        mrpt::math::CMatrixDouble33 Mf;
        Mw.setZero();
        Mf.setZero();
        size_t nW = 0;
        size_t nF = 0;

        for (const auto& n : normals)
        {
            const double d      = std::abs(n.x * u.x + n.y * u.y + n.z * u.z);
            const bool   isWall = d < params.wall_nz;
            const bool   isFlat = d > params.flat_nz;
            if (!isWall && !isFlat)
            {
                continue;
            }
            auto& M = isWall ? Mw : Mf;
            if (isWall)
            {
                nW++;
            }
            else
            {
                nF++;
            }
            for (int a = 0; a < 3; a++)
            {
                for (int b = a; b < 3; b++)
                {
                    M(a, b) += n[a] * n[b];
                }
            }
        }
        for (int a = 0; a < 3; a++)
        {
            for (int b = 0; b < a; b++)
            {
                Mw(a, b) = Mw(b, a);
                Mf(a, b) = Mf(b, a);
            }
        }

        if (nW < params.min_population)
        {
            nW = 0;
        }
        if (nF < params.min_population)
        {
            nF = 0;
        }
        ASSERTMSG_(
            nW + nF > 0,
            mrpt::format(
                "Not enough wall or floor/ceiling normals (minimum: %zu of either kind).",
                params.min_population));

        mrpt::math::CMatrixDouble33 M;
        M.setZero();
        if (nW > 0)
        {
            Mw.asEigen() *= 1.0 / static_cast<double>(nW);
            M.asEigen() += Mw.asEigen();
        }
        if (nF > 0)
        {
            // (1/|F|) sum (I - n n^T) = I - (1/|F|) sum n n^T
            Mf.asEigen() *= 1.0 / static_cast<double>(nF);
            M.asEigen() += Eigen::Matrix3d::Identity() - Mf.asEigen();
        }

        mrpt::math::CMatrixDouble33 eigVecs;
        std::vector<double>         eigVals;
        M.eig_symmetric(eigVecs, eigVals, true /*sorted*/);

        mrpt::math::TVector3D newUp(eigVecs(0, 0), eigVecs(1, 0), eigVecs(2, 0));
        newUp *= 1.0 / newUp.norm();
        if (newUp.z < 0)
        {
            newUp = -newUp;
        }

        r.up         = newUp;
        r.wall_count = nW;
        r.flat_count = nF;
        for (int k = 0; k < 3; k++)
        {
            r.eigVals[k] = eigVals[k];
        }

        r.wallEigVals = {0, 0, 0};
        if (nW > 0)
        {
            std::vector<double> wallEigVals;
            Mw.eig_symmetric(eigVecs, wallEigVals, true /*sorted*/);
            for (int k = 0; k < 3; k++)
            {
                r.wallEigVals[k] = wallEigVals[k];
            }
        }
        r.single_wall_direction = r.wallEigVals[1] - r.wallEigVals[0] < params.min_eigen_gap;
    }

    ASSERTMSG_(
        r.eigVals[1] - r.eigVals[0] >= params.min_eigen_gap,
        mrpt::format(
            "The up direction is not observable from the normals (eigenvalues: %g %g %g, "
            "walls: %zu, floors/ceilings: %zu). It needs walls with at least two "
            "non-parallel directions, or floors/ceilings.",
            r.eigVals[0], r.eigVals[1], r.eigVals[2], r.wall_count, r.flat_count));

    return r;

    MRPT_END
}

mrpt::poses::CPose3D rotation_aligning_up_to_z(const mrpt::math::TVector3D& up)
{
    const auto                  u = up * (1.0 / up.norm());
    const mrpt::math::TVector3D z = {0, 0, 1};

    // Rodrigues: v = u x z, R = I + [v]x + [v]x^2 (1 - c) / s^2
    const auto   v = mrpt::math::crossProduct3D(u, z);
    const double s = v.norm();
    const double c = u.z;

    mrpt::math::CMatrixDouble33 R;
    R.setIdentity();
    if (s > 1e-12)
    {
        mrpt::math::CMatrixDouble33 K;
        K.setZero();
        K(0, 1) = -v.z;
        K(0, 2) = v.y;
        K(1, 0) = v.z;
        K(1, 2) = -v.x;
        K(2, 0) = -v.y;
        K(2, 1) = v.x;
        R.asEigen() += K.asEigen() + K.asEigen() * K.asEigen() * ((1.0 - c) / (s * s));
    }
    else
    {
        ASSERTMSG_(c > 0, "rotation_aligning_up_to_z(): 'up' points downwards");
    }

    return mrpt::poses::CPose3D::FromRotationAndTranslation(R, mrpt::math::TVector3D(0, 0, 0));
}

}  // namespace mp2p_icp_filters
