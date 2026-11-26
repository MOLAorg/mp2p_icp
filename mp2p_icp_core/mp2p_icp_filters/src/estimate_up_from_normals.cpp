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
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

namespace mp2p_icp_filters
{
std::vector<mrpt::math::TVector3D> estimate_planar_normals(
    const mrpt::maps::CPointsMap& pc, const PlanarNormalsParams& params,
    std::vector<mrpt::math::TPoint3Df>* outPoints)
{
    MRPT_START

    ASSERT_GT_(params.search_radius, 0.0);
    ASSERT_GE_(params.min_neighbors, 3UL);
    ASSERT_GT_(params.parallelization_grain_size, 0UL);

    if (outPoints)
    {
        outPoints->clear();
    }

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
    for (size_t i = 0; i < N; i++)
    {
        const auto& n = normals[i];
        if (n.x == 0 && n.y == 0 && n.z == 0)
        {
            continue;
        }
        out.push_back(n);
        if (outPoints)
        {
            outPoints->emplace_back(xs[i], ys[i], zs[i]);
        }
    }
    return out;

    MRPT_END
}

double EstimateUpResult::tilt_deg() const
{
    return mrpt::RAD2DEG(std::acos(mrpt::saturate_val(up.z / up.norm(), -1.0, 1.0)));
}

std::array<double, 2> EstimateUpResult::up_sigmas_deg() const
{
    return {
        mrpt::RAD2DEG(std::sqrt(std::max(0.0, up_cov(0, 0)))),
        mrpt::RAD2DEG(std::sqrt(std::max(0.0, up_cov(1, 1))))};
}

namespace
{
enum class NormalKind : uint8_t
{
    Ignored,
    Wall,
    Flat
};

NormalKind classify(
    const mrpt::math::TVector3D& n, const mrpt::math::TVector3D& u, const EstimateUpParams& params)
{
    const double d = std::abs(n.x * u.x + n.y * u.y + n.z * u.z);
    if (d < params.wall_nz)
    {
        return NormalKind::Wall;
    }
    if (d > params.flat_nz && params.flats_weight > 0)
    {
        return NormalKind::Flat;
    }
    return NormalKind::Ignored;
}

bool fullRank(const Eigen::Matrix2d& G)
{
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(G);
    const auto&                                          ev = es.eigenvalues();
    return ev[1] > 0 && ev[0] > 1e-9 * ev[1];
}

// The smallest covariance that is at least A and at least B: B + max(A - B, 0).
Eigen::Matrix2d atLeast(const Eigen::Matrix2d& A, const Eigen::Matrix2d& B)
{
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A - B);
    const Eigen::Vector2d                                ev = es.eigenvalues().cwiseMax(0.0);
    return B + es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
}

// Unbiased for the scores summing to zero: n / (n - 1)
void scaleForGroups(Eigen::Matrix2d& G, std::size_t n)
{
    if (n > 1)
    {
        G *= static_cast<double>(n) / static_cast<double>(n - 1);
    }
}

/* Sandwich covariance of the estimated up. With u(d) = normalize(u + E d), E the
 * other two eigenvectors of M, the cost u^T M u has the Hessian
 * H = 2 diag(l1 - l0, l2 - l0) in d, and each normal adds the score
 * s = 2 w (n.u) E^T n (w > 0 for walls, w < 0 for flats). Then
 * Cov(d) = H^-1 (sum_g S_g S_g^T) H^-1, with S_g the sum of the scores of group g.
 * The normals are those of the last fit: classified against `classUp`, the up
 * that fit started from, with its population weights.
 */
void estimateUpCovariance(
    const std::vector<mrpt::math::TVector3D>& normals,
    const std::vector<mrpt::math::TPoint3Df>* points, const EstimateUpParams& params,
    const mrpt::math::CMatrixDouble33& eigVecs, const std::array<double, 3>& eigVals,
    const mrpt::math::TVector3D& classUp, double wallWeight, double flatWeight, EstimateUpResult& r)
{
    const Eigen::Vector3d       u(r.up.x, r.up.y, r.up.z);
    Eigen::Matrix<double, 3, 2> E;
    E.col(0) = eigVecs.asEigen().col(1);
    E.col(1) = eigVecs.asEigen().col(2);

    const bool                                        grouped = points && params.cluster_size > 0;
    std::map<std::array<int32_t, 3>, Eigen::Vector2d> cellScores;
    Eigen::Matrix2d                                   Gnormals = Eigen::Matrix2d::Zero();
    std::size_t                                       nNormals = 0;

    for (size_t i = 0; i < normals.size(); i++)
    {
        const auto&      nn   = normals[i];
        const NormalKind kind = classify(nn, classUp, params);
        if (kind == NormalKind::Ignored)
        {
            continue;
        }
        const double w = kind == NormalKind::Wall ? wallWeight : -flatWeight;
        if (w == 0)
        {
            continue;
        }
        const Eigen::Vector3d n(nn.x, nn.y, nn.z);
        const Eigen::Vector2d score = 2.0 * w * n.dot(u) * (E.transpose() * n);

        Gnormals += score * score.transpose();
        nNormals++;
        if (grouped)
        {
            const auto& p   = (*points)[i];
            const auto  key = std::array<int32_t, 3>{
                 static_cast<int32_t>(std::floor(p.x / params.cluster_size)),
                 static_cast<int32_t>(std::floor(p.y / params.cluster_size)),
                 static_cast<int32_t>(std::floor(p.z / params.cluster_size))};
            cellScores.try_emplace(key, Eigen::Vector2d::Zero()).first->second += score;
        }
    }
    scaleForGroups(Gnormals, nNormals);

    Eigen::Matrix2d Hinv = Eigen::Matrix2d::Zero();
    Hinv(0, 0)           = 1.0 / (2.0 * (eigVals[1] - eigVals[0]));
    Hinv(1, 1)           = 1.0 / (2.0 * (eigVals[2] - eigVals[0]));

    Eigen::Matrix2d covD = Hinv * Gnormals * Hinv;
    r.cov_groups         = nNormals;
    r.cov_reliable       = nNormals >= params.min_cov_groups && fullRank(Gnormals);

    if (grouped)
    {
        Eigen::Matrix2d Gcells = Eigen::Matrix2d::Zero();
        for (const auto& [key, S] : cellScores)
        {
            Gcells += S * S.transpose();
        }
        scaleForGroups(Gcells, cellScores.size());

        r.cov_groups   = cellScores.size();
        r.cov_reliable = cellScores.size() >= params.min_cov_groups && fullRank(Gcells);
        if (r.cov_reliable)
        {
            covD = atLeast(Hinv * Gcells * Hinv, covD);
        }
    }

    const Eigen::Matrix3d covUp = E * covD * E.transpose();
    r.up_cov.asEigen()          = covUp.topLeftCorner<2, 2>();
}

}  // namespace

EstimateUpResult estimate_up_from_normals(
    const std::vector<mrpt::math::TVector3D>& normals, const EstimateUpParams& params,
    const std::vector<mrpt::math::TPoint3Df>* points)
{
    MRPT_START

    ASSERT_GT_(params.iterations, 0UL);
    // Written so that NaN values also fail:
    ASSERTMSG_(
        params.wall_nz > 0 && params.wall_nz < params.flat_nz && params.flat_nz < 1,
        mrpt::format(
            "Thresholds must satisfy 0 < wall_nz < flat_nz < 1 (got wall_nz=%g flat_nz=%g)",
            params.wall_nz, params.flat_nz));
    ASSERTMSG_(
        params.flats_weight >= 0,
        mrpt::format("flats_weight must be >= 0 (got %g)", params.flats_weight));
    ASSERTMSG_(
        params.cluster_size >= 0,
        mrpt::format("cluster_size must be >= 0 (got %g)", params.cluster_size));
    if (points)
    {
        ASSERT_EQUAL_(points->size(), normals.size());
    }

    EstimateUpResult            r;
    mrpt::math::CMatrixDouble33 eigVecs;
    mrpt::math::TVector3D       classUp    = r.up;  // the up the last fit classified against
    double                      wallWeight = 0;
    double                      flatWeight = 0;

    for (size_t iter = 0; iter < params.iterations; iter++)
    {
        const auto u = r.up;
        classUp      = u;

        mrpt::math::CMatrixDouble33 Mw;
        mrpt::math::CMatrixDouble33 Mf;
        Mw.setZero();
        Mf.setZero();
        size_t nW = 0;
        size_t nF = 0;

        for (const auto& n : normals)
        {
            const NormalKind kind = classify(n, u, params);
            if (kind == NormalKind::Ignored)
            {
                continue;
            }
            const bool isWall = kind == NormalKind::Wall;
            auto&      M      = isWall ? Mw : Mf;
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

        wallWeight = nW > 0 ? 1.0 / static_cast<double>(nW) : 0.0;
        flatWeight = nF > 0 ? params.flats_weight / static_cast<double>(nF) : 0.0;

        mrpt::math::CMatrixDouble33 M;
        M.setZero();
        if (nW > 0)
        {
            Mw.asEigen() *= wallWeight;
            M.asEigen() += Mw.asEigen();
        }
        if (nF > 0)
        {
            // w_F (1/|F|) sum (I - n n^T) = w_F (I - (1/|F|) sum n n^T)
            Mf.asEigen() *= 1.0 / static_cast<double>(nF);
            M.asEigen() += params.flats_weight * (Eigen::Matrix3d::Identity() - Mf.asEigen());
        }

        std::vector<double> eigVals;
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
            mrpt::math::CMatrixDouble33 wallEigVecs;
            std::vector<double>         wallEigVals;
            Mw.eig_symmetric(wallEigVecs, wallEigVals, true /*sorted*/);
            for (int k = 0; k < 3; k++)
            {
                r.wallEigVals[k] = wallEigVals[k];
            }
        }
        r.single_wall_direction = r.wallEigVals[1] - r.wallEigVals[0] < params.min_eigen_gap;

        // Checked on every iteration: an unobservable start lets `up` wander off,
        // and the normals reclassified against it can then look consistent.
        ASSERTMSG_(
            r.eigVals[1] - r.eigVals[0] >= params.min_eigen_gap,
            mrpt::format(
                "The up direction is not observable from the normals (eigenvalues: %g %g %g, "
                "walls: %zu, floors/ceilings: %zu). It needs walls with at least two "
                "non-parallel directions, or floors/ceilings.",
                r.eigVals[0], r.eigVals[1], r.eigVals[2], r.wall_count, r.flat_count));
    }

    estimateUpCovariance(
        normals, points, params, eigVecs, r.eigVals, classUp, wallWeight, flatWeight, r);

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

mrpt::math::TVector3D interpolate_up(const std::vector<UpSample>& samples, double t)
{
    if (samples.empty())
    {
        return {0, 0, 1};
    }
    const auto byTime = [](const UpSample& s, double tt) { return s.t < tt; };
    const auto it     = std::lower_bound(samples.begin(), samples.end(), t, byTime);

    double ux = 0;
    double uy = 0;
    if (it == samples.begin())
    {
        ux = it->up.x;
        uy = it->up.y;
    }
    else if (it == samples.end())
    {
        ux = samples.back().up.x;
        uy = samples.back().up.y;
    }
    else
    {
        const auto&  a   = *(it - 1);
        const auto&  b   = *it;
        const double dt  = b.t - a.t;
        const double frc = dt > 0 ? (t - a.t) / dt : 1.0;
        ux               = a.up.x + frc * (b.up.x - a.up.x);
        uy               = a.up.y + frc * (b.up.y - a.up.y);
    }
    const double h2 = ux * ux + uy * uy;
    ASSERTMSG_(h2 < 1.0, "interpolate_up(): the up samples must point upwards");
    return {ux, uy, std::sqrt(1.0 - h2)};
}

std::vector<mrpt::poses::CPose3D> level_poses_from_up(
    const std::vector<double>& times, const std::vector<mrpt::poses::CPose3D>& poses,
    const std::vector<UpSample>& samples)
{
    MRPT_START

    ASSERT_EQUAL_(times.size(), poses.size());
    ASSERT_(std::is_sorted(times.begin(), times.end()));
    ASSERT_(std::is_sorted(
        samples.begin(), samples.end(),
        [](const UpSample& a, const UpSample& b) { return a.t < b.t; }));

    std::vector<mrpt::poses::CPose3D> out;
    out.reserve(poses.size());
    for (size_t k = 0; k < poses.size(); k++)
    {
        const auto Rk = rotation_aligning_up_to_z(interpolate_up(samples, times[k]));

        mrpt::math::TPoint3D p;
        if (k == 0)
        {
            p = poses[0].translation();
        }
        else
        {
            const double tMid = 0.5 * (times[k - 1] + times[k]);
            const auto   Rmid = rotation_aligning_up_to_z(interpolate_up(samples, tMid));
            const auto   step = poses[k].translation() - poses[k - 1].translation();
            p                 = out.back().translation() + Rmid.rotateVector(step);
        }
        out.push_back(mrpt::poses::CPose3D::FromRotationAndTranslation(
            Rk.getRotationMatrix() * poses[k].getRotationMatrix(), p));
    }
    return out;

    MRPT_END
}

}  // namespace mp2p_icp_filters
