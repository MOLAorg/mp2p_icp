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
 * @file   level_by_place.cpp
 * @brief  Levels a trajectory with a vertical correction that depends on place.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mp2p_icp_filters/level_by_place.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>

namespace mp2p_icp_filters
{
namespace
{
Eigen::Vector3d toEigen(const mrpt::math::TPoint3D& p) { return {p.x, p.y, p.z}; }

mrpt::math::TVector3D unit(const mrpt::math::TVector3D& v)
{
    const double n = v.norm();
    ASSERTMSG_(n > 0 && std::isfinite(n), "level_by_place: invalid up vector");
    return v * (1.0 / n);
}

Eigen::Matrix3d correction(const mrpt::math::TVector3D& up)
{
    return rotation_aligning_up_to_z(up).getRotationMatrix().asEigen();
}

}  // namespace

PlaceLevelingResult level_poses_by_place(
    const std::vector<mrpt::poses::CPose3D>& poses, const std::vector<mrpt::math::TVector3D>& ups,
    const std::vector<WeightedEdge>& links, const PlaceLevelingParams& params)
{
    MRPT_START
    const size_t N = poses.size();
    ASSERT_EQUAL_(ups.size(), N);
    ASSERT_GT_(params.sequential_weight, 0.0);

    PlaceLevelingResult out;
    if (N == 0)
    {
        return out;
    }

    std::vector<mrpt::math::TVector3D> u;
    std::vector<Eigen::Matrix3d>       C;
    u.reserve(N);
    C.reserve(N);
    for (const auto& v : ups)
    {
        u.push_back(unit(v));
        C.push_back(correction(u.back()));
    }

    // All edges: consecutive keyframes first, then the links
    std::vector<WeightedEdge> edges;
    edges.reserve(N + links.size());
    for (size_t k = 0; k + 1 < N; k++)
    {
        edges.push_back(
            {static_cast<uint32_t>(k), static_cast<uint32_t>(k + 1), params.sequential_weight});
    }
    const size_t nSeq = edges.size();
    for (const auto& e : links)
    {
        ASSERTMSG_(e.i < N && e.j < N, "level_poses_by_place(): link index out of range");
        ASSERTMSG_(e.w >= 0 && std::isfinite(e.w), "level_poses_by_place(): invalid link weight");
        if (e.i != e.j && e.w > 0)
        {
            edges.push_back(e);
        }
    }

    // Target relative position of each edge:
    std::vector<Eigen::Vector3d> target;
    target.reserve(edges.size());
    for (const auto& e : edges)
    {
        const Eigen::Matrix3d Cij = correction(unit(u[e.i] + u[e.j]));
        target.push_back(
            Cij * (toEigen(poses[e.j].translation()) - toEigen(poses[e.i].translation())));
    }

    // Normal equations, with p'_0 = p_0 fixed (unknown k-1 for keyframe k >= 1):
    std::vector<Eigen::Vector3d> pos(N);
    pos[0] = toEigen(poses[0].translation());
    if (N > 1)
    {
        const auto                          M = static_cast<Eigen::Index>(N - 1);
        std::vector<Eigen::Triplet<double>> trip;
        trip.reserve(4 * edges.size());
        Eigen::MatrixXd b = Eigen::MatrixXd::Zero(M, 3);
        for (size_t k = 0; k < edges.size(); k++)
        {
            const auto&  e  = edges[k];
            const auto   ii = static_cast<Eigen::Index>(e.i) - 1;
            const auto   jj = static_cast<Eigen::Index>(e.j) - 1;
            const double w  = e.w;
            // Residual: x_j - x_i - target
            if (jj >= 0)
            {
                trip.emplace_back(jj, jj, w);
                b.row(jj) += w * target[k].transpose();
                if (ii < 0)
                {
                    b.row(jj) += w * pos[0].transpose();
                }
            }
            if (ii >= 0)
            {
                trip.emplace_back(ii, ii, w);
                b.row(ii) -= w * target[k].transpose();
                if (jj < 0)
                {
                    b.row(ii) += w * pos[0].transpose();
                }
            }
            if (ii >= 0 && jj >= 0)
            {
                trip.emplace_back(ii, jj, -w);
                trip.emplace_back(jj, ii, -w);
            }
        }
        Eigen::SparseMatrix<double> H(M, M);
        H.setFromTriplets(trip.begin(), trip.end());

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(H);
        ASSERTMSG_(
            solver.info() == Eigen::Success, "level_poses_by_place(): the position solve failed");
        const Eigen::MatrixXd x = solver.solve(b);
        ASSERTMSG_(
            solver.info() == Eigen::Success && x.allFinite(),
            "level_poses_by_place(): the position solve failed");
        for (size_t k = 1; k < N; k++)
        {
            pos[k] = x.row(static_cast<Eigen::Index>(k - 1)).transpose();
        }
    }

    out.poses.reserve(N);
    for (size_t k = 0; k < N; k++)
    {
        mrpt::math::CMatrixDouble33 R;
        R.asEigen() = C[k] * poses[k].getRotationMatrix().asEigen();
        out.poses.push_back(mrpt::poses::CPose3D::FromRotationAndTranslation(
            R, mrpt::math::TVector3D(pos[k].x(), pos[k].y(), pos[k].z())));
    }

    // Residual diagnostics, per edge kind:
    double sumSeq  = 0;
    double sumLink = 0;
    for (size_t k = 0; k < edges.size(); k++)
    {
        const double r = (pos[edges[k].j] - pos[edges[k].i] - target[k]).norm();
        if (k < nSeq)
        {
            sumSeq += r * r;
            out.sequential_residual_max = std::max(out.sequential_residual_max, r);
        }
        else
        {
            sumLink += r * r;
            out.link_residual_max = std::max(out.link_residual_max, r);
        }
    }
    if (nSeq > 0)
    {
        out.sequential_residual_rms = std::sqrt(sumSeq / static_cast<double>(nSeq));
    }
    if (edges.size() > nSeq)
    {
        out.link_residual_rms = std::sqrt(sumLink / static_cast<double>(edges.size() - nSeq));
    }
    return out;
    MRPT_END
}

std::vector<mrpt::poses::CPose3D> level_path_by_place(
    const std::vector<double>& pathTimes, const std::vector<mrpt::poses::CPose3D>& pathPoses,
    const std::vector<double>& kfTimes, const std::vector<mrpt::poses::CPose3D>& kfBefore,
    const std::vector<mrpt::poses::CPose3D>&  kfAfter,
    const std::vector<mrpt::math::TVector3D>& kfUps)
{
    MRPT_START
    ASSERT_EQUAL_(pathTimes.size(), pathPoses.size());
    const size_t N = kfTimes.size();
    ASSERT_GE_(N, 1UL);
    ASSERT_EQUAL_(kfBefore.size(), N);
    ASSERT_EQUAL_(kfAfter.size(), N);
    ASSERT_EQUAL_(kfUps.size(), N);
    ASSERTMSG_(
        std::is_sorted(kfTimes.begin(), kfTimes.end()),
        "level_path_by_place(): keyframe times must be sorted");

    std::vector<mrpt::math::TVector3D> u;
    u.reserve(N);
    for (const auto& v : kfUps)
    {
        u.push_back(unit(v));
    }

    // Where keyframe k takes point p: its leveled position plus the rotated offset
    const auto moved = [&](size_t k, const Eigen::Vector3d& p) -> Eigen::Vector3d
    {
        return toEigen(kfAfter[k].translation()) +
               correction(u[k]) * (p - toEigen(kfBefore[k].translation()));
    };

    std::vector<mrpt::poses::CPose3D> out;
    out.reserve(pathPoses.size());
    for (size_t i = 0; i < pathPoses.size(); i++)
    {
        const double t  = pathTimes[i];
        const auto   it = std::upper_bound(kfTimes.begin(), kfTimes.end(), t);
        size_t       a  = 0;
        size_t       b  = 0;
        double       f  = 0;
        if (it == kfTimes.end())
        {
            a = b = N - 1;
        }
        else if (it != kfTimes.begin())
        {
            b              = static_cast<size_t>(it - kfTimes.begin());
            a              = b - 1;
            const double d = kfTimes[b] - kfTimes[a];
            f              = d > 0 ? (t - kfTimes[a]) / d : 0.0;
        }
        const Eigen::Vector3d p  = toEigen(pathPoses[i].translation());
        const Eigen::Vector3d pp = (1.0 - f) * moved(a, p) + f * moved(b, p);
        const auto            up = unit(u[a] * (1.0 - f) + u[b] * f);

        mrpt::math::CMatrixDouble33 R;
        R.asEigen() = correction(up) * pathPoses[i].getRotationMatrix().asEigen();
        out.push_back(mrpt::poses::CPose3D::FromRotationAndTranslation(
            R, mrpt::math::TVector3D(pp.x(), pp.y(), pp.z())));
    }
    return out;
    MRPT_END
}

}  // namespace mp2p_icp_filters
