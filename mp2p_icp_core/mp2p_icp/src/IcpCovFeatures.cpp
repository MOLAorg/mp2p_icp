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
 * @file   IcpCovFeatures.cpp
 * @brief  Framework-independent evaluation of a registration result.
 */

#include <mp2p_icp/IcpCovFeatures.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace mp2p_icp;

namespace
{
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

constexpr size_t kMinAssociations    = 30;
constexpr size_t kMinNormalNeighbors = 6;

struct VoxelAcc
{
    double   x = 0;
    double   y = 0;
    double   z = 0;
    uint32_t n = 0;
};

uint64_t voxel_key(double x, double y, double z, double invVoxel)
{
    // 21 bits per axis around the origin: +-1048576 voxels.
    const auto     ix   = static_cast<int64_t>(std::floor(x * invVoxel)) + (1LL << 20);
    const auto     iy   = static_cast<int64_t>(std::floor(y * invVoxel)) + (1LL << 20);
    const auto     iz   = static_cast<int64_t>(std::floor(z * invVoxel)) + (1LL << 20);
    const uint64_t mask = (1ULL << 21) - 1;
    return (static_cast<uint64_t>(ix) & mask) | ((static_cast<uint64_t>(iy) & mask) << 21) |
           ((static_cast<uint64_t>(iz) & mask) << 42);
}

double quantile(std::vector<double>& v, double q)
{
    if (v.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto k = static_cast<size_t>(q * static_cast<double>(v.size() - 1) + 0.5);
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
    return v[k];
}

double cauchy_rho(double r, double c)
{
    const double u = r / c;
    return 0.5 * c * c * std::log1p(u * u);
}

double cauchy_weight(double r, double c)
{
    const double u = r / c;
    return 1.0 / (1.0 + u * u);
}

double safe_log10(double x, double floor = 1e-30) { return std::log10(std::max(x, floor)); }

// Point-to-plane Jacobian row with respect to the right tangent [x y z rx ry rz]
// of the pose, for a vehicle-frame point p and a map-frame normal n:
//   d/d(eps) n.(R (exp(eps) p) + t) = [ (R^T n)^T , (p x R^T n)^T ]
Vec6 jacobian_row(const Eigen::Matrix3d& R, const Eigen::Vector3d& p, const Eigen::Vector3d& n)
{
    const Eigen::Vector3d nb = R.transpose() * n;
    Vec6                  J;
    J.head<3>() = nb;
    J.tail<3>() = p.cross(nb);
    return J;
}

void rotation_and_translation(
    const mrpt::poses::CPose3D& pose, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    R = pose.getRotationMatrix().asEigen();
    t = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
}

mrpt::poses::CPose3D compose_right(const mrpt::poses::CPose3D& pose, const Vec6& eps)
{
    mrpt::math::CVectorFixedDouble<6> e;
    for (int i = 0; i < 6; i++)
    {
        e[i] = eps[i];
    }
    return pose + mrpt::poses::Lie::SE<3>::exp(e);
}

Vec6 log_of(const mrpt::poses::CPose3D& pose)
{
    const auto l = mrpt::poses::Lie::SE<3>::log(pose);
    Vec6       v;
    for (int i = 0; i < 6; i++)
    {
        v[i] = l[i];
    }
    return v;
}

}  // namespace

CovFeatureEvaluator::CovFeatureEvaluator(const CovFeatureParameters& p) : params_(p)
{
    ASSERT_EQUAL_(params_.probe_scales.size(), CovFeatureParameters::kNumProbeScales);
    ASSERT_GT_(params_.map_voxel, 0.0);
    ASSERT_GT_(params_.scan_voxel, 0.0);
    ASSERT_GT_(params_.cauchy_scale, 0.0);
}

size_t CovFeatureEvaluator::referenceSize() const { return ref_ ? ref_->size() : 0; }

void CovFeatureEvaluator::setReference(
    const std::vector<std::pair<const mrpt::maps::CPointsMap*, mrpt::poses::CPose3D>>& clouds)
{
    const double                           invVoxel = 1.0 / params_.map_voxel;
    std::unordered_map<uint64_t, VoxelAcc> grid;

    for (const auto& [cloud, pose] : clouds)
    {
        if (cloud == nullptr)
        {
            continue;
        }
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        rotation_and_translation(pose, R, t);
        const auto& xs = cloud->getPointsBufferRef_x();
        const auto& ys = cloud->getPointsBufferRef_y();
        const auto& zs = cloud->getPointsBufferRef_z();
        for (size_t i = 0; i < xs.size(); i++)
        {
            const Eigen::Vector3d q = R * Eigen::Vector3d(xs[i], ys[i], zs[i]) + t;
            if (!q.allFinite())
            {
                continue;
            }
            auto& a = grid[voxel_key(q.x(), q.y(), q.z(), invVoxel)];
            a.x += q.x();
            a.y += q.y();
            a.z += q.z();
            a.n++;
        }
    }

    ref_ = mrpt::maps::CSimplePointsMap::Create();
    ref_->reserve(grid.size());
    for (const auto& [key, a] : grid)
    {
        const double inv = 1.0 / static_cast<double>(a.n);
        ref_->insertPointFast(
            static_cast<float>(a.x * inv), static_cast<float>(a.y * inv),
            static_cast<float>(a.z * inv));
    }
    ref_->mark_as_modified();

    normals_.assign(ref_->size(), Eigen::Vector3f::Zero());
    shapes_.assign(ref_->size(), Eigen::Vector3f::Zero());
    normalState_.assign(ref_->size(), 0);
}

void CovFeatureEvaluator::setScan(const mrpt::maps::CPointsMap& scan)
{
    const double                           invVoxel = 1.0 / params_.scan_voxel;
    std::unordered_map<uint64_t, VoxelAcc> grid;

    const auto& xs = scan.getPointsBufferRef_x();
    const auto& ys = scan.getPointsBufferRef_y();
    const auto& zs = scan.getPointsBufferRef_z();
    for (size_t i = 0; i < xs.size(); i++)
    {
        if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]) || !std::isfinite(zs[i]))
        {
            continue;
        }
        auto& a = grid[voxel_key(xs[i], ys[i], zs[i], invVoxel)];
        a.x += xs[i];
        a.y += ys[i];
        a.z += zs[i];
        a.n++;
    }

    // Deterministic order before shuffling: hash-map iteration order is not.
    std::vector<std::pair<uint64_t, Eigen::Vector3d>> pts;
    pts.reserve(grid.size());
    for (const auto& [key, a] : grid)
    {
        const double inv = 1.0 / static_cast<double>(a.n);
        pts.emplace_back(key, Eigen::Vector3d(a.x * inv, a.y * inv, a.z * inv));
    }
    std::sort(
        pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    mrpt::random::CRandomGenerator rng;
    rng.randomize(static_cast<uint32_t>(params_.seed));
    for (size_t i = pts.size(); i > 1; i--)
    {
        const size_t j = rng.drawUniform32bit() % i;
        std::swap(pts[i - 1], pts[j]);
    }

    const size_t n = std::min(pts.size(), params_.max_scan_points);
    scan_.clear();
    scan_.reserve(n);
    for (size_t i = 0; i < n; i++)
    {
        scan_.push_back(pts[i].second);
    }
}

bool CovFeatureEvaluator::mapNormal(size_t idx)
{
    if (normalState_[idx] != 0)
    {
        return normalState_[idx] == 1;
    }
    normalState_[idx] = 2;

    float x = 0;
    float y = 0;
    float z = 0;
    ref_->getPointFast(idx, x, y, z);

    std::vector<size_t> idxs;
    std::vector<float>  d2;
    ref_->kdTreeNClosestPoint3DIdx(x, y, z, params_.normal_k, idxs, d2);
    if (idxs.size() < kMinNormalNeighbors)
    {
        return false;
    }
    const double maxD2 = params_.normal_max_dist * params_.normal_max_dist;
    for (const float dd : d2)
    {
        if (dd > maxD2)
        {
            return false;
        }
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto k : idxs)
    {
        float px = 0;
        float py = 0;
        float pz = 0;
        ref_->getPointFast(k, px, py, pz);
        mean += Eigen::Vector3d(px, py, pz);
    }
    mean /= static_cast<double>(idxs.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto k : idxs)
    {
        float px = 0;
        float py = 0;
        float pz = 0;
        ref_->getPointFast(k, px, py, pz);
        const Eigen::Vector3d dlt = Eigen::Vector3d(px, py, pz) - mean;
        cov += dlt * dlt.transpose();
    }
    cov /= static_cast<double>(idxs.size());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    const Eigen::Vector3d                                lam = es.eigenvalues();  // ascending
    if (!(lam(2) > 0))
    {
        return false;
    }
    normals_[idx] = es.eigenvectors().col(0).cast<float>();
    shapes_[idx]  = Eigen::Vector3f(
         static_cast<float>((lam(1) - lam(0)) / lam(2)),
         static_cast<float>((lam(2) - lam(1)) / lam(2)), static_cast<float>(lam(0) / lam(2)));
    normalState_[idx] = 1;
    return true;
}

CovFeatureEvaluator::Assoc CovFeatureEvaluator::associate(const Eigen::Vector3d& q)
{
    Assoc a;
    float mx = 0;
    float my = 0;
    float mz = 0;
    float d2 = 0;
    a.mapIdx = ref_->kdTreeClosestPoint3D(
        static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()), mx, my, mz,
        d2);
    a.d = std::sqrt(static_cast<double>(d2));
    if (a.d > params_.max_assoc_dist)
    {
        return a;
    }
    if (!mapNormal(a.mapIdx))
    {
        return a;
    }
    a.n  = normals_[a.mapIdx].cast<double>();
    a.r  = a.n.dot(q - Eigen::Vector3d(mx, my, mz));
    a.ok = true;
    return a;
}

double CovFeatureEvaluator::cost(const mrpt::poses::CPose3D& pose, size_t count)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    rotation_and_translation(pose, R, t);

    const double c        = params_.cauchy_scale;
    const double rhoUnass = cauchy_rho(params_.max_assoc_dist, c);
    double       sum      = 0;
    for (size_t i = 0; i < count; i++)
    {
        const Assoc a = associate(R * scan_[i] + t);
        sum += a.ok ? cauchy_rho(a.r, c) : rhoUnass;
    }
    return sum;
}

const std::vector<std::string>& CovFeatureEvaluator::featureNames()
{
    static const std::vector<std::string> names = []()
    {
        std::vector<std::string>  n;
        const char*               ax[6] = {"x", "y", "z", "rx", "ry", "rz"};
        const std::vector<double> qs    = {10, 25, 50, 75, 90, 99};

        // A: problem structure
        for (int i = 0; i < 6; i++)
        {
            n.push_back("A_logeig" + std::to_string(i));
        }
        n.push_back("A_logcond");
        n.push_back("A_effrank");
        n.push_back("A_logsigma2");
        for (const auto* a : ax)
        {
            n.push_back(std::string("A_logsd_") + a);
        }
        // B: residuals
        for (const auto q : qs)
        {
            n.push_back("B_rpl_p" + std::to_string(static_cast<int>(q)));
        }
        for (const auto q : qs)
        {
            n.push_back("B_dnn_p" + std::to_string(static_cast<int>(q)));
        }
        n.push_back("B_frac_assoc");
        n.push_back("B_frac_inlier");
        n.push_back("B_log_npts");
        n.push_back("B_log_nassoc");
        n.push_back("B_mean_w");
        // C: finite-scale curvature along whitened axes
        for (const char* kind : {"sym", "asym"})
        {
            for (int i = 0; i < 6; i++)
            {
                for (size_t k = 0; k < CovFeatureParameters::kNumProbeScales; k++)
                {
                    n.push_back(
                        std::string("C_") + kind + "_a" + std::to_string(i) + "_k" +
                        std::to_string(k));
                }
            }
        }
        for (int i = 0; i < 6; i++)
        {
            n.push_back("C_step_a" + std::to_string(i));
        }
        // D: scene geometry
        for (int i = 0; i < 3; i++)
        {
            n.push_back("D_nscat" + std::to_string(i));
        }
        n.push_back("D_planarity");
        n.push_back("D_linearity");
        n.push_back("D_scattering");
        n.push_back("D_range_p10");
        n.push_back("D_range_p50");
        n.push_back("D_range_p90");
        n.push_back("D_range_max");
        n.push_back("D_zextent");
        n.push_back("D_frac_ground");
        // E: prior and initialization
        n.push_back("E_prior_in_solver");
        for (int i = 0; i < 6; i++)
        {
            n.push_back("E_logP" + std::to_string(i));
        }
        n.push_back("E_logdetP");
        n.push_back("E_maha_init");
        n.push_back("E_maha_a");
        n.push_back("E_move_t");
        n.push_back("E_move_r");
        n.push_back("E_iters");
        n.push_back("E_converged");
        n.push_back("E_quality");
        return n;
    }();
    return names;
}

CovFeatureResult CovFeatureEvaluator::evaluate(
    const mrpt::poses::CPose3D& solution, const CovFeatureContext& ctx)
{
    ASSERTMSG_(ref_ && !ref_->empty(), "setReference() first, with a non-empty cloud");
    ASSERTMSG_(!scan_.empty(), "setScan() first, with a non-empty scan");

    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    CovFeatureResult out;
    const double     c      = params_.cauchy_scale;
    const size_t     N      = scan_.size();
    const size_t     nProbe = std::min(params_.probe_points, N);

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    rotation_and_translation(solution, R, t);

    // ---- One association pass at the solution: H, g, residuals, scene ----
    Mat6   H      = Mat6::Zero();
    Mat6   Hp     = Mat6::Zero();  // probe prefix only
    Vec6   g      = Vec6::Zero();
    double sumWr2 = 0;
    double sumW   = 0;
    double cost0p = 0;
    size_t nAssoc = 0;
    size_t nInl   = 0;

    std::vector<double> absR;
    std::vector<double> dnn;
    std::vector<double> ranges;
    std::vector<double> zs;
    absR.reserve(N);
    dnn.reserve(N);
    ranges.reserve(N);
    zs.reserve(N);

    Eigen::Matrix3d nScatter = Eigen::Matrix3d::Zero();
    Eigen::Vector3d shapeSum = Eigen::Vector3d::Zero();
    size_t          nGround  = 0;

    const double rhoUnass = cauchy_rho(params_.max_assoc_dist, c);
    for (size_t i = 0; i < N; i++)
    {
        const auto& p = scan_[i];
        ranges.push_back(p.norm());
        zs.push_back(p.z());

        const Assoc a = associate(R * p + t);
        dnn.push_back(std::min(a.d, params_.max_assoc_dist));
        if (!a.ok)
        {
            if (i < nProbe)
            {
                cost0p += rhoUnass;
            }
            continue;
        }
        nAssoc++;
        absR.push_back(std::abs(a.r));
        if (std::abs(a.r) < c)
        {
            nInl++;
        }
        const double w  = cauchy_weight(a.r, c);
        const Vec6   J  = jacobian_row(R, p, a.n);
        const Mat6   JJ = w * J * J.transpose();
        H += JJ;
        g += w * a.r * J;
        sumWr2 += w * a.r * a.r;
        sumW += w;
        if (i < nProbe)
        {
            Hp += JJ;
            cost0p += cauchy_rho(a.r, c);
        }
        nScatter += a.n * a.n.transpose();
        shapeSum += shapes_[a.mapIdx].cast<double>();
        if (std::abs(a.n.z()) > 0.9)
        {
            nGround++;
        }
    }

    std::vector<double> f;
    f.reserve(featureNames().size());
    const std::vector<double> qs = {0.10, 0.25, 0.50, 0.75, 0.90, 0.99};

    out.ok = nAssoc >= kMinAssociations;

    // ---- Analytic covariance of the common cost ----
    Vec6 lamC = Vec6::Constant(kNaN);
    if (out.ok)
    {
        const Eigen::SelfAdjointEigenSolver<Mat6> es(H);
        const Vec6                                lam = es.eigenvalues();  // ascending
        if (!(lam(5) > 0))
        {
            out.ok = false;
        }
        else
        {
            lamC            = lam.cwiseMax(lam(5) / params_.max_condition);
            const Mat6 Hinv = es.eigenvectors() * lamC.cwiseInverse().asDiagonal() *
                              es.eigenvectors().transpose();
            out.sigma2  = sumWr2 / std::max(1.0, sumW - 6.0);
            out.H       = H;
            out.g       = g;
            out.Sigma_a = out.sigma2 * Hinv;
            out.Sigma_a = 0.5 * (out.Sigma_a + out.Sigma_a.transpose()).eval();
            const Eigen::LLT<Mat6> llt(out.Sigma_a);
            if (llt.info() != Eigen::Success || !(out.sigma2 > 0))
            {
                out.ok = false;
            }
            else
            {
                out.L_a = llt.matrixL();
            }
        }
    }

    // ---- A: problem structure ----
    if (out.ok)
    {
        for (int i = 0; i < 6; i++)
        {
            f.push_back(safe_log10(lamC(i)));
        }
        f.push_back(safe_log10(lamC(5) / lamC(0)));
        int eff = 0;
        for (int i = 0; i < 6; i++)
        {
            if (lamC(i) > 1e-6 * lamC(5))
            {
                eff++;
            }
        }
        f.push_back(eff);
        f.push_back(safe_log10(out.sigma2));
        for (int i = 0; i < 6; i++)
        {
            f.push_back(safe_log10(std::sqrt(out.Sigma_a(i, i))));
        }
    }
    else
    {
        f.insert(f.end(), 15, kNaN);
    }

    // ---- B: residuals ----
    for (const auto q : qs)
    {
        f.push_back(absR.empty() ? kNaN : quantile(absR, q));
    }
    for (const auto q : qs)
    {
        f.push_back(quantile(dnn, q));
    }
    f.push_back(static_cast<double>(nAssoc) / static_cast<double>(N));
    f.push_back(static_cast<double>(nInl) / static_cast<double>(N));
    f.push_back(std::log10(static_cast<double>(N)));
    f.push_back(std::log10(1.0 + static_cast<double>(nAssoc)));
    f.push_back(nAssoc > 0 ? sumW / static_cast<double>(nAssoc) : kNaN);

    // ---- C: finite-scale curvature along the whitened axes ----
    if (out.ok)
    {
        std::vector<double> sym;
        std::vector<double> asym;
        for (int i = 0; i < 6; i++)
        {
            const Vec6   axis  = out.L_a.col(i);
            const double quad1 = 0.5 * axis.dot(Hp * axis);  // per unit k^2
            for (const double k : params_.probe_scales)
            {
                const double dPlus  = cost(compose_right(solution, k * axis), nProbe) - cost0p;
                const double dMinus = cost(compose_right(solution, -k * axis), nProbe) - cost0p;
                const double pred   = std::max(k * k * quad1, 1e-300);
                const double s      = 0.5 * (dPlus + dMinus);
                sym.push_back(std::log10(std::max(s, 1e-3 * pred) / pred));
                asym.push_back(std::log10(1.0 + std::abs(dPlus - dMinus) / (2.0 * pred)));
            }
        }
        f.insert(f.end(), sym.begin(), sym.end());
        f.insert(f.end(), asym.begin(), asym.end());

        // Gauss-Newton step to the common cost's optimum, in analytic sigmas.
        const Vec6 step = -(out.Sigma_a / out.sigma2) * g;
        const Vec6 ws   = out.L_a.triangularView<Eigen::Lower>().solve(step);
        for (int i = 0; i < 6; i++)
        {
            f.push_back(std::log10(1.0 + std::abs(ws(i))));
        }
    }
    else
    {
        f.insert(f.end(), 6 * CovFeatureParameters::kNumProbeScales * 2 + 6, kNaN);
    }

    // ---- D: scene geometry ----
    if (nAssoc > 0)
    {
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
            nScatter / static_cast<double>(nAssoc));
        const Eigen::Vector3d ev = es.eigenvalues();
        const double          s  = std::max(ev.sum(), 1e-30);
        for (int i = 0; i < 3; i++)
        {
            f.push_back(ev(i) / s);
        }
        for (int i = 0; i < 3; i++)
        {
            f.push_back(shapeSum(i) / static_cast<double>(nAssoc));
        }
    }
    else
    {
        f.insert(f.end(), 6, kNaN);
    }
    {
        std::vector<double> rr = ranges;
        f.push_back(safe_log10(quantile(rr, 0.10), 1e-3));
        f.push_back(safe_log10(quantile(rr, 0.50), 1e-3));
        f.push_back(safe_log10(quantile(rr, 0.90), 1e-3));
        f.push_back(safe_log10(*std::max_element(ranges.begin(), ranges.end()), 1e-3));
        std::vector<double> zz  = zs;
        const double        z5  = quantile(zz, 0.05);
        const double        z95 = quantile(zz, 0.95);
        f.push_back(std::log10(0.01 + std::max(0.0, z95 - z5)));
    }
    f.push_back(nAssoc > 0 ? static_cast<double>(nGround) / static_cast<double>(nAssoc) : kNaN);

    // ---- E: prior and initialization ----
    f.push_back(ctx.priorInSolver ? 1.0 : 0.0);
    const Vec6 move = log_of(solution - ctx.initialGuess);  // initialGuess^-1 * solution
    if (ctx.priorCovariance && out.ok)
    {
        const Mat6 Li = out.L_a.triangularView<Eigen::Lower>().solve(Mat6::Identity());
        const Mat6 P  = Li * (*ctx.priorCovariance) * Li.transpose();
        for (int i = 0; i < 6; i++)
        {
            f.push_back(safe_log10(P(i, i)));
        }
        f.push_back(safe_log10(std::max(P.determinant(), 1e-300)) / 6.0);
    }
    else
    {
        f.insert(f.end(), 7, kNaN);
    }
    if (ctx.priorCovariance)
    {
        const Eigen::LDLT<Mat6> ldlt(*ctx.priorCovariance);
        const double            m2 = move.dot(ldlt.solve(move));
        f.push_back(std::log10(1e-3 + std::sqrt(std::max(0.0, m2))));
    }
    else
    {
        f.push_back(kNaN);
    }
    if (out.ok)
    {
        const Vec6 wm = out.L_a.triangularView<Eigen::Lower>().solve(move);
        f.push_back(std::log10(1e-3 + wm.norm()));
    }
    else
    {
        f.push_back(kNaN);
    }
    f.push_back(std::log10(1e-4 + move.head<3>().norm()));
    f.push_back(std::log10(1e-5 + move.tail<3>().norm()));
    f.push_back(std::log10(1.0 + ctx.iterations));
    f.push_back(ctx.converged ? 1.0 : 0.0);
    f.push_back(ctx.quality);

    ASSERT_EQUAL_(f.size(), featureNames().size());
    out.features.assign(f.begin(), f.end());
    return out;
}
