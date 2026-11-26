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
 * @file   test-mp2p_estimate_up_from_normals.cpp
 * @brief  Unit test for estimate_up_from_normals()
 * @author Jose Luis Blanco Claraco
 * @date   Oct 8, 2026
 */

#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <tuple>

using namespace mp2p_icp_filters;

namespace
{
struct RoomOptions
{
    bool   floor       = true;
    bool   ceiling     = true;
    bool   wallsAlongX = true;  // walls at y=const (normal along y)
    bool   wallsAlongY = true;  // walls at x=const (normal along x)
    double noiseStd    = 0.003;
};

// A 6 x 4 x 2.5 m room sampled every 5 cm, transformed by `T`.
mrpt::maps::CSimplePointsMap makeRoom(const mrpt::poses::CPose3D& T, const RoomOptions& o = {})
{
    constexpr double sx   = 6.0;
    constexpr double sy   = 4.0;
    constexpr double sz   = 2.5;
    constexpr double step = 0.05;

    std::mt19937                     rng(1234);
    std::normal_distribution<double> noise(0.0, o.noiseStd);

    mrpt::maps::CSimplePointsMap pc;
    const auto                   add = [&](double x, double y, double z)
    {
        const auto p =
            T.composePoint(mrpt::math::TPoint3D(x + noise(rng), y + noise(rng), z + noise(rng)));
        pc.insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    };

    const auto nx = static_cast<int>(sx / step);
    const auto ny = static_cast<int>(sy / step);
    const auto nz = static_cast<int>(sz / step);

    for (int i = 0; i <= nx; i++)
    {
        for (int j = 0; j <= ny; j++)
        {
            if (o.floor)
            {
                add(i * step, j * step, 0.0);
            }
            if (o.ceiling)
            {
                add(i * step, j * step, sz);
            }
        }
    }
    for (int k = 1; k < nz; k++)
    {
        for (int i = 0; i <= nx; i++)
        {
            if (o.wallsAlongX)
            {
                add(i * step, 0.0, k * step);
                add(i * step, sy, k * step);
            }
        }
        for (int j = 0; j <= ny; j++)
        {
            if (o.wallsAlongY)
            {
                add(0.0, j * step, k * step);
                add(sx, j * step, k * step);
            }
        }
    }
    return pc;
}

// Rotation of `angle_deg` about a horizontal axis at azimuth `azimuth_deg`.
mrpt::poses::CPose3D tiltPose(double angle_deg, double azimuth_deg)
{
    const double                      a = mrpt::DEG2RAD(angle_deg);
    const double                      b = mrpt::DEG2RAD(azimuth_deg);
    mrpt::math::CVectorFixedDouble<3> w;
    w[0] = a * std::cos(b);
    w[1] = a * std::sin(b);
    w[2] = 0;
    return mrpt::poses::CPose3D::FromRotationAndTranslation(
        mrpt::poses::Lie::SO<3>::exp(w), mrpt::math::TVector3D(0, 0, 0));
}

double rotationAngleDeg(const mrpt::poses::CPose3D& p)
{
    return mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(p.getRotationMatrix()).norm());
}

void test_recover_tilt()
{
    for (const double angle : {0.0, 0.3, 1.0, 3.0})
    {
        for (const double azimuth : {0.0, 35.0, 210.0})
        {
            const auto T  = tiltPose(angle, azimuth);
            const auto pc = makeRoom(T);

            const auto normals = estimate_planar_normals(pc);
            const auto est     = estimate_up_from_normals(normals);
            const auto R       = rotation_aligning_up_to_z(est.up);

            // Exact recovery of a horizontal-axis tilt, including its heading:
            const double err = rotationAngleDeg(R + T);
            std::cout << "  tilt " << angle << " deg @ " << azimuth << " deg: estimated "
                      << est.tilt_deg() << " deg, walls=" << est.wall_count
                      << " flats=" << est.flat_count << ", error " << err << " deg\n";
            ASSERT_LT_(err, 0.05);
            ASSERT_NEAR_(est.tilt_deg(), angle, 0.05);
            ASSERT_(!est.single_wall_direction);

            // Its rotation axis is horizontal:
            const auto w = mrpt::poses::Lie::SO<3>::log(R.getRotationMatrix());
            ASSERT_LT_(std::abs(w[2]), 1e-9);
        }
    }
}

void test_single_wall_direction_rejected()
{
    RoomOptions o;
    o.floor       = false;
    o.ceiling     = false;
    o.wallsAlongY = false;

    const auto pc      = makeRoom(tiltPose(1.0, 20.0), o);
    const auto normals = estimate_planar_normals(pc);

    bool thrown = false;
    try
    {
        [[maybe_unused]] const auto est = estimate_up_from_normals(normals);
    }
    catch (const std::exception&)
    {
        thrown = true;
    }
    ASSERTMSG_(thrown, "A single wall direction without floors must be rejected");
}

void test_single_wall_direction_with_floor()
{
    RoomOptions o;
    o.wallsAlongY = false;

    const auto T       = tiltPose(1.0, 20.0);
    const auto pc      = makeRoom(T, o);
    const auto normals = estimate_planar_normals(pc);
    const auto est     = estimate_up_from_normals(normals);

    ASSERT_(est.single_wall_direction);
    ASSERT_LT_(rotationAngleDeg(rotation_aligning_up_to_z(est.up) + T), 0.05);
}

void test_deterministic()
{
    const auto pc = makeRoom(tiltPose(0.7, 120.0));

    const auto n1 = estimate_planar_normals(pc);
    const auto n2 = estimate_planar_normals(pc);
    ASSERT_EQUAL_(n1.size(), n2.size());
    for (size_t i = 0; i < n1.size(); i++)
    {
        ASSERT_(n1[i] == n2[i]);
    }

    const auto e1 = estimate_up_from_normals(n1);
    const auto e2 = estimate_up_from_normals(n2);
    ASSERT_(e1.up == e2.up);
    ASSERT_(e1.eigVals == e2.eigVals);
}

void test_walls_only_ignores_sloped_floor()
{
    // Walls tilted 1 deg, the floor and ceiling 3 deg more (a ramp-like floor):
    const auto T     = tiltPose(1.0, 40.0);
    const auto Tflat = tiltPose(3.0, 130.0) + T;

    RoomOptions walls;
    walls.floor   = false;
    walls.ceiling = false;
    RoomOptions flats;
    flats.wallsAlongX = false;
    flats.wallsAlongY = false;

    auto       normals      = estimate_planar_normals(makeRoom(T, walls));
    const auto flatsNormals = estimate_planar_normals(makeRoom(Tflat, flats));
    normals.insert(normals.end(), flatsNormals.begin(), flatsNormals.end());

    EstimateUpParams p;
    p.flats_weight = 0;
    // Wide thresholds would take the sloped floor for neither walls nor flats:
    const auto est = estimate_up_from_normals(normals, p);
    const auto err = rotationAngleDeg(rotation_aligning_up_to_z(est.up) + T);
    std::cout << "  walls only: error " << err << " deg (flats used: " << est.flat_count << ")\n";
    ASSERT_LT_(err, 0.05);
    ASSERT_EQUAL_(est.flat_count, 0UL);

    // With the flats, the sloped floor pulls the estimate away:
    const auto mixed = estimate_up_from_normals(normals);
    ASSERT_GT_(rotationAngleDeg(rotation_aligning_up_to_z(mixed.up) + T), 0.5);
}

void test_walls_only_single_direction_rejected()
{
    RoomOptions o;
    o.wallsAlongY = false;  // one wall direction, plus floor and ceiling

    const auto       normals = estimate_planar_normals(makeRoom(tiltPose(1.0, 20.0), o));
    EstimateUpParams p;
    p.flats_weight = 0;

    bool thrown = false;
    try
    {
        [[maybe_unused]] const auto est = estimate_up_from_normals(normals, p);
    }
    catch (const std::exception&)
    {
        thrown = true;
    }
    ASSERTMSG_(thrown, "Walls only with a single wall direction must be rejected");
}

// A unit vector at a random angle of `sigma` [rad] per axis from `n`.
mrpt::math::TVector3D perturb(const mrpt::math::TVector3D& n, double sigma, std::mt19937& rng)
{
    std::normal_distribution<double> g(0.0, sigma);
    mrpt::math::TVector3D            v(n.x + g(rng), n.y + g(rng), n.z + g(rng));
    return v * (1.0 / v.norm());
}

struct CalibrationResult
{
    double empiricalSigmaX = 0;
    double empiricalSigmaY = 0;
    double reportedSigmaX  = 0;
    double reportedSigmaY  = 0;
    double meanErrorDeg    = 0;
};

// Monte Carlo of the up estimate on synthetic normals of a room tilted by
// `T`: `clusters` groups of `perCluster` normals per population (walls along
// X, walls along Y, flats), each group sharing an angular offset of
// `sigmaShared` [rad] plus `sigmaEach` [rad] per normal.
CalibrationResult monteCarlo(
    const mrpt::poses::CPose3D& T, size_t clusters, size_t perCluster, double sigmaShared,
    double sigmaEach, bool givePoints, const EstimateUpParams& params, size_t trials)
{
    std::mt19937 rng(42);

    const std::array<mrpt::math::TVector3D, 3> bases = {
        T.rotateVector({1, 0, 0}), T.rotateVector({0, 1, 0}), T.rotateVector({0, 0, 1})};
    const auto trueUp = bases[2];

    std::vector<double> ex;
    std::vector<double> ey;
    double              repX = 0;
    double              repY = 0;
    double              err  = 0;

    for (size_t trial = 0; trial < trials; trial++)
    {
        std::vector<mrpt::math::TVector3D> normals;
        std::vector<mrpt::math::TPoint3Df> points;
        for (size_t b = 0; b < bases.size(); b++)
        {
            if (params.flats_weight == 0 && b == 2)
            {
                continue;
            }
            for (size_t c = 0; c < clusters; c++)
            {
                const auto shared = perturb(bases[b], sigmaShared, rng);
                const auto cell   = mrpt::math::TPoint3Df(
                      static_cast<float>(10.0 * c + 1.0), static_cast<float>(10.0 * b + 1.0), 1.0f);
                for (size_t i = 0; i < perCluster; i++)
                {
                    const auto n = perturb(shared, sigmaEach, rng);
                    normals.push_back((i % 2) ? n : -n);  // arbitrary signs
                    points.push_back(cell);
                }
            }
        }
        const auto est = estimate_up_from_normals(normals, params, givePoints ? &points : nullptr);
        ex.push_back(est.up.x - trueUp.x);
        ey.push_back(est.up.y - trueUp.y);
        repX += est.up_cov(0, 0);
        repY += est.up_cov(1, 1);
        err += mrpt::RAD2DEG(std::acos(mrpt::saturate_val(est.up.dot(trueUp), -1.0, 1.0)));
    }

    const auto rms = [](const std::vector<double>& v)
    {
        double m = 0;
        for (const double x : v)
        {
            m += x;
        }
        m /= static_cast<double>(v.size());
        double s = 0;
        for (const double x : v)
        {
            s += (x - m) * (x - m);
        }
        return std::sqrt(s / static_cast<double>(v.size() - 1));
    };

    const auto        n = static_cast<double>(trials);
    CalibrationResult r;
    r.empiricalSigmaX = mrpt::RAD2DEG(rms(ex));
    r.empiricalSigmaY = mrpt::RAD2DEG(rms(ey));
    r.reportedSigmaX  = mrpt::RAD2DEG(std::sqrt(repX / n));
    r.reportedSigmaY  = mrpt::RAD2DEG(std::sqrt(repY / n));
    r.meanErrorDeg    = err / n;
    return r;
}

void checkCalibrated(const CalibrationResult& r, const char* label)
{
    const double ratioX = r.reportedSigmaX / r.empiricalSigmaX;
    const double ratioY = r.reportedSigmaY / r.empiricalSigmaY;
    std::cout << "  " << label << ": sigma x reported " << r.reportedSigmaX << " vs empirical "
              << r.empiricalSigmaX << " deg (" << ratioX << "), y " << r.reportedSigmaY << " vs "
              << r.empiricalSigmaY << " (" << ratioY << ")\n";
    ASSERT_GT_(ratioX, 0.7);
    ASSERT_LT_(ratioX, 1.3);
    ASSERT_GT_(ratioY, 0.7);
    ASSERT_LT_(ratioY, 1.3);
}

void test_covariance_independent_normals()
{
    const auto T = tiltPose(1.0, 60.0);

    EstimateUpParams withFlats;
    checkCalibrated(
        monteCarlo(T, 300, 1, 0.0, mrpt::DEG2RAD(2.0), false, withFlats, 400), "walls + flats");

    EstimateUpParams wallsOnly;
    wallsOnly.flats_weight = 0;
    checkCalibrated(
        monteCarlo(T, 300, 1, 0.0, mrpt::DEG2RAD(2.0), false, wallsOnly, 400), "walls only");
}

void test_covariance_grouped_errors()
{
    // Each group of 20 normals shares a 1 deg error, as the normals of one
    // leaning plane do:
    const auto T = tiltPose(0.5, 200.0);

    EstimateUpParams p;
    p.cluster_size = 2.0;
    checkCalibrated(
        monteCarlo(T, 40, 20, mrpt::DEG2RAD(1.0), mrpt::DEG2RAD(2.0), true, p, 400),
        "grouped, with points");

    // Without the points, every normal counts as independent: optimistic.
    const auto r = monteCarlo(T, 40, 20, mrpt::DEG2RAD(1.0), mrpt::DEG2RAD(2.0), false, p, 400);
    std::cout << "  grouped, no points: sigma x reported " << r.reportedSigmaX << " vs empirical "
              << r.empiricalSigmaX << " deg\n";
    ASSERT_LT_(r.reportedSigmaX, 0.6 * r.empiricalSigmaX);
}

void test_covariance_few_groups()
{
    const auto perNormal = [](const std::vector<mrpt::math::TVector3D>& normals)
    { return estimate_up_from_normals(normals).up_cov; };

    // The room at a known place, so the cells it falls in are known:
    for (const auto& [origin, cell, groups] :
         {std::tuple<double, double, size_t>{1.0, 25.0, 1},
          std::tuple<double, double, size_t>{14.0, 16.0, 2}})
    {
        const auto T =
            mrpt::poses::CPose3D::FromTranslation(origin, 1.0, 1.0) + tiltPose(1.0, 70.0);
        std::vector<mrpt::math::TPoint3Df> points;
        const auto normals = estimate_planar_normals(makeRoom(T), {}, &points);

        EstimateUpParams p;
        p.cluster_size = cell;
        const auto est = estimate_up_from_normals(normals, p, &points);
        std::cout << "  " << est.cov_groups << " group(s): sigma x " << est.up_sigmas_deg()[0]
                  << " deg, reliable " << est.cov_reliable << "\n";
        ASSERT_EQUAL_(est.cov_groups, groups);
        ASSERT_(!est.cov_reliable);
        // Falls back to the per-normal covariance, never a null or rank-1 one:
        const auto ind = perNormal(normals);
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                ASSERT_EQUAL_(est.up_cov(i, j), ind(i, j));
            }
        }
        const double det =
            est.up_cov(0, 0) * est.up_cov(1, 1) - est.up_cov(0, 1) * est.up_cov(1, 0);
        ASSERT_GT_(det, 0);
    }

    // Enough cells: grouped, and never below the per-normal covariance
    const auto T = mrpt::poses::CPose3D::FromTranslation(1.0, 1.0, 1.0) + tiltPose(1.0, 70.0);
    std::vector<mrpt::math::TPoint3Df> points;
    const auto                         normals = estimate_planar_normals(makeRoom(T), {}, &points);
    EstimateUpParams                   p;
    p.cluster_size = 1.0;
    const auto est = estimate_up_from_normals(normals, p, &points);
    ASSERT_(est.cov_reliable);
    ASSERT_GE_(est.cov_groups, p.min_cov_groups);
    const auto ind = perNormal(normals);
    ASSERT_GE_(est.up_cov(0, 0), ind(0, 0));
    ASSERT_GE_(est.up_cov(1, 1), ind(1, 1));
}

void test_level_poses_from_up()
{
    // A walk at 1 m/s, turning, whose estimated vertical drifts from 0 to 3
    // deg in 100 s. Each 10 s window sees the room leaning by the drift then.
    const auto drift = [](double t) { return tiltPose(3.0 * t / 100.0, 30.0 + 0.5 * t); };

    std::vector<double>               times;
    std::vector<mrpt::poses::CPose3D> truth;
    std::vector<mrpt::poses::CPose3D> drifted;
    for (int k = 0; k <= 100; k++)
    {
        const double         t   = k;
        const double         yaw = mrpt::DEG2RAD(1.5 * t);
        mrpt::math::TPoint3D p(0, 0, 0);
        if (k > 0)
        {
            const auto& prev = truth.back();
            p = prev.translation() + mrpt::math::TVector3D(std::cos(yaw), std::sin(yaw), 0);
        }
        times.push_back(t);
        truth.push_back(mrpt::poses::CPose3D::FromXYZYawPitchRoll(p.x, p.y, p.z, yaw, 0, 0));

        mrpt::math::TPoint3D q(0, 0, 0);
        if (k > 0)
        {
            const auto step = truth[k].translation() - truth[k - 1].translation();
            q               = drifted.back().translation() + drift(t - 0.5).rotateVector(step);
        }
        drifted.push_back(mrpt::poses::CPose3D::FromRotationAndTranslation(
            drift(t).getRotationMatrix() * truth[k].getRotationMatrix(), q));
    }

    std::vector<UpSample> samples;
    for (int w = 0; w < 10; w++)
    {
        const double tc  = 10.0 * w + 5.0;
        const auto   est = estimate_up_from_normals(estimate_planar_normals(makeRoom(drift(tc))));
        samples.push_back({tc, est.up});
    }

    const auto leveled = level_poses_from_up(times, drifted, samples);
    ASSERT_EQUAL_(leveled.size(), truth.size());

    double maxAttErrInside = 0;
    double maxAttErr       = 0;
    double maxZ            = 0;
    for (size_t k = 0; k < truth.size(); k++)
    {
        const auto   up  = leveled[k].rotateVector({0, 0, 1});
        const auto   tu  = truth[k].rotateVector({0, 0, 1});
        const double err = mrpt::RAD2DEG(std::acos(mrpt::saturate_val(up.dot(tu), -1.0, 1.0)));
        maxAttErr        = std::max(maxAttErr, err);
        if (times[k] >= 5.0 && times[k] <= 95.0)
        {
            maxAttErrInside = std::max(maxAttErrInside, err);
        }
        maxZ = std::max(maxZ, std::abs(leveled[k].z()));
        if (k > 0)
        {
            // Continuous: each step keeps its length
            const double a = (leveled[k].translation() - leveled[k - 1].translation()).norm();
            const double b = (drifted[k].translation() - drifted[k - 1].translation()).norm();
            ASSERT_NEAR_(a, b, 1e-9);
        }
    }
    double driftedZ = 0;
    for (const auto& d : drifted)
    {
        driftedZ = std::max(driftedZ, std::abs(d.z()));
    }
    std::cout << "  leveled: attitude error max " << maxAttErr << " deg (" << maxAttErrInside
              << " between window centres), |z| max " << maxZ << " m (drifted: " << driftedZ
              << " m)\n";
    ASSERT_LT_(maxAttErrInside, 0.1);
    ASSERT_LT_(maxAttErr, 0.2);
    ASSERT_LT_(maxZ, 0.05);
    ASSERT_GT_(driftedZ, 1.0);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_recover_tilt();
        std::cout << "test_recover_tilt: Success" << std::endl;

        test_single_wall_direction_rejected();
        std::cout << "test_single_wall_direction_rejected: Success" << std::endl;

        test_single_wall_direction_with_floor();
        std::cout << "test_single_wall_direction_with_floor: Success" << std::endl;

        test_deterministic();
        std::cout << "test_deterministic: Success" << std::endl;

        test_walls_only_ignores_sloped_floor();
        std::cout << "test_walls_only_ignores_sloped_floor: Success" << std::endl;

        test_walls_only_single_direction_rejected();
        std::cout << "test_walls_only_single_direction_rejected: Success" << std::endl;

        test_covariance_independent_normals();
        std::cout << "test_covariance_independent_normals: Success" << std::endl;

        test_covariance_grouped_errors();
        std::cout << "test_covariance_grouped_errors: Success" << std::endl;

        test_covariance_few_groups();
        std::cout << "test_covariance_few_groups: Success" << std::endl;

        test_level_poses_from_up();
        std::cout << "test_level_poses_from_up: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}
