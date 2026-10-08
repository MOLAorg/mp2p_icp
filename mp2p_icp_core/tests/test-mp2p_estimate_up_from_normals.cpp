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

#include <cmath>
#include <iostream>
#include <random>

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

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}
