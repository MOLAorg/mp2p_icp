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
 * @file   test-mp2p_FilterBeamCorrection.cpp
 * @brief  Unit test for FilterBeamCorrection
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBeamCorrection.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

using namespace mp2p_icp_filters;

namespace
{
constexpr int kRings = 8;

/** True corrections of a test sensor (6 parameters, ring 7 without a row). */
struct TrueBeam
{
    double de = 0, da = 0, dr = 0, s = 0, dv = 0, dh = 0;  // NOLINT
};

TrueBeam trueBeam(int k)
{
    TrueBeam b;
    b.de = mrpt::DEG2RAD(0.05 * (k - 3.5));
    b.da = mrpt::DEG2RAD(0.03 * ((k % 3) - 1));
    b.dr = 0.004 * (k - 4);
    b.s  = 1e-4 * (k % 2 == 0 ? 1 : -1);
    b.dv = 0.002 * ((k % 4) - 1.5);
    b.dh = -0.001 * k;
    return b;
}

/// The table as YAML text (a flow sequence, one map per ring).
std::string beamsText(bool sixParams)
{
    std::string t = "[";
    for (int k = 0; k < kRings - 1; k++)
    {
        const auto b = trueBeam(k);
        t += mrpt::format(
            "%s{ring: %d, d_elevation_deg: %.15g, d_azimuth_deg: %.15g, d_range_m: %.15g",
            k ? ", " : "", k, mrpt::RAD2DEG(b.de), mrpt::RAD2DEG(b.da), b.dr);
        if (sixParams)
        {
            t += mrpt::format(
                ", d_range_scale: %.15g, d_vertical_m: %.15g, d_horizontal_m: %.15g", b.s, b.dv,
                b.dh);
        }
        t += "}";
    }
    return t + "]";
}

std::string poseText(const mrpt::poses::CPose3D& p)
{
    return mrpt::format(
        "[%.15g, %.15g, %.15g, %.15g, %.15g, %.15g]", p.x(), p.y(), p.z(), p.yaw(), p.pitch(),
        p.roll());
}

struct Cloud
{
    mrpt::maps::CGenericPointsMap::Ptr pc;
    std::vector<mrpt::math::TPoint3D>  expected;  // in the cloud frame
};

/** Reported points of rays at nominal angles, in the cloud frame, and the
 * points the corrections should give. */
Cloud makeCloud(
    const mrpt::poses::CPose3D& T_cloud_sensor, bool sixParams, double beamOrigin,
    const std::vector<double>& azOffsets)
{
    Cloud                                  out;
    std::mt19937                           rng(3);
    std::uniform_real_distribution<double> uAz(-M_PI, M_PI);
    std::uniform_real_distribution<double> uR(1.0, 40.0);
    out.pc = mrpt::maps::CGenericPointsMap::Create();
    ASSERT_(out.pc->registerField_uint16("ring"));
    for (int i = 0; i < 400; i++)
    {
        const int    k     = i % kRings;
        const double elev  = mrpt::DEG2RAD(-15.0 + 4.0 * k);
        const double theta = uAz(rng);
        const double range = uR(rng);
        const double aOff  = azOffsets.empty() ? 0.0 : azOffsets[static_cast<size_t>(k)];
        const double az    = theta + aOff;
        const mrpt::math::TPoint3D o(beamOrigin * std::cos(theta), beamOrigin * std::sin(theta), 0);
        const mrpt::math::TPoint3D rep(
            o.x + range * std::cos(elev) * std::cos(az),
            o.y + range * std::cos(elev) * std::sin(az), range * std::sin(elev));

        mrpt::math::TPoint3D truth = rep;
        if (k < kRings - 1)
        {
            const auto   b   = trueBeam(k);
            const double e2  = elev + b.de;
            const double a2  = az + b.da;
            const double s   = sixParams ? b.s : 0.0;
            const double dv  = sixParams ? b.dv : 0.0;
            const double dh  = sixParams ? b.dh : 0.0;
            const double rho = (1 + s) * range + b.dr;
            truth            = mrpt::math::TPoint3D(
                           o.x - dh * std::sin(a2) + rho * std::cos(e2) * std::cos(a2),
                           o.y + dh * std::cos(a2) + rho * std::cos(e2) * std::sin(a2),
                           dv + rho * std::sin(e2));
        }
        const auto pC = T_cloud_sensor.composePoint(rep);
        out.pc->insertPoint(
            static_cast<float>(pC.x), static_cast<float>(pC.y), static_cast<float>(pC.z));
        out.pc->insertPointField_uint16("ring", static_cast<uint16_t>(k));
        out.expected.push_back(T_cloud_sensor.composePoint(truth));
    }
    return out;
}

void checkCorrected(const Cloud& c, float tol)
{
    const auto& xs = c.pc->getPointsBufferRef_x();
    const auto& ys = c.pc->getPointsBufferRef_y();
    const auto& zs = c.pc->getPointsBufferRef_z();
    ASSERT_EQUAL_(xs.size(), c.expected.size());
    for (size_t i = 0; i < xs.size(); i++)
    {
        const auto& e = c.expected[i];
        ASSERT_NEAR_(xs[i], static_cast<float>(e.x), tol);
        ASSERT_NEAR_(ys[i], static_cast<float>(e.y), tol);
        ASSERT_NEAR_(zs[i], static_cast<float>(e.z), tol);
    }
}

void runFilter(Cloud& c, const std::string& paramsText)
{
    mp2p_icp::metric_map_t map;
    map.layers["raw"] = c.pc;
    FilterBeamCorrection f;
    f.initialize(mrpt::containers::yaml::FromText(paramsText));
    f.filter(map);
}

void test_three_params_single_origin()
{
    const mrpt::poses::CPose3D T(0.5, -0.2, 1.3, 0.3, -0.05, 0.02);
    Cloud                      c = makeCloud(T, false, 0.0, {});
    runFilter(
        c, "pointcloud_layer: raw\nsensor_pose: " + poseText(T) + "\nbeams: " + beamsText(false));
    checkCorrected(c, 2e-4f);
}

void test_six_params_beam_origin()
{
    const mrpt::poses::CPose3D T(0, 0, 0.4, -1.2, 0, 0);
    std::vector<double>        azOff;
    for (int k = 0; k < kRings; k++)
    {
        azOff.push_back(mrpt::DEG2RAD(k % 2 == 0 ? 3.0 : -1.0));
    }
    Cloud       c    = makeCloud(T, true, 0.0158, azOff);
    std::string offs = "[";
    for (size_t k = 0; k < azOff.size(); k++)
    {
        offs += mrpt::format("%s%.15g", k ? ", " : "", mrpt::RAD2DEG(azOff[k]));
    }
    runFilter(
        c, "pointcloud_layer: raw\nsensor_pose: " + poseText(T) + "\nbeams: " + beamsText(true) +
               "\nbeam_origin_m: 0.0158\nbeam_azimuth_offsets_deg: " + offs + "]");
    checkCorrected(c, 2e-4f);
}

void test_zero_table_is_identity()
{
    Cloud c = makeCloud(mrpt::poses::CPose3D(), false, 0.0, {});
    // Expected: the reported points themselves.
    for (size_t i = 0; i < c.expected.size(); i++)
    {
        c.expected[i] = mrpt::math::TPoint3D(
            c.pc->getPointsBufferRef_x()[i], c.pc->getPointsBufferRef_y()[i],
            c.pc->getPointsBufferRef_z()[i]);
    }
    runFilter(c, "pointcloud_layer: raw\nbeams: [{ring: 0, d_elevation_deg: 0, d_range_m: 0}]");
    checkCorrected(c, 1e-5f);
}

void test_calibration_file()
{
    const auto T = mrpt::poses::CPose3D(0.1, 0, 0.3, 0, 0, 0);
    Cloud      c = makeCloud(T, false, 0.0, {});

    // The calibrator's output layout: sensors.<name>.intrinsics.beams.
    const std::string file = mrpt::system::getTempFileName() + "_calibration.yaml";
    {
        std::ofstream f(file);
        f << "format: mola_sensor_calib/result/1\nsensors:\n"
             "  imu0:\n    intrinsics: {gyro_scale: [1, 1, 1]}\n"
             "  lidar0:\n    intrinsics:\n      beam_model: 3param\n      beams:\n";
        for (int k = 0; k < kRings - 1; k++)
        {
            const auto b = trueBeam(k);
            f << mrpt::format(
                "        - {ring: %d, d_elevation_deg: %.12g, d_azimuth_deg: %.12g, "
                "d_range_m: %.12g, sigma: [0.001, 0.001, 0.0005]}\n",
                k, mrpt::RAD2DEG(b.de), mrpt::RAD2DEG(b.da), b.dr);
        }
    }
    runFilter(
        c, "pointcloud_layer: raw\nsensor_pose: " + poseText(T) + "\nbeams_file: '" + file + "'");
    checkCorrected(c, 2e-4f);
    mrpt::system::deleteFile(file);
}

void test_missing_ring_field_throws()
{
    mp2p_icp::metric_map_t map;
    auto                   pc = mrpt::maps::CGenericPointsMap::Create();
    pc->insertPoint(1.0f, 2.0f, 3.0f);
    map.layers["raw"] = pc;
    FilterBeamCorrection f;
    f.initialize(
        mrpt::containers::yaml::FromText("pointcloud_layer: raw\nbeams: " + beamsText(false)));
    bool thrown = false;
    try
    {
        f.filter(map);
    }
    catch (const std::exception&)
    {
        thrown = true;
    }
    ASSERT_(thrown);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_three_params_single_origin();
        std::cout << "test_three_params_single_origin: Success" << std::endl;

        test_six_params_beam_origin();
        std::cout << "test_six_params_beam_origin: Success" << std::endl;

        test_zero_table_is_identity();
        std::cout << "test_zero_table_is_identity: Success" << std::endl;

        test_calibration_file();
        std::cout << "test_calibration_file: Success" << std::endl;

        test_missing_ring_field_throws();
        std::cout << "test_missing_ring_field_throws: Success" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error:\n" << e.what() << std::endl;
        return 1;
    }
}
