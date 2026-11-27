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
 * @file   test-mp2p_sm_cli_level_walls_guard.cpp
 * @brief  sm-cli level-walls refuses scenes whose walls contradict their floors
 */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr double T0       = 1000.0;  // first stamp [s]
constexpr double LENGTH   = 100.0;  // corridor length [m], walked at 1 m/s
constexpr double WIDTH    = 6.0;
constexpr double HEIGHT   = 3.0;
constexpr double STEP     = 0.1;  // point spacing [m]
constexpr double VIEW     = 4.0;  // each keyframe sees this far [m]
constexpr double SLOPE    = 8.0;  // [deg]
constexpr int    SKIP_RET = 77;  // ctest SKIP_RETURN_CODE

struct Corridor
{
    bool   crossWalls = true;  // pilasters every 5 m: walls of a second direction
    double slopedUpTo = 0;  // the floor rises SLOPE deg from x=0 up to this x [m]
};

double floorZ(const Corridor& c, double x)
{
    return std::tan(mrpt::DEG2RAD(SLOPE)) * std::min(x, c.slopedUpTo);
}

// Long walls at y=0 and y=WIDTH (plumb), optional cross walls, and the floor
std::vector<mrpt::math::TPoint3D> corridor(const Corridor& c)
{
    std::vector<mrpt::math::TPoint3D> pts;
    const auto                        nx = static_cast<int>(LENGTH / STEP);
    const auto                        ny = static_cast<int>(WIDTH / STEP);
    const auto                        nz = static_cast<int>(HEIGHT / STEP);
    for (int i = 0; i <= nx; i++)
    {
        const double x  = i * STEP;
        const double zf = floorZ(c, x);
        for (int j = 0; j <= ny; j++)
        {
            pts.emplace_back(x, j * STEP, zf);
        }
        for (int k = 1; k < nz; k++)
        {
            pts.emplace_back(x, 0.0, zf + k * STEP);
            pts.emplace_back(x, WIDTH, zf + k * STEP);
        }
    }
    if (c.crossWalls)
    {
        for (double x = 2.5; x < LENGTH; x += 5.0)
        {
            const double zf = floorZ(c, x);
            for (double y = 0; y <= 1.0; y += STEP)
            {
                for (int k = 1; k < nz; k++)
                {
                    pts.emplace_back(x, y, zf + k * STEP);
                    pts.emplace_back(x, WIDTH - y, zf + k * STEP);
                }
            }
        }
    }
    return pts;
}

// Keyframes every second along the corridor, at their true (level) attitude
void writeSimplemap(const std::string& f, const Corridor& c)
{
    const auto             world = corridor(c);
    mrpt::maps::CSimpleMap sm;
    for (int k = 0; k < static_cast<int>(LENGTH); k++)
    {
        const double               x = 0.5 + k;
        const mrpt::poses::CPose3D truth(x, 0.5 * WIDTH, floorZ(c, x) + 1.5, 0, 0, 0);
        auto                       pc = mrpt::maps::CSimplePointsMap::Create();
        for (const auto& w : world)
        {
            const auto l = truth.inverseComposePoint(w);
            if (l.norm() < VIEW)
            {
                pc->insertPoint(
                    static_cast<float>(l.x), static_cast<float>(l.y), static_cast<float>(l.z));
            }
        }
        auto obs         = mrpt::obs::CObservationPointCloud::Create();
        obs->timestamp   = mrpt::Clock::fromDouble(T0 + k);
        obs->sensorLabel = "lidar";
        obs->pointcloud  = pc;
        auto sf          = mrpt::obs::CSensoryFrame::Create();
        sf->insert(obs);
        auto pose  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose->mean = truth;
        sm.insert(pose, sf);
    }
    ASSERT_(sm.saveToFile(f));
}

int run(const std::string& args)
{
    const std::string cmd = std::string("\"") + SM_CLI_PATH + "\" level-walls " + args;
    std::cout << "Running: " << cmd << std::endl;
    return std::system(cmd.c_str());
}

void expectExit(const std::string& args, bool ok)
{
    const int ret = run(args);
    std::cout << "  -> " << (ret == 0 ? "exit 0" : "failed") << "\n";
    ASSERTMSG_((ret == 0) == ok, "unexpected exit status for: " + args);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        const std::string bin = SM_CLI_PATH;
        if (bin.empty() || !mrpt::system::fileExists(bin))
        {
            std::cout << "sm-cli not built: test skipped.\n";
            return SKIP_RET;
        }
        const std::string dir = mrpt::system::getTempFileName() + "_level_walls_guard";
        ASSERT_(mrpt::system::createDirectory(dir));
        const std::string out = " " + dir + "/out.simplemap --max-correction-deg 20";

        // A level corridor: walls confirm the floor
        const std::string level = dir + "/level.simplemap";
        writeSimplemap(level, {});
        expectExit(level + out, true);
        expectExit(level + out + " --piecewise 20", true);

        // The floor rises 8 deg along the first 80 m of 100: the walls contradict it
        // in 4 of the 5 windows, and over the whole map
        const std::string ramp80 = dir + "/ramp80.simplemap";
        writeSimplemap(ramp80, {true, 80.0});
        expectExit(ramp80 + out, false);
        expectExit(ramp80 + " --estimate-only --max-correction-deg 20", false);
        expectExit(ramp80 + out + " --max-walls-flats-deg 0", true);
        expectExit(ramp80 + out + " --piecewise 20", false);

        // Along the first 60 m: 3 windows of 5, below the refusal
        const std::string ramp60 = dir + "/ramp60.simplemap";
        writeSimplemap(ramp60, {true, 60.0});
        expectExit(ramp60 + out + " --piecewise 20", true);

        // A single facade direction over a level floor: the walls cannot tell, so they
        // neither confirm nor contradict the floor, and nothing is refused
        const std::string facade = dir + "/facade.simplemap";
        writeSimplemap(facade, {false, 0.0});
        expectExit(facade + out, true);
        expectExit(facade + out + " --piecewise 20", true);

        // Invalid limit
        expectExit(level + out + " --max-walls-flats-deg -1", false);

        mrpt::system::deleteFilesInDirectory(dir, true);
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
