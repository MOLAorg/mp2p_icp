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
 * @file   test-mp2p_sm_cli_level_walls_tum.cpp
 * @brief  sm-cli level-walls --input-tum: a dense path leveled with the keyframes
 */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double T0       = 1000.0;  // first stamp [s]
constexpr double DURATION = 38.0;  // [s]
constexpr double SPEED    = 1.0;  // [m/s]
constexpr double DENSE_DT = 0.2;  // [s]
constexpr int    KF_EVERY = 10;  // dense poses per keyframe (2 s)
constexpr int    SKIP_RET = 77;  // ctest SKIP_RETURN_CODE

// The odometry frame's tilt at time t: pitch and roll drifting by a few degrees
mrpt::poses::CPose3D frameTilt(double t)
{
    const double s = (t - T0) / DURATION;
    return mrpt::poses::CPose3D::FromYawPitchRoll(
        0.0, mrpt::DEG2RAD(0.5 + 2.0 * s), mrpt::DEG2RAD(1.0 - 1.5 * s));
}

mrpt::poses::CPose3D truePose(double t)
{
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        2.0 + SPEED * (t - T0), 4.0, 1.5, mrpt::DEG2RAD(10.0 * std::sin(t - T0)), 0.0, 0.0);
}

// A 44 x 8 x 3 m corridor (floor, ceiling, four walls) every 10 cm
std::vector<mrpt::math::TPoint3D> corridor()
{
    constexpr double                  sx   = 44.0;
    constexpr double                  sy   = 8.0;
    constexpr double                  sz   = 3.0;
    constexpr double                  step = 0.1;
    std::vector<mrpt::math::TPoint3D> pts;
    const auto                        nx = static_cast<int>(sx / step);
    const auto                        ny = static_cast<int>(sy / step);
    const auto                        nz = static_cast<int>(sz / step);
    for (int i = 0; i <= nx; i++)
    {
        for (int j = 0; j <= ny; j++)
        {
            pts.emplace_back(i * step, j * step, 0.0);
            pts.emplace_back(i * step, j * step, sz);
        }
    }
    for (int k = 1; k < nz; k++)
    {
        for (int i = 0; i <= nx; i++)
        {
            pts.emplace_back(i * step, 0.0, k * step);
            pts.emplace_back(i * step, sy, k * step);
        }
        for (int j = 0; j <= ny; j++)
        {
            pts.emplace_back(0.0, j * step, k * step);
            pts.emplace_back(sx, j * step, k * step);
        }
    }
    return pts;
}

struct Path
{
    std::vector<double>               t;
    std::vector<mrpt::poses::CPose3D> p;
};

// The true motion seen in the drifting frame: each increment and attitude
// rotated by the tilt of its time.
Path driftingPath()
{
    Path       out;
    const auto n = static_cast<int>(DURATION / DENSE_DT);
    for (int k = 0; k <= n; k++)
    {
        const double         t   = T0 + k * DENSE_DT;
        const auto           pt  = truePose(t);
        mrpt::math::TPoint3D pos = pt.translation();
        if (k > 0)
        {
            const double tMid = t - 0.5 * DENSE_DT;
            const auto   step = pt.translation() - truePose(t - DENSE_DT).translation();
            pos               = out.p.back().translation() + frameTilt(tMid).rotateVector(step);
        }
        out.t.push_back(t);
        out.p.push_back(mrpt::poses::CPose3D::FromRotationAndTranslation(
            frameTilt(t).getRotationMatrix() * pt.getRotationMatrix(), pos));
    }
    return out;
}

void writeTum(const std::string& f, const Path& path, double timeShift = 0)
{
    std::ofstream o(f);
    for (size_t i = 0; i < path.t.size(); i++)
    {
        mrpt::math::CQuaternionDouble q;
        path.p[i].getAsQuaternion(q);
        o << mrpt::format(
            "%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n", path.t[i] + timeShift, path.p[i].x(),
            path.p[i].y(), path.p[i].z(), q.x(), q.y(), q.z(), q.w());
    }
}

Path readTum(const std::string& f)
{
    std::ifstream in(f);
    ASSERTMSG_(in.is_open(), "cannot read " + f);
    Path        out;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream ss(line);
        double             t  = 0;
        double             x  = 0;
        double             y  = 0;
        double             z  = 0;
        double             qx = 0;
        double             qy = 0;
        double             qz = 0;
        double             qw = 0;
        ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw;
        out.t.push_back(t);
        out.p.emplace_back(mrpt::math::CQuaternionDouble(qw, qx, qy, qz), x, y, z);
    }
    return out;
}

// Keyframes every KF_EVERY dense poses from dense pose `first` on, with the
// corridor seen from the true pose
void writeSimplemap(const std::string& f, const Path& path, size_t first = 0)
{
    const auto             world = corridor();
    mrpt::maps::CSimpleMap sm;
    for (size_t k = first; k < path.t.size(); k += KF_EVERY)
    {
        const auto truth = truePose(path.t[k]);
        auto       pc    = mrpt::maps::CSimplePointsMap::Create();
        for (const auto& w : world)
        {
            const auto l = truth.inverseComposePoint(w);
            if (l.norm() < 15.0)
            {
                pc->insertPoint(
                    static_cast<float>(l.x), static_cast<float>(l.y), static_cast<float>(l.z));
            }
        }
        auto obs         = mrpt::obs::CObservationPointCloud::Create();
        obs->timestamp   = mrpt::Clock::fromDouble(path.t[k]);
        obs->sensorLabel = "lidar";
        obs->pointcloud  = pc;
        auto sf          = mrpt::obs::CSensoryFrame::Create();
        sf->insert(obs);
        auto pose  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose->mean = path.p[k];
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

double heightRange(const Path& p)
{
    double lo = p.p.front().z();
    double hi = lo;
    for (const auto& q : p.p)
    {
        lo = std::min(lo, q.z());
        hi = std::max(hi, q.z());
    }
    return hi - lo;
}

// The leveled dense path, at each keyframe's time, against the leveled keyframe
void checkFollowsKeyframes(
    const std::string& smFile, const Path& dense, double maxPosErr, size_t first = 0)
{
    mrpt::maps::CSimpleMap sm;
    ASSERT_(sm.loadFromFile(smFile));
    ASSERT_EQUAL_(sm.size(), (dense.t.size() - first + KF_EVERY - 1) / KF_EVERY);
    double maxPos = 0;
    double maxRot = 0;
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto kf   = sm.get(i).pose->getMeanVal();
        const auto d    = dense.p.at(first + i * KF_EVERY);
        const auto diff = kf - d;
        maxPos          = std::max(maxPos, diff.translation().norm());
        const auto ang  = mrpt::poses::CPose3D::FromRotationAndTranslation(
             diff.getRotationMatrix(), mrpt::math::TPoint3D(0, 0, 0));
        maxRot = std::max(
            maxRot, std::acos(std::min(1.0, 0.5 * (ang.getRotationMatrix().trace() - 1.0))));
    }
    std::cout << mrpt::format(
        "Dense path vs keyframes: %.4f m, %.4f deg\n", maxPos, mrpt::RAD2DEG(maxRot));
    ASSERT_LT_(maxPos, maxPosErr);
    ASSERT_LT_(mrpt::RAD2DEG(maxRot), 0.01);
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
        const std::string dir = mrpt::system::getTempFileName() + "_level_walls_tum";
        ASSERT_(mrpt::system::createDirectory(dir));
        const std::string smIn  = dir + "/in.simplemap";
        const std::string tumIn = dir + "/dense.tum";

        const auto path = driftingPath();
        writeTum(tumIn, path);
        writeSimplemap(smIn, path);
        std::cout << "Input height range: " << heightRange(path) << " m\n";
        ASSERT_GT_(heightRange(path), 0.5);

        // Piecewise: the dense path follows the leveled keyframes and is level
        {
            const std::string smOut  = dir + "/pw.simplemap";
            const std::string tumOut = dir + "/pw.tum";
            ASSERT_EQUAL_(
                run(smIn + " " + smOut + " --piecewise 10 --max-correction-deg 10 --input-tum " +
                    tumIn + " --output-tum " + tumOut),
                0);
            const auto leveled = readTum(tumOut);
            ASSERT_EQUAL_(leveled.t.size(), path.t.size());
            checkFollowsKeyframes(smOut, leveled, 0.005);
            std::cout << "Piecewise height range: " << heightRange(leveled) << " m\n";
            ASSERT_LT_(heightRange(leveled), 0.25 * heightRange(path));
        }
        // One rotation of the whole map, and --estimate-only writes the path too
        {
            const std::string smOut  = dir + "/global.simplemap";
            const std::string tumOut = dir + "/global.tum";
            ASSERT_EQUAL_(
                run(smIn + " " + smOut + " --max-correction-deg 10 --input-tum " + tumIn +
                    " --output-tum " + tumOut),
                0);
            checkFollowsKeyframes(smOut, readTum(tumOut), 1e-4);

            const std::string tumOnly = dir + "/estimate_only.tum";
            ASSERT_EQUAL_(
                run(smIn + " --estimate-only --max-correction-deg 10 --input-tum " + tumIn +
                    " --output-tum " + tumOnly),
                0);
            ASSERT_(mrpt::system::fileExists(tumOnly));
        }
        // A dense path that starts 6 s before the first keyframe (e.g. a cropped
        // map): where both exist, it moves as much as the keyframes do
        {
            constexpr size_t  first  = 3 * KF_EVERY;
            const std::string smLate = dir + "/late.simplemap";
            writeSimplemap(smLate, path, first);
            const std::string smOut  = dir + "/late_pw.simplemap";
            const std::string tumOut = dir + "/late_pw.tum";
            ASSERT_EQUAL_(
                run(smLate + " " + smOut + " --piecewise 10 --max-correction-deg 10 --input-tum " +
                    tumIn + " --output-tum " + tumOut),
                0);
            checkFollowsKeyframes(smOut, readTum(tumOut), 0.01, first);
        }
        // Another clock (relative times): refused, the windows cannot be matched
        {
            const std::string tumRel = dir + "/relative.tum";
            writeTum(tumRel, path, -T0);
            ASSERT_(
                run(smIn + " " + dir +
                    "/rel.simplemap --piecewise 10 --max-correction-deg 10 "
                    "--input-tum " +
                    tumRel + " --output-tum " + dir + "/rel.tum") != 0);
            ASSERT_(!mrpt::system::fileExists(dir + "/rel.tum"));
        }
        // Not a TUM file: extra columns, or a null quaternion
        for (const char* bad : {"1000 0 0 0 0 0 0 1 7\n", "1000 0 0 0 0 0 0 0\n"})
        {
            const std::string tumBad = dir + "/bad.tum";
            std::ofstream(tumBad) << bad;
            ASSERT_(
                run(smIn + " --estimate-only --input-tum " + tumBad + " --output-tum " + dir +
                    "/bad_out.tum") != 0);
        }
        // The path must not overwrite either map
        ASSERT_(
            run(smIn + " " + dir + "/same.out --input-tum " + tumIn + " --output-tum " + dir +
                "/same.out") != 0);
        ASSERT_(
            run(smIn + " " + dir + "/y.simplemap --input-tum " + tumIn + " --output-tum " + smIn) !=
            0);
        ASSERT_(mrpt::system::fileExists(smIn) && !mrpt::system::fileExists(dir + "/y.simplemap"));
        // Both options or none
        ASSERT_(run(smIn + " " + dir + "/x.simplemap --input-tum " + tumIn) != 0);

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
