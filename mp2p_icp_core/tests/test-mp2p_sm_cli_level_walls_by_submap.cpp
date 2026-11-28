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
 * @file   test-mp2p_sm_cli_level_walls_by_submap.cpp
 * @brief  sm-cli partition + level-walls --by-submap on a two-storey map whose
 *         tilt error depends on place, with a revisit of the lower storey.
 */

#include <mp2p_icp_filters/simplemap_partition.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double T0       = 2000.0;  // first stamp [s]
constexpr double DENSE_DT = 0.1;  // [s]
constexpr int    KF_EVERY = 10;  // dense poses per keyframe (1 s)
constexpr int    SKIP_RET = 77;  // ctest SKIP_RETURN_CODE

constexpr double Z_LOW  = 1.2;  // sensor height, lower storey
constexpr double Z_HIGH = 4.0;  // sensor height, upper storey
constexpr double SLAB   = 2.6;  // lower ceiling; the upper floor is 0.1 m above

using Pts = std::vector<mrpt::math::TPoint3D>;

// Surfaces of an axis-aligned box, every 10 cm
Pts room(double z0, double z1, double y0 = 0.0, double y1 = 6.0)
{
    constexpr double step = 0.1;
    constexpr double sx   = 10.0;
    Pts              pts;
    for (double x = 0; x <= sx + 1e-9; x += step)
    {
        for (double y = y0; y <= y1 + 1e-9; y += step)
        {
            pts.emplace_back(x, y, z0);
            pts.emplace_back(x, y, z1);
        }
    }
    for (double z = z0 + step; z < z1 - 1e-9; z += step)
    {
        for (double x = 0; x <= sx + 1e-9; x += step)
        {
            pts.emplace_back(x, y0, z);
            pts.emplace_back(x, y1, z);
        }
        for (double y = y0; y <= y1 + 1e-9; y += step)
        {
            pts.emplace_back(0.0, y, z);
            pts.emplace_back(sx, y, z);
        }
    }
    return pts;
}

// A loop around the room, 1.5 m from the walls (20 m long), at `offset` aside
mrpt::math::TPoint3D onLoop(double s, double z, double offset)
{
    s              = std::fmod(s, 20.0);
    const double a = 1.5 + offset;
    const double b = 8.5 - offset;
    const double c = 1.5 + offset;
    const double d = 4.5 - offset;
    if (s < 7.0)
    {
        return {a + s * (b - a) / 7.0, c, z};
    }
    if (s < 10.0)
    {
        return {b, c + (s - 7.0) * (d - c) / 3.0, z};
    }
    if (s < 17.0)
    {
        return {b - (s - 10.0) * (b - a) / 7.0, d, z};
    }
    return {a, d - (s - 17.0) * (d - c) / 3.0, z};
}

// True position: two loops below, up the stairs, two loops above, down, and
// two more loops below (a revisit after more than 30 s)
mrpt::math::TPoint3D truePosition(double t)
{
    const double u = t - T0;
    if (u < 40)
    {
        return onLoop(u, Z_LOW, 0.0);
    }
    if (u < 50)
    {
        const double f = (u - 40) / 10.0;
        return {1.5 + 7.0 * f, 1.5, Z_LOW + f * (Z_HIGH - Z_LOW)};
    }
    if (u < 90)
    {
        return onLoop(8.5 - 1.5 + (u - 50), Z_HIGH, 0.05);
    }
    if (u < 100)
    {
        const double f = (u - 90) / 10.0;
        return {1.5 + 7.0 * f, 1.5, Z_HIGH - f * (Z_HIGH - Z_LOW)};
    }
    return onLoop(7.0 + (u - 100), Z_LOW, 0.1);
}

constexpr double DURATION = 140.0;

mrpt::poses::CPose3D truePose(double t)
{
    const auto p  = truePosition(t);
    const auto p2 = truePosition(t + 0.05);
    const auto yw = std::atan2(p2.y - p.y, p2.x - p.x);
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(p.x, p.y, p.z, yw, 0.0, 0.0);
}

// The map's tilt error depends on the storey (place), blended on the stairs
mrpt::poses::CPose3D tiltAt(double z)
{
    const double f     = std::clamp((z - Z_LOW) / (Z_HIGH - Z_LOW), 0.0, 1.0);
    const double pitch = (1.0 - f) * 1.5 + f * (-1.0);
    const double roll  = (1.0 - f) * 0.5 + f * 1.2;
    return mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll));
}

// The tilted map: everything on a storey rotated about the room center
mrpt::poses::CPose3D mapPose(const mrpt::poses::CPose3D& truth)
{
    const mrpt::math::TPoint3D c(5.0, 3.0, 0.0);
    const auto                 E = tiltAt(truth.z());
    const auto                 p = c + E.rotateVector(truth.translation() - c);
    return mrpt::poses::CPose3D::FromRotationAndTranslation(
        E.getRotationMatrix().asEigen() * truth.getRotationMatrix().asEigen(), p);
}

struct Path
{
    std::vector<double>               t;
    std::vector<mrpt::poses::CPose3D> p;
};

Path densePath(bool inMap)
{
    Path       out;
    const auto n = static_cast<int>(DURATION / DENSE_DT);
    for (int k = 0; k < n; k++)
    {
        const double t = T0 + k * DENSE_DT;
        out.t.push_back(t);
        out.p.push_back(inMap ? mapPose(truePose(t)) : truePose(t));
    }
    return out;
}

void writeTum(const std::string& f, const Path& path)
{
    std::ofstream o(f);
    for (size_t i = 0; i < path.t.size(); i++)
    {
        mrpt::math::CQuaternionDouble q;
        path.p[i].getAsQuaternion(q);
        o << mrpt::format(
            "%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n", path.t[i], path.p[i].x(), path.p[i].y(),
            path.p[i].z(), q.x(), q.y(), q.z(), q.w());
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

// Keyframes every second, at their map pose, seeing the room of their storey,
// or, on the stairs, a stairwell beside the rooms (visibility is not modeled)
// Floors and ceilings sloped along x, walls plumb: walls contradict floors
Pts sloped(Pts pts, double z0, double z1, double slopeDeg)
{
    const double t = std::tan(mrpt::DEG2RAD(slopeDeg));
    for (auto& p : pts)
    {
        if (std::abs(p.z - z0) < 1e-9 || std::abs(p.z - z1) < 1e-9)
        {
            p.z += t * (p.x - 5.0);
        }
    }
    return pts;
}

// `timeShift` moves the keyframe stamps; without `stamps`, keyframes have none
void writeSimplemap(
    const std::string& f, const Path& mapPath, double slopeDeg = 0, bool stamps = true,
    double timeShift = 0)
{
    const Pts lower = sloped(room(0.0, SLAB), 0.0, SLAB, slopeDeg);
    const Pts upper = sloped(room(SLAB + 0.1, SLAB + 2.8), SLAB + 0.1, SLAB + 2.8, slopeDeg);
    const Pts stair = room(0.0, SLAB + 2.8, 6.2, 8.2);
    mrpt::maps::CSimpleMap sm;
    for (size_t k = 0; k < mapPath.t.size(); k += KF_EVERY)
    {
        const auto truth = truePose(mapPath.t[k]);
        const Pts* world = &stair;
        if (truth.z() < Z_LOW + 0.01)
        {
            world = &lower;
        }
        else if (truth.z() > Z_HIGH - 0.01)
        {
            world = &upper;
        }
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        for (const auto& w : *world)
        {
            const auto l = truth.inverseComposePoint(w);
            pc->insertPoint(
                static_cast<float>(l.x), static_cast<float>(l.y), static_cast<float>(l.z));
        }
        auto obs = mrpt::obs::CObservationPointCloud::Create();
        if (stamps)
        {
            obs->timestamp = mrpt::Clock::fromDouble(mapPath.t[k] + timeShift);
        }
        obs->sensorLabel = "lidar";
        obs->pointcloud  = pc;
        auto sf          = mrpt::obs::CSensoryFrame::Create();
        sf->insert(obs);
        auto pose  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose->mean = mapPath.p[k];
        sm.insert(pose, sf);
    }
    ASSERT_(sm.saveToFile(f));
}

int run(const std::string& verb, const std::string& args)
{
    const std::string cmd = std::string("\"") + SM_CLI_PATH + "\" " + verb + " " + args;
    std::cout << "Running: " << cmd << std::endl;
    return std::system(cmd.c_str());
}

bool onStorey(const mrpt::poses::CPose3D& truth)
{
    return std::abs(truth.z() - Z_LOW) < 1e-6 || std::abs(truth.z() - Z_HIGH) < 1e-6;
}

// Revisits of the dense path on a storey (closer than 0.3 m in plan, under 1 m
// in height, more than 30 s apart): rms error of their leveled height
// difference against the true one [m], and the number of pairs. The stairs
// are left out: their tilt error varies along them, which no submap models.
std::pair<double, size_t> revisitError(const Path& leveled, const Path& truth)
{
    double sum   = 0;
    size_t pairs = 0;
    for (size_t i = 0; i < truth.t.size(); i += 5)
    {
        for (size_t j = i + 5; j < truth.t.size(); j += 5)
        {
            const auto d = truth.p[j].translation() - truth.p[i].translation();
            if (truth.t[j] - truth.t[i] > 30.0 && std::abs(d.z) < 1.0 &&
                std::hypot(d.x, d.y) < 0.3 && onStorey(truth.p[i]) && onStorey(truth.p[j]))
            {
                const double e = (leveled.p[j].z() - leveled.p[i].z()) - d.z;
                sum += e * e;
                pairs++;
            }
        }
    }
    return {pairs ? std::sqrt(sum / static_cast<double>(pairs)) : 0.0, pairs};
}

// Largest angle between the leveled and true vertical axis of the poses on
// either storey [deg]
double maxTiltError(const Path& leveled, const Path& truth)
{
    double worst = 0;
    for (size_t i = 0; i < truth.t.size(); i++)
    {
        if (!onStorey(truth.p[i]))
        {
            continue;
        }
        const auto zl = leveled.p[i].getRotationMatrix().asEigen().col(2);
        const auto zt = truth.p[i].getRotationMatrix().asEigen().col(2);
        worst         = std::max(worst, std::acos(std::min(1.0, zl.dot(zt))));
    }
    return mrpt::RAD2DEG(worst);
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
        const std::string dir = mrpt::system::getTempFileName() + "_level_walls_by_submap";
        ASSERT_(mrpt::system::createDirectory(dir));
        const std::string smIn  = dir + "/in.simplemap";
        const std::string tumIn = dir + "/dense.tum";
        const std::string part  = dir + "/part.yaml";

        const auto truth   = densePath(false);
        const auto mapPath = densePath(true);
        writeTum(tumIn, mapPath);
        writeSimplemap(smIn, mapPath);

        // The storeys are different places; both visits of the lower one, one:
        ASSERT_EQUAL_(run("partition", smIn + " " + part), 0);
        const auto p = mp2p_icp_filters::SimplemapPartition::load_from_file(part);
        ASSERT_EQUAL_(p.keyframe_count(), static_cast<size_t>(DURATION));
        std::set<uint32_t> lowFirst;
        std::set<uint32_t> lowLast;
        std::set<uint32_t> high;
        for (size_t k = 0; k < p.keyframe_count(); k++)
        {
            if (k < 38)
            {
                lowFirst.insert(p.keyframe_submap[k]);
            }
            else if (k >= 52 && k < 88)
            {
                high.insert(p.keyframe_submap[k]);
            }
            else if (k >= 102)
            {
                lowLast.insert(p.keyframe_submap[k]);
            }
        }
        std::cout << "Submaps: " << p.submap_count() << "\n";
        ASSERT_(lowFirst == lowLast);
        for (const auto s : high)
        {
            ASSERT_(lowFirst.count(s) == 0);
        }

        const auto before = revisitError(mapPath, truth);
        std::cout << mrpt::format(
            "Revisits: %zu pairs, before leveling %.2f mm rms\n", before.second,
            1e3 * before.first);
        ASSERT_GT_(before.second, 50UL);

        // By submap: level, and the revisits keep their true height difference
        const std::string tumSub = dir + "/sub.tum";
        ASSERT_EQUAL_(
            run("level-walls", smIn + " " + dir + "/sub.simplemap --by-submap " + part +
                                   " --input-tum " + tumIn + " --output-tum " + tumSub),
            0);
        const auto sub    = readTum(tumSub);
        const auto subErr = revisitError(sub, truth);
        const auto subTlt = maxTiltError(sub, truth);
        std::cout << mrpt::format(
            "By submap: revisits %.2f mm rms, tilt error up to %.3f deg\n", 1e3 * subErr.first,
            subTlt);
        ASSERT_LT_(subErr.first, 1e-3);
        ASSERT_LT_(subTlt, 0.1);

        // By time: also level, but the revisit no longer matches
        const std::string tumPw = dir + "/pw.tum";
        ASSERT_EQUAL_(
            run("level-walls", smIn + " " + dir + "/pw.simplemap --piecewise 20 --input-tum " +
                                   tumIn + " --output-tum " + tumPw),
            0);
        const auto pwErr = revisitError(readTum(tumPw), truth);
        std::cout << mrpt::format("By time: revisits %.2f mm rms\n", 1e3 * pwErr.first);
        ASSERT_GT_(pwErr.first, 5 * subErr.first);

        // The output simple-map holds the poses of the leveled path at the keyframes:
        {
            mrpt::maps::CSimpleMap out;
            ASSERT_(out.loadFromFile(dir + "/sub.simplemap"));
            ASSERT_EQUAL_(out.size(), p.keyframe_count());
            for (size_t k = 0; k < out.size(); k++)
            {
                const auto kf = out.get(k).pose->getMeanVal();
                const auto d  = sub.p.at(k * KF_EVERY);
                ASSERT_LT_((kf.translation() - d.translation()).norm(), 1e-5);
                ASSERT_LT_(
                    (kf.getRotationMatrix().asEigen() - d.getRotationMatrix().asEigen()).norm(),
                    1e-6);
            }
        }
        // --estimate-only writes the path, not the map:
        {
            const std::string t = dir + "/est_only.tum";
            ASSERT_EQUAL_(
                run("level-walls", smIn + " --estimate-only --by-submap " + part + " --input-tum " +
                                       tumIn + " --output-tum " + t),
                0);
            ASSERT_(mrpt::system::fileExists(t));
        }
        // Refused, and nothing written: a submap above --max-correction-deg, or
        // no submap within --max-window-sigma-deg
        for (const char* opts : {" --max-correction-deg 1.0", " --max-window-sigma-deg 0.0001"})
        {
            const std::string o = dir + "/refused.simplemap";
            ASSERT_(run("level-walls", smIn + " " + o + " --by-submap " + part + opts) != 0);
            ASSERT_(!mrpt::system::fileExists(o));
        }
        // Floors that slope where walls are plumb, everywhere: refused
        {
            const std::string smSloped = dir + "/sloped.simplemap";
            const std::string pSloped  = dir + "/sloped.yaml";
            writeSimplemap(smSloped, mapPath, 4.0);
            ASSERT_EQUAL_(run("partition", smSloped + " " + pSloped), 0);
            ASSERT_(
                run("level-walls",
                    smSloped + " " + dir + "/sloped_out.simplemap --by-submap " + pSloped) != 0);
            ASSERT_(!mrpt::system::fileExists(dir + "/sloped_out.simplemap"));
        }
        // Keyframes without timestamps: fine without a dense path
        {
            const std::string smNoT = dir + "/no_stamps.simplemap";
            const std::string pNoT  = dir + "/no_stamps.yaml";
            writeSimplemap(smNoT, mapPath, 0, false);
            ASSERT_EQUAL_(run("partition", smNoT + " " + pNoT), 0);
            ASSERT_EQUAL_(
                run("level-walls",
                    smNoT + " " + dir + "/no_stamps_out.simplemap --by-submap " + pNoT),
                0);
        }
        // Outputs that would overwrite an input, through a hard link too:
        {
            const std::string link = dir + "/link.simplemap";
            std::error_code   ec;
            std::filesystem::create_hard_link(smIn, link, ec);
            if (!ec)
            {
                ASSERT_(run("partition", smIn + " " + link) != 0);
                ASSERT_(run("level-walls", smIn + " " + link + " --by-submap " + part) != 0);
            }
            ASSERT_(run("level-walls", smIn + " " + part + " --by-submap " + part) != 0);
            ASSERT_(
                run("level-walls", smIn + " --estimate-only --by-submap " + part + " --input-tum " +
                                       tumIn + " --output-tum " + tumIn) != 0);
        }
        // A partition of a map with other keyframe stamps is refused:
        {
            const std::string smLate = dir + "/late.simplemap";
            writeSimplemap(smLate, mapPath, 0, true, 1.0);
            ASSERT_(run("level-walls", smLate + " --estimate-only --by-submap " + part) != 0);
        }

        // A partition of another map is refused:
        {
            mp2p_icp_filters::SimplemapPartition other = p;
            other.keyframe_submap.pop_back();
            other.keyframe_links.pop_back();
            const std::string otherFile = dir + "/other.yaml";
            other.save_to_file(otherFile);
            ASSERT_(run("level-walls", smIn + " --estimate-only --by-submap " + otherFile) != 0);
        }
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
