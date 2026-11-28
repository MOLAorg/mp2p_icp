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
 * @file   sm-cli-level-walls.cpp
 * @brief  Levels a simplemap so walls are vertical and floors horizontal.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 8, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/yaml_variables.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mp2p_icp_filters/level_by_place.h>
#include <mp2p_icp_filters/simplemap_partition.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <utility>

#include "sm-cli.h"

namespace
{
int printCommandsLevelWalls(bool showErrorMsg);

constexpr const char* ANALYSIS_LAYER = "level_walls_analysis";

// Converts keyframes [first, last] into one voxel-decimated cloud in the map frame,
// merging all point layers the pipeline outputs.
mrpt::maps::CPointsMap::Ptr buildAnalysisCloud(
    const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, size_t first,
    size_t last)
{
    mp2p_icp_filters::sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.start_index     = first;
    opts.end_index       = last;

    mp2p_icp::metric_map_t mm;
    mp2p_icp_filters::simplemap_to_metricmap(sm, mm, pipeline, opts);

    mrpt::containers::yaml layers = mrpt::containers::yaml::Sequence();
    for (const auto& [name, layer] : mm.layers)
    {
        if (mp2p_icp::MapToPointsMap(*layer))
        {
            layers.push_back(name);
        }
    }
    ASSERTMSG_(!layers.asSequence().empty(), "The pipeline produced no point cloud layer.");

    mrpt::containers::yaml p;
    p["input_pointcloud_layer"]  = layers;
    p["output_pointcloud_layer"] = ANALYSIS_LAYER;
    p["voxel_filter_resolution"] = cli->arg_voxel;
    p["decimate_method"]         = "DecimateMethod::FirstPoint";

    mp2p_icp_filters::FilterDecimateVoxels voxelFilter;
    voxelFilter.initialize(p);
    voxelFilter.filter(mm);

    auto pc = mm.point_layer(ANALYSIS_LAYER);
    ASSERT_(pc);
    return pc;
}

mp2p_icp_filters::EstimateUpParams estimateParams()
{
    mp2p_icp_filters::EstimateUpParams p;
    p.wall_nz                   = cli->arg_wall_nz;
    p.flat_nz                   = cli->arg_flat_nz;
    p.flats_weight              = cli->arg_walls_only ? 0.0 : 1.0;
    p.cluster_size              = cli->arg_cluster_size;
    p.max_walls_flats_angle_deg = cli->arg_max_walls_flats;
    return p;
}

std::vector<mrpt::math::TVector3D> normalsFromCloud(
    const mrpt::maps::CPointsMap& pc, std::vector<mrpt::math::TPoint3Df>* points = nullptr)
{
    mp2p_icp_filters::PlanarNormalsParams p;
    p.search_radius = cli->arg_normal_radius;
    return mp2p_icp_filters::estimate_planar_normals(pc, p, points);
}

// "[0 0 0 yaw pitch roll]" in degrees, as expected by `sm-cli tf`
std::string tfString(const mrpt::poses::CPose3D& p)
{
    return mrpt::format(
        "[0 0 0 %.4f %.4f %.4f]", mrpt::RAD2DEG(p.yaw()), mrpt::RAD2DEG(p.pitch()),
        mrpt::RAD2DEG(p.roll()));
}

std::optional<double> keyframeTime(const mrpt::obs::CSensoryFrame& sf)
{
    for (const auto& obs : sf)
    {
        if (!obs || IS_CLASS(*obs, mrpt::obs::CObservationComment))
        {
            continue;
        }
        if (const auto t = obs->getTimeStamp(); t != mrpt::system::InvalidTimeStamp())
        {
            return mrpt::Clock::toDouble(t);
        }
    }
    return std::nullopt;
}

// Times of all keyframes, if all have one and they are sorted.
std::optional<std::vector<double>> keyframeTimes(const mrpt::maps::CSimpleMap& sm)
{
    std::vector<double> times;
    times.reserve(sm.size());
    for (const auto& [pose, sf, twist] : sm)
    {
        const auto t = sf ? keyframeTime(*sf) : std::nullopt;
        if (!t)
        {
            return std::nullopt;
        }
        times.push_back(*t);
    }
    if (times.empty() || !std::is_sorted(times.begin(), times.end()) ||
        !(times.back() > times.front()))
    {
        return std::nullopt;
    }
    return times;
}

// Contiguous keyframe index ranges [first, last], from the window of each keyframe.
std::vector<std::pair<size_t, size_t>> rangesOf(const std::vector<size_t>& windowOfKF)
{
    std::vector<std::pair<size_t, size_t>> ranges;
    for (size_t i = 0; i < windowOfKF.size(); i++)
    {
        if (i == 0 || windowOfKF[i] != windowOfKF[i - 1])
        {
            ranges.emplace_back(i, i);
        }
        ranges.back().second = i;
    }
    return ranges;
}

// Keyframe index ranges [first, last] of N windows of equal duration, or of
// equal keyframe count if any keyframe lacks a timestamp.
std::vector<std::pair<size_t, size_t>> timeWindows(const mrpt::maps::CSimpleMap& sm, size_t N)
{
    const size_t        nKFs = sm.size();
    std::vector<size_t> windowOfKF(nKFs);

    // Windows must be contiguous keyframe ranges, so times must be sorted:
    if (const auto times = keyframeTimes(sm); times)
    {
        const double t0 = times->front();
        const double dt = (times->back() - t0) / static_cast<double>(N);
        for (size_t i = 0; i < nKFs; i++)
        {
            windowOfKF[i] = std::min(N - 1, static_cast<size_t>(((*times)[i] - t0) / dt));
        }
    }
    else
    {
        std::cout << "Keyframe timestamps are missing or unsorted: using windows of equal keyframe "
                     "count.\n";
        for (size_t i = 0; i < nKFs; i++)
        {
            windowOfKF[i] = (i * N) / nKFs;
        }
    }
    return rangesOf(windowOfKF);
}

// Keyframe index ranges [first, last] of consecutive windows of `duration` seconds.
std::vector<std::pair<size_t, size_t>> durationWindows(
    const std::vector<double>& times, double duration)
{
    std::vector<size_t> windowOfKF(times.size());
    for (size_t i = 0; i < times.size(); i++)
    {
        windowOfKF[i] = static_cast<size_t>((times[i] - times.front()) / duration);
    }
    auto ranges = rangesOf(windowOfKF);

    // A short trailing window would be held to the end on few keyframes: merge it
    if (ranges.size() > 1 &&
        times[ranges.back().second] - times[ranges.back().first] < 0.5 * duration)
    {
        ranges[ranges.size() - 2].second = ranges.back().second;
        ranges.pop_back();
    }
    return ranges;
}

struct TumPath
{
    std::vector<double>               times;
    std::vector<mrpt::poses::CPose3D> poses;
};

// Reads a TUM trajectory ("t x y z qx qy qz qw" rows, '#' comments). Times
// must be sorted, as level_poses_from_up() integrates the path in order.
TumPath readTum(const std::string& file)
{
    std::ifstream f(file);
    ASSERTMSG_(f.is_open(), "Cannot read --input-tum file: " + file);
    TumPath     path;
    std::string line;
    size_t      lineNum = 0;
    while (std::getline(f, line))
    {
        lineNum++;
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#')
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
        std::string        extra;
        ASSERTMSG_(
            static_cast<bool>(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw) && !(ss >> extra),
            mrpt::format("%s:%zu: expected exactly 't x y z qx qy qz qw'", file.c_str(), lineNum));
        const double qn = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        ASSERTMSG_(
            std::isfinite(qn) && qn > 1e-6,
            mrpt::format("%s:%zu: invalid quaternion", file.c_str(), lineNum));
        mrpt::math::CQuaternionDouble q(qw, qx, qy, qz);
        q.normalize();
        path.times.push_back(t);
        path.poses.emplace_back(q, x, y, z);
    }
    // A read error ends the loop like the end of the file: not a whole path
    ASSERTMSG_(!f.bad(), "Error reading --input-tum: " + file);
    ASSERTMSG_(!path.times.empty(), "--input-tum has no poses: " + file);
    ASSERTMSG_(
        std::is_sorted(path.times.begin(), path.times.end()),
        "--input-tum timestamps must be sorted: " + file);
    return path;
}

void writeTum(const std::string& file, const TumPath& path)
{
    std::ofstream f(file);
    ASSERTMSG_(f.is_open(), "Cannot write --output-tum file: " + file);
    f << "# t x y z qx qy qz qw\n";
    for (size_t i = 0; i < path.times.size(); i++)
    {
        const auto&                   p = path.poses[i];
        mrpt::math::CQuaternionDouble q;
        p.getAsQuaternion(q);
        f << mrpt::format(
            "%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n", path.times[i], p.x(), p.y(), p.z(), q.x(),
            q.y(), q.z(), q.w());
    }
    f.close();
    ASSERTMSG_(!f.fail(), "Error writing --output-tum file: " + file);
    std::cout << "Leveled trajectory (" << path.times.size() << " poses) written to: '" << file
              << "'\n";
}

// Position at time t along a sorted path, linearly interpolated (held at the ends)
mrpt::math::TPoint3D positionAt(
    const std::vector<double>& times, const std::vector<mrpt::poses::CPose3D>& poses, double t)
{
    const auto it = std::lower_bound(times.begin(), times.end(), t);
    if (it == times.begin())
    {
        return poses.front().translation();
    }
    if (it == times.end())
    {
        return poses.back().translation();
    }
    const size_t i   = static_cast<size_t>(it - times.begin());
    const double dt  = times[i] - times[i - 1];
    const double frc = dt > 0 ? (t - times[i - 1]) / dt : 1.0;
    const auto   a   = poses[i - 1].translation();
    const auto   b   = poses[i].translation();
    return a + (b - a) * frc;
}

// --input-tum must be in the keyframes' clock: no overlap at all means
// another clock (e.g. relative times), and it is refused.
// `held` names what holds its correction outside the keyframe times
void checkTumOverlap(const TumPath& path, const std::vector<double>& kfTimes, const char* held)
{
    const double t0 = kfTimes.front();
    const double t1 = kfTimes.back();
    ASSERTMSG_(
        path.times.back() >= t0 && path.times.front() <= t1,
        mrpt::format(
            "--input-tum times [%.3f, %.3f] do not overlap the keyframes' [%.3f, %.3f]: they "
            "must use the same clock",
            path.times.front(), path.times.back(), t0, t1));
    size_t outside = 0;
    for (const double t : path.times)
    {
        outside += (t < t0 || t > t1) ? 1 : 0;
    }
    if (outside > 0)
    {
        std::cout << mrpt::format(
            "Warning: %zu of %zu --input-tum poses are outside the keyframes' time range; there "
            "the correction of the nearest %s is held.\n",
            outside, path.times.size(), held);
    }
}

// RMS of the angle between wall normals and the horizontal plane [deg]
double wallElevationRmsDeg(const std::vector<mrpt::math::TVector3D>& normals)
{
    double sumSqr = 0;
    size_t n      = 0;
    for (const auto& v : normals)
    {
        if (std::abs(v.z) < cli->arg_wall_nz)
        {
            sumSqr += mrpt::square(std::asin(v.z));
            n++;
        }
    }
    return n > 0 ? mrpt::RAD2DEG(std::sqrt(sumSqr / static_cast<double>(n))) : 0.0;
}

void printEstimate(const mp2p_icp_filters::EstimateUpResult& est)
{
    std::cout << mrpt::format(
        "Normals used: walls=%zu flats=%zu\n"
        "Estimated up: [%+.6f %+.6f %+.6f]\n"
        "Tilt        : %.3f deg\n"
        "Sigma up    : x %.3f y %.3f deg (%zu groups)\n",
        est.wall_count, est.flat_count, est.up.x, est.up.y, est.up.z, est.tilt_deg(),
        est.up_sigmas_deg()[0], est.up_sigmas_deg()[1], est.cov_groups);
    if (est.single_wall_direction)
    {
        std::cout << "Warning: all walls share one direction: the estimate relies on the floors "
                     "and ceilings.\n";
    }
    if (std::isfinite(est.walls_flats_angle_deg))
    {
        std::cout << mrpt::format(
            "Walls vs floors: %.3f deg apart%s\n", est.walls_flats_angle_deg,
            est.walls_contradict_flats ? " (beyond --max-walls-flats-deg: they contradict)" : "");
    }
    else if (est.references_disagree)
    {
        std::cout << "Walls vs floors: the walls alone do not give up, so they can neither confirm "
                     "nor contradict the floors.\n";
    }
}

// The estimate of one window of keyframes, also as a --windows-out row.
struct WindowEstimate
{
    size_t                                            first = 0;
    size_t                                            last  = 0;
    double                                            t0    = 0;
    double                                            t1    = 0;
    std::optional<mp2p_icp_filters::EstimateUpResult> est;

    /// Observable, with a reliable covariance and sigmas within
    /// --max-window-sigma-deg: what --piecewise uses.
    bool usable = false;
};

// Normals and their points
struct NormalsSet
{
    std::vector<mrpt::math::TVector3D> normals;
    std::vector<mrpt::math::TPoint3Df> points;
};

// Estimates up from normals, and tells whether --piecewise or --by-submap may use it.
void estimateFromNormals(
    WindowEstimate& w, const std::vector<mrpt::math::TVector3D>& normals,
    const std::vector<mrpt::math::TPoint3Df>& points)
{
    try
    {
        w.est = mp2p_icp_filters::estimate_up_from_normals(normals, estimateParams(), &points);
        printEstimate(*w.est);

        const auto sigmas = w.est->up_sigmas_deg();
        // With --cluster-size 0 the sigmas are per-normal: not checked for reliability
        if (cli->arg_cluster_size > 0 && !w.est->cov_reliable)
        {
            std::cout << "Not usable: too few groups of normals for a reliable sigma.\n";
        }
        else if (std::max(sigmas[0], sigmas[1]) > cli->arg_max_window_sigma)
        {
            std::cout << "Not usable: sigma above --max-window-sigma-deg.\n";
        }
        else
        {
            w.usable = true;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "Could not estimate: " << mrpt::exception_to_str(e) << "\n";
    }
}

// Estimates up on the analysis cloud of keyframes [first, last] of `sm`. If
// `gathered` is given, it also receives the normals and their points.
void estimateInto(
    WindowEstimate& w, const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline,
    size_t first, size_t last, NormalsSet* gathered = nullptr)
{
    std::vector<mrpt::math::TVector3D> normals;
    std::vector<mrpt::math::TPoint3Df> points;
    try
    {
        const auto pc = buildAnalysisCloud(sm, pipeline, first, last);
        normals       = normalsFromCloud(*pc, &points);
    }
    catch (const std::exception& e)
    {
        std::cout << "Could not estimate: " << mrpt::exception_to_str(e) << "\n";
        return;
    }
    estimateFromNormals(w, normals, points);
    if (gathered)
    {
        gathered->normals = std::move(normals);
        gathered->points  = std::move(points);
    }
}

WindowEstimate estimateWindow(
    const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline,
    const std::optional<std::vector<double>>& times, size_t index, size_t count, size_t first,
    size_t last)
{
    WindowEstimate w;
    w.first = first;
    w.last  = last;
    if (times)
    {
        w.t0 = (*times)[first];
        w.t1 = (*times)[last];
    }
    std::cout << "\nWindow " << (index + 1) << "/" << count << ": keyframes [" << first << ", "
              << last << "]";
    if (times)
    {
        std::cout << mrpt::format(
            ", t=[%.3f, %.3f] s", w.t0 - times->front(), w.t1 - times->front());
    }
    std::cout << "\n";
    estimateInto(w, sm, pipeline, first, last);
    return w;
}

void writeWindowsOut(const std::vector<WindowEstimate>& windows)
{
    if (cli->arg_windows_out.empty())
    {
        return;
    }
    std::ofstream f(cli->arg_windows_out);
    ASSERTMSG_(f.is_open(), "Cannot write --windows-out file: " + cli->arg_windows_out);
    f << "# window kf_first kf_last t_start t_end up_x up_y up_z tilt_deg sigma_x_deg "
         "sigma_y_deg walls flats groups observable usable walls_flats_deg flats_unconfirmed "
         "flats_contradicted\n";
    for (size_t i = 0; i < windows.size(); i++)
    {
        const auto& w = windows[i];
        f << mrpt::format("%zu %zu %zu %.6f %.6f ", i, w.first, w.last, w.t0, w.t1);
        if (w.est)
        {
            const auto& e = *w.est;
            f << mrpt::format(
                "%.8f %.8f %.8f %.4f %.4f %.4f %zu %zu %zu 1 %d %.4f %d %d\n", e.up.x, e.up.y,
                e.up.z, e.tilt_deg(), e.up_sigmas_deg()[0], e.up_sigmas_deg()[1], e.wall_count,
                e.flat_count, e.cov_groups, w.usable ? 1 : 0, e.walls_flats_angle_deg,
                e.references_disagree ? 1 : 0, e.walls_contradict_flats ? 1 : 0);
        }
        else
        {
            f << "nan nan nan nan nan nan 0 0 0 0 0 nan 0 0\n";
        }
    }
    f.close();
    ASSERTMSG_(!f.fail(), "Error writing --windows-out file: " + cli->arg_windows_out);
    std::cout << "Windows written to: '" << cli->arg_windows_out << "'\n";
}

// Systematic error of the references added to an estimate's covariance when
// testing whether a correction is needed at all [deg]
constexpr double GATE_SYSTEMATIC_DEG = 0.1;

// The horizontal components of up and their information, with the systematic
// error of the references added to the covariance
std::pair<Eigen::Vector2d, Eigen::Matrix2d> upInfo(
    const mrpt::math::TVector3D& up, const mrpt::math::CMatrixDouble22& cov)
{
    const double    f = mrpt::DEG2RAD(GATE_SYSTEMATIC_DEG);
    const auto      u = up * (1.0 / up.norm());
    Eigen::Matrix2d C = cov.asEigen();
    C += f * f * Eigen::Matrix2d::Identity();
    return {Eigen::Vector2d(u.x, u.y), C.inverse()};
}

// How many sigmas an up estimate is from vertical (its Mahalanobis distance)
double significance(const mrpt::math::TVector3D& up, const mrpt::math::CMatrixDouble22& cov)
{
    const auto [h, I] = upInfo(up, cov);
    return std::sqrt(h.dot(I * h));
}

// --min-significance, with the default of each mode
double minSignificance(bool bySubmap)
{
    if (cli->arg_min_significance >= 0)
    {
        return cli->arg_min_significance;
    }
    return bySubmap ? 2.0 : 0.0;
}

// Not leveled because no estimate is significantly off vertical
int notSignificant(double best, double threshold, const char* what)
{
    std::cout << mrpt::format(
        "Not leveled: %s at most %.2f sigma from vertical, below --min-significance %.2f: "
        "the map is level within what the walls and floors can tell.\n",
        what, best, threshold);
    return 1;
}

// A sloped or quarried scene: nearly every window's walls contradict its
// floors. A few such windows (a ramp, a sloped lawn) are not a reason to
// stand down, and skipping them only leaves gaps to interpolate over.
// Windows whose walls cannot tell (one wall direction, along a single
// facade) are not counted: they say nothing against the floors.
// `weights`, if given, weigh each window (e.g. by its keyframes) instead of 1.
bool slopedScene(
    const std::vector<WindowEstimate>& windows, const char* what,
    const std::vector<size_t>* weights = nullptr)
{
    size_t estimated    = 0;
    size_t contradicted = 0;
    for (size_t i = 0; i < windows.size(); i++)
    {
        const auto& w = windows[i];
        if (w.est)
        {
            const size_t n = weights ? (*weights)[i] : 1;
            estimated += n;
            contradicted += w.est->walls_contradict_flats ? n : 0;
        }
    }
    if (estimated > 0 && contradicted * 5 >= estimated * 4)
    {
        setConsoleErrorColor();
        std::cerr << mrpt::format(
            "Error: in %zu of the %zu %s with an estimate the walls contradict the floors "
            "(--max-walls-flats-deg): sloped ground or leaning walls, not a scene of plumb walls "
            "and level floors. Nothing was leveled.\n",
            contradicted, estimated, what);
        setConsoleNormalColor();
        return true;
    }
    return false;
}

// How leveling changed the height difference of keyframes that revisit a
// place (closer than 0.3 m in plan, under 1 m apart in height, over 30 s
// apart in time): a correction that depends on place keeps it.
void printRevisitSplit(
    const std::vector<double>& times, const std::vector<mrpt::poses::CPose3D>& before,
    const std::vector<mrpt::poses::CPose3D>& after)
{
    // Candidates from a 0.3 m plan grid: the same and the eight neighbor cells
    constexpr double                                           cell = 0.3;
    std::map<std::pair<int64_t, int64_t>, std::vector<size_t>> grid;
    const auto cellOf = [&](const mrpt::poses::CPose3D& p)
    {
        return std::make_pair(
            static_cast<int64_t>(std::floor(p.x() / cell)),
            static_cast<int64_t>(std::floor(p.y() / cell)));
    };
    for (size_t i = 0; i < before.size(); i++)
    {
        grid[cellOf(before[i])].push_back(i);
    }
    double sumSqr = 0;
    size_t pairs  = 0;
    for (size_t i = 0; i < before.size(); i++)
    {
        const auto [cx, cy] = cellOf(before[i]);
        for (int64_t dx = -1; dx <= 1; dx++)
        {
            for (int64_t dy = -1; dy <= 1; dy++)
            {
                const auto it = grid.find({cx + dx, cy + dy});
                if (it == grid.end())
                {
                    continue;
                }
                for (const size_t j : it->second)
                {
                    if (j <= i)
                    {
                        continue;
                    }
                    const auto d = before[j].translation() - before[i].translation();
                    if (times[j] - times[i] > 30.0 && std::abs(d.z) < 1.0 &&
                        std::hypot(d.x, d.y) < cell)
                    {
                        const double split = (after[j].z() - after[i].z()) - d.z;
                        sumSqr += split * split;
                        pairs++;
                    }
                }
            }
        }
    }
    if (pairs == 0)
    {
        std::cout << "Revisits: no keyframe pair revisits a place.\n";
        return;
    }
    std::cout << mrpt::format(
        "Revisits: %zu keyframe pairs, change of their height difference %.3f cm rms.\n", pairs,
        100.0 * std::sqrt(sumSqr / static_cast<double>(pairs)));
}

// --piecewise: a correction per time window, interpolated between windows.
int levelPiecewise(
    mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, bool estimateOnly,
    std::optional<TumPath> inputTum)
{
    const auto times = keyframeTimes(sm);
    ASSERTMSG_(times, "--piecewise needs sorted timestamps in all keyframes");
    if (inputTum)
    {
        checkTumOverlap(*inputTum, *times, "window");
    }

    const auto ranges = durationWindows(*times, cli->arg_piecewise);

    std::vector<WindowEstimate>             windows;
    std::vector<mp2p_icp_filters::UpSample> samples;
    double                                  maxTilt = 0;
    for (size_t i = 0; i < ranges.size(); i++)
    {
        auto w = estimateWindow(
            sm, pipeline, times, i, ranges.size(), ranges[i].first, ranges[i].second);
        if (w.usable)
        {
            samples.push_back({0.5 * (w.t0 + w.t1), w.est->up});
            maxTilt = std::max(maxTilt, w.est->tilt_deg());
        }
        windows.push_back(std::move(w));
    }
    writeWindowsOut(windows);

    if (slopedScene(windows, "windows"))
    {
        return 1;
    }

    ASSERTMSG_(!samples.empty(), "No time window had a usable up direction.");
    std::cout << mrpt::format(
        "\nPiecewise: %zu of %zu windows usable, largest tilt %.3f deg\n", samples.size(),
        windows.size(), maxTilt);

    if (maxTilt > cli->arg_max_correction_deg)
    {
        setConsoleErrorColor();
        std::cerr << mrpt::format(
            "Error: a window needs a correction (%.3f deg) above --max-correction-deg (%.3f "
            "deg).\n",
            maxTilt, cli->arg_max_correction_deg);
        setConsoleNormalColor();
        return 1;
    }
    std::vector<mrpt::poses::CPose3D> poses;
    poses.reserve(sm.size());
    for (const auto& [pose, sf, twist] : sm)
    {
        poses.push_back(pose->getMeanVal());
    }
    const auto leveled = mp2p_icp_filters::level_poses_from_up(*times, poses, samples);
    printRevisitSplit(*times, poses, leveled);

    if (inputTum)
    {
        // The same samples as the keyframes, integrated on the path's own steps.
        // The keyframes are integrated from the first one, so the path is
        // anchored where both first exist: there it moves as much as they do.
        auto&        path     = *inputTum;
        const double tRef     = std::max(times->front(), path.times.front());
        const auto   kfOffset = positionAt(*times, leveled, tRef) - positionAt(*times, poses, tRef);
        const auto   before   = positionAt(path.times, path.poses, tRef);
        path.poses       = mp2p_icp_filters::level_poses_from_up(path.times, path.poses, samples);
        const auto shift = before + kfOffset - positionAt(path.times, path.poses, tRef);
        for (auto& p : path.poses)
        {
            p.x(p.x() + shift.x);
            p.y(p.y() + shift.y);
            p.z(p.z() + shift.z);
        }
        writeTum(cli->arg_output_tum, path);
    }
    if (estimateOnly)
    {
        return 0;
    }

    double maxShift = 0;
    size_t k        = 0;
    for (auto& [pose, sf, twist] : sm)
    {
        // Rotate the covariance with the attitude, then place the mean:
        mrpt::poses::CPose3DPDFGaussian g;
        g.copyFrom(*pose);
        g.changeCoordinatesReference(mp2p_icp_filters::rotation_aligning_up_to_z(
            mp2p_icp_filters::interpolate_up(samples, (*times)[k])));
        g.mean = leveled[k];
        pose   = std::make_shared<mrpt::poses::CPose3DPDFGaussian>(g);

        maxShift = std::max(maxShift, (leveled[k].translation() - poses[k].translation()).norm());
        k++;
    }
    std::cout << mrpt::format("Keyframes moved up to %.3f m.\n", maxShift);

    const std::string outFile = cli->argCmd.at(2);
    std::cout << "Saving result to: '" << outFile << "'... " << std::endl;
    const bool saveOk = sm.saveToFile(outFile);
    ASSERTMSG_(saveOk, "Error writing output file");

    return 0;
}

// --by-submap: a correction per submap of a partition (`sm-cli partition`),
// blended where submaps overlap, with positions solved on the graph of
// consecutive keyframes and links between keyframes of the same place.
int levelBySubmap(
    mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, bool estimateOnly,
    std::optional<TumPath> inputTum)
{
    const auto part = mp2p_icp_filters::SimplemapPartition::load_from_file(cli->arg_by_submap);
    ASSERTMSG_(
        part.keyframe_count() == sm.size(),
        mrpt::format(
            "--by-submap: the partition has %zu keyframes, the simple-map %zu: it was made for "
            "another map",
            part.keyframe_count(), sm.size()));
    ASSERTMSG_(
        part.map_fingerprint == 0 ||
            part.map_fingerprint == mp2p_icp_filters::simplemap_fingerprint(sm),
        "--by-submap: the keyframe timestamps of the partition and the simple-map differ: it was "
        "made for another map");

    const auto times = keyframeTimes(sm);
    if (inputTum)
    {
        ASSERTMSG_(times, "--input-tum with --by-submap needs sorted timestamps in all keyframes");
        checkTumOverlap(*inputTum, *times, "keyframe");
    }

    const double                                           tStart = mrpt::Clock::nowDouble();
    const auto                                             groups = part.submap_keyframes();
    std::vector<WindowEstimate>                            submaps;
    std::vector<std::optional<mp2p_icp_filters::SubmapUp>> ups(groups.size());
    std::vector<WindowEstimate> original;  // walls and floors, for the scene rule
    std::vector<size_t>         weights;  // keyframes of each submap
    size_t                      used    = 0;
    double                      maxTilt = 0;
    for (size_t s = 0; s < groups.size(); s++)
    {
        const auto&    g = groups[s];
        WindowEstimate w;
        if (g.empty())
        {
            submaps.push_back(w);
            original.push_back(w);
            weights.push_back(0);
            continue;
        }
        w.first = g.front();
        w.last  = g.back();
        if (times)
        {
            w.t0 = (*times)[w.first];
            w.t1 = (*times)[w.last];
        }
        std::cout << "\nSubmap " << (s + 1) << "/" << groups.size() << ": " << g.size()
                  << " keyframes in [" << w.first << ", " << w.last << "]\n";

        mrpt::maps::CSimpleMap sub;
        for (const size_t k : g)
        {
            sub.insert(sm.get(k));
        }
        NormalsSet normals;
        estimateInto(w, sub, pipeline, 0, sub.size() - 1, &normals);
        std::optional<mp2p_icp_filters::EstimateUpResult> est = w.est;
        if (est && est->walls_contradict_flats)
        {
            // Floors that are not level here (stairs, a ramp): the walls alone
            est.reset();
            auto p         = estimateParams();
            p.flats_weight = 0;
            try
            {
                est =
                    mp2p_icp_filters::estimate_up_from_normals(normals.normals, p, &normals.points);
                std::cout << mrpt::format(
                    "The walls contradict the floors: walls alone, tilt %.3f deg, sigma %.3f / "
                    "%.3f deg\n",
                    est->tilt_deg(), est->up_sigmas_deg()[0], est->up_sigmas_deg()[1]);
            }
            catch (const std::exception&)
            {
                std::cout << "The walls contradict the floors, and alone they do not give up.\n";
            }
        }
        original.push_back(w);
        weights.push_back(g.size());

        // Every estimate with a reliable covariance within --max-window-sigma-deg
        // counts, weighted by it; the others follow their neighbors:
        w.usable = false;
        if (est && cli->arg_cluster_size > 0 && !est->cov_reliable)
        {
            std::cout << "Not used: too few groups of normals for a reliable sigma.\n";
        }
        else if (
            est &&
            std::max(est->up_sigmas_deg()[0], est->up_sigmas_deg()[1]) > cli->arg_max_window_sigma)
        {
            std::cout << "Not used: sigma above --max-window-sigma-deg.\n";
        }
        else if (est)
        {
            ups[s]   = mp2p_icp_filters::SubmapUp{est->up, est->up_cov};
            w.usable = true;
            maxTilt  = std::max(maxTilt, est->tilt_deg());
            used++;
        }
        w.est = est;  // what was used, for --windows-out
        submaps.push_back(std::move(w));
    }
    writeWindowsOut(submaps);

    if (slopedScene(original, "keyframes in submaps", &weights))
    {
        return 1;
    }
    if (maxTilt > cli->arg_max_correction_deg)
    {
        setConsoleErrorColor();
        std::cerr << mrpt::format(
            "Error: a submap needs a correction (%.3f deg) above --max-correction-deg (%.3f "
            "deg).\n",
            maxTilt, cli->arg_max_correction_deg);
        setConsoleNormalColor();
        return 1;
    }
    if (used == 0)
    {
        setConsoleErrorColor();
        std::cerr << "Error: no submap has a usable up direction. Nothing was leveled.\n";
        setConsoleNormalColor();
        return 1;
    }
    std::cout << mrpt::format("\nBy submap: %zu of %zu submaps used\n", used, groups.size());
    {
        // Each submap against its own uncertainty, and all of them together
        // (a tilt shared by the whole map adds up over its submaps)
        double          best = 0;
        Eigen::Matrix2d sumI = Eigen::Matrix2d::Zero();
        Eigen::Vector2d sumV = Eigen::Vector2d::Zero();
        for (const auto& u : ups)
        {
            if (u)
            {
                best              = std::max(best, significance(u->up, u->cov));
                const auto [h, I] = upInfo(u->up, u->cov);
                sumI += I;
                sumV += I * h;
            }
        }
        const Eigen::Vector2d mean     = sumI.ldlt().solve(sumV);
        const double          together = std::sqrt(mean.dot(sumI * mean));
        std::cout << mrpt::format(
            "Correction: largest submap %.2f sigma, all submaps together %.2f sigma\n", best,
            together);
        if (const double th = minSignificance(true); std::max(best, together) < th)
        {
            return notSignificant(std::max(best, together), th, "the map is");
        }
    }

    std::vector<mrpt::poses::CPose3D> poses;
    std::vector<mrpt::math::TPoint3D> positions;
    poses.reserve(sm.size());
    positions.reserve(sm.size());
    for (const auto& [pose, sf, twist] : sm)
    {
        poses.push_back(pose->getMeanVal());
        positions.push_back(poses.back().translation());
    }
    const double tSubmaps = mrpt::Clock::nowDouble();

    mp2p_icp_filters::UpFieldParams upParams;
    upParams.step_sigma_deg = cli->arg_up_step_sigma;
    const auto   kfUps      = mp2p_icp_filters::keyframe_ups_from_submaps(part, ups, upParams);
    const double tField     = mrpt::Clock::nowDouble();
    {
        std::vector<double> tilts;
        tilts.reserve(kfUps.size());
        for (const auto& u : kfUps)
        {
            tilts.push_back(mrpt::RAD2DEG(std::acos(std::min(1.0, u.z / u.norm()))));
        }
        std::sort(tilts.begin(), tilts.end());
        std::cout << mrpt::format(
            "Up field: keyframe corrections %.3f / %.3f / %.3f deg (min / median / max)\n",
            tilts.front(), tilts[tilts.size() / 2], tilts.back());
        if (tilts.back() > cli->arg_max_correction_deg)
        {
            setConsoleErrorColor();
            std::cerr << mrpt::format(
                "Error: a keyframe needs a correction (%.3f deg) above --max-correction-deg "
                "(%.3f deg).\n",
                tilts.back(), cli->arg_max_correction_deg);
            setConsoleNormalColor();
            return 1;
        }
    }
    const auto links = mp2p_icp_filters::keyframe_place_links(
        part, positions, cli->arg_max_link_length, cli->arg_link_weight);
    const auto   res    = mp2p_icp_filters::level_poses_by_place(poses, kfUps, links);
    const double tSolve = mrpt::Clock::nowDouble();
    std::cout << mrpt::format(
        "Time: submap estimates %.2f s, up field %.3f s, position solve %.3f s\n",
        tSubmaps - tStart, tField - tSubmaps, tSolve - tField);
    std::cout << mrpt::format(
        "Position solve: %zu links; residual rms/max: consecutive %.4f/%.4f m, links "
        "%.4f/%.4f m\n",
        links.size(), res.sequential_residual_rms, res.sequential_residual_max,
        res.link_residual_rms, res.link_residual_max);
    if (times)
    {
        printRevisitSplit(*times, poses, res.poses);
    }

    if (inputTum)
    {
        auto& path = *inputTum;
        path.poses = mp2p_icp_filters::level_path_by_place(
            path.times, path.poses, *times, poses, res.poses, kfUps);
        writeTum(cli->arg_output_tum, path);
    }
    if (estimateOnly)
    {
        return 0;
    }

    double maxShift = 0;
    size_t k        = 0;
    for (auto& [pose, sf, twist] : sm)
    {
        // Rotate the covariance with the attitude, then place the mean:
        mrpt::poses::CPose3DPDFGaussian g;
        g.copyFrom(*pose);
        g.changeCoordinatesReference(mp2p_icp_filters::rotation_aligning_up_to_z(kfUps[k]));
        g.mean = res.poses[k];
        pose   = std::make_shared<mrpt::poses::CPose3DPDFGaussian>(g);

        maxShift = std::max(maxShift, (res.poses[k].translation() - poses[k].translation()).norm());
        k++;
    }
    std::cout << mrpt::format("Keyframes moved up to %.3f m.\n", maxShift);

    const std::string outFile = cli->argCmd.at(2);
    std::cout << "Saving result to: '" << outFile << "'... " << std::endl;
    const bool saveOk = sm.saveToFile(outFile);
    ASSERTMSG_(saveOk, "Error writing output file");

    return 0;
}

}  // namespace

int commandLevelWalls()
{
    const auto& lstCmds = cli->argCmd;
    if (cli->argHelp)
    {
        return printCommandsLevelWalls(false);
    }
    const bool estimateOnly = cli->arg_estimate_only;
    if (lstCmds.size() != 3 && !(estimateOnly && lstCmds.size() == 2))
    {
        return printCommandsLevelWalls(true);
    }

    const std::string inFile = lstCmds.at(1);
    ASSERTMSG_(
        cli->arg_max_walls_flats >= 0,
        mrpt::format("--max-walls-flats-deg must be >= 0 (got %g)", cli->arg_max_walls_flats));

    // Checked before the map is loaded, so a typo fails at once:
    ASSERTMSG_(
        cli->arg_input_tum.empty() == cli->arg_output_tum.empty(),
        "--input-tum and --output-tum must be given together");
    {
        // No output may overwrite an input or another output
        const auto same = [](const std::string& a, const std::string& b)
        { return !a.empty() && !b.empty() && same_file(a, b); };
        const std::string outMap = estimateOnly ? std::string() : lstCmds.at(2);
        const std::vector<std::pair<std::string, const char*>> inputs = {
            {inFile, "the input simple-map"},
            {cli->arg_input_tum, "--input-tum"},
            {cli->arg_by_submap, "--by-submap"},
            {cli->arg_pipeline, "--pipeline"}};
        const std::vector<std::pair<std::string, const char*>> outputs = {
            {outMap, "the output simple-map"},
            {cli->arg_output_tum, "--output-tum"},
            {cli->arg_windows_out, "--windows-out"}};
        for (size_t i = 0; i < outputs.size(); i++)
        {
            for (const auto& [in, inName] : inputs)
            {
                ASSERTMSG_(
                    !same(outputs[i].first, in),
                    mrpt::format("%s must not be %s", outputs[i].second, inName));
            }
            for (size_t j = i + 1; j < outputs.size(); j++)
            {
                ASSERTMSG_(
                    !same(outputs[i].first, outputs[j].first),
                    mrpt::format("%s must not be %s", outputs[i].second, outputs[j].second));
            }
        }
    }
    std::optional<TumPath> inputTum;
    if (!cli->arg_input_tum.empty())
    {
        inputTum = readTum(cli->arg_input_tum);
    }

    mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(inFile);
    ASSERT_(!sm.empty());

    // Same lazy-load base directory convention as sm2mm:
    if (const auto lazyBaseDir = mrpt::system::fileNameChangeExtension(inFile, "") + "_Images";
        mrpt::system::directoryExists(lazyBaseDir))
    {
        mrpt::io::setLazyLoadPathBase(lazyBaseDir);
        std::cout << "Using lazy-load base dir: " << lazyBaseDir << std::endl;
    }

    mrpt::containers::yaml pipeline;
    if (!cli->arg_pipeline.empty())
    {
        ASSERT_FILE_EXISTS_(cli->arg_pipeline);
        pipeline = mp2p_icp::load_yaml_file(cli->arg_pipeline);
    }
    else
    {
        // Per-keyframe voxel decimation, so the accumulated cloud stays small:
        pipeline = mp2p_icp_filters::decimated_keyframes_pipeline(cli->arg_voxel);
    }

    if ((cli->arg_piecewise > 0 || !cli->arg_by_submap.empty()) && cli->arg_windows > 0)
    {
        std::cout << "Warning: --windows is ignored with --piecewise and --by-submap.\n";
    }
    if (!cli->arg_windows_out.empty() && cli->arg_piecewise <= 0 && cli->arg_windows == 0 &&
        cli->arg_by_submap.empty())
    {
        std::cout << "Warning: --windows-out needs --windows, --piecewise or --by-submap: nothing "
                     "will be written to it.\n";
    }
    if (!cli->arg_by_submap.empty())
    {
        ASSERTMSG_(cli->arg_piecewise <= 0, "--by-submap and --piecewise are mutually exclusive");
        return levelBySubmap(sm, pipeline, estimateOnly, std::move(inputTum));
    }
    if (cli->arg_piecewise > 0)
    {
        return levelPiecewise(sm, pipeline, estimateOnly, std::move(inputTum));
    }

    // Whole run:
    const auto pc = buildAnalysisCloud(sm, pipeline, 0, sm.size() - 1);
    std::cout << "Analysis cloud: " << pc->size() << " points.\n";

    std::vector<mrpt::math::TPoint3Df> points;
    const auto                         normals = normalsFromCloud(*pc, &points);
    std::cout << "Planar normals: " << normals.size() << "\n";

    const auto est = mp2p_icp_filters::estimate_up_from_normals(normals, estimateParams(), &points);
    printEstimate(est);

    const auto R = mp2p_icp_filters::rotation_aligning_up_to_z(est.up);
    std::cout << "Rotation    : " << tfString(R) << "  (sm-cli tf " << inFile << " OUT.simplemap \""
              << tfString(R) << "\")\n";

    // Residual, on the rotated normals:
    {
        std::vector<mrpt::math::TVector3D> rotNormals;
        rotNormals.reserve(normals.size());
        for (const auto& n : normals)
        {
            rotNormals.push_back(R.rotateVector(n));
        }
        const auto res = mp2p_icp_filters::estimate_up_from_normals(rotNormals, estimateParams());
        std::cout << mrpt::format(
            "Residual    : %.4f deg\n"
            "Wall normals elevation RMS: %.3f deg -> %.3f deg\n",
            res.tilt_deg(), wallElevationRmsDeg(normals), wallElevationRmsDeg(rotNormals));
    }

    // Drift diagnostic:
    if (cli->arg_windows > 0)
    {
        const auto                  times  = keyframeTimes(sm);
        const auto                  ranges = timeWindows(sm, cli->arg_windows);
        std::vector<WindowEstimate> windows;
        for (size_t w = 0; w < ranges.size(); w++)
        {
            windows.push_back(estimateWindow(
                sm, pipeline, times, w, ranges.size(), ranges[w].first, ranges[w].second));
        }
        std::cout << "\n";
        writeWindowsOut(windows);
    }

    if (est.tilt_deg() > cli->arg_max_correction_deg)
    {
        setConsoleErrorColor();
        std::cerr << mrpt::format(
            "Error: the estimated correction (%.3f deg) exceeds --max-correction-deg (%.3f deg).\n",
            est.tilt_deg(), cli->arg_max_correction_deg);
        setConsoleNormalColor();
        return 1;
    }
    if (est.walls_contradict_flats)
    {
        // Also with --estimate-only: the printed rotation is not a vertical reference
        setConsoleErrorColor();
        std::cerr << "Error: the walls contradict the floors (--max-walls-flats-deg): sloped "
                     "floors or leaning walls, not a vertical reference. Nothing was leveled.\n";
        setConsoleNormalColor();
        return 1;
    }
    if (const double th = minSignificance(false); th > 0)
    {
        const double sig = significance(est.up, est.up_cov);
        std::cout << mrpt::format("Correction: %.2f sigma\n", sig);
        if (sig < th)
        {
            return notSignificant(sig, th, "the estimate is");
        }
    }

    if (inputTum)
    {
        // One rotation about the map origin, as for the keyframes (any clock):
        auto& path = *inputTum;
        for (auto& p : path.poses)
        {
            p = R + p;
        }
        writeTum(cli->arg_output_tum, path);
    }

    if (estimateOnly)
    {
        return 0;
    }

    // Same as `sm-cli tf`: changes both the mean and the covariance.
    for (auto& [pose, sf, twist] : sm)
    {
        pose->changeCoordinatesReference(R);
    }

    const std::string outFile = lstCmds.at(2);
    std::cout << "Saving result to: '" << outFile << "'... " << std::endl;
    const bool saveOk = sm.saveToFile(outFile);
    ASSERTMSG_(saveOk, "Error writing output file");

    return 0;
}

namespace
{
int printCommandsLevelWalls(bool showErrorMsg)
{
    if (showErrorMsg)
    {
        setConsoleErrorColor();
        std::cerr << "Error: missing or unknown subcommand.\n";
        setConsoleNormalColor();
    }

    fprintf(
        stderr,
        R"XXX(Usage:

    sm-cli level-walls <input.simplemap> <output.simplemap>
        [--pipeline <sm2mm-pipeline.yaml>]  # keyframes to points (default: built-in)
        [--voxel 0.05]                      # analysis cloud voxel size [m]
        [--normal-radius 0.4]               # neighborhood for local normals [m]
        [--wall-nz 0.25] [--flat-nz 0.95]   # |n.up| thresholds for walls / floors
        [--max-correction-deg 5]            # refuse larger corrections
        [--estimate-only]                   # print the rotation, do not write output
        [--windows N]                       # also estimate on N equal time windows
        [--piecewise SECONDS]               # level each time window on its own
        [--by-submap <partition.yaml>]      # level each submap (sm-cli partition) on its own
        [--max-link-length 5.0]             # --by-submap: longest same-place link [m]
        [--link-weight 10]                  # --by-submap: weight of same-place links
        [--up-step-sigma-deg 0.3]           # --by-submap: stiffness of the up field
        [--min-significance K]              # level only if some estimate is K sigma off
        [--walls-only]                      # walls are the only vertical reference
        [--cluster-size 2.0]                # cells grouping normals for the sigmas [m]
        [--max-window-sigma-deg 1.0]        # --piecewise skips less certain windows
        [--max-walls-flats-deg 2.0]         # refuse when walls contradict floors
        [--windows-out <file.txt>]          # one row per window, machine readable
        [--input-tum <in.tum> --output-tum <out.tum>]  # level a dense path too

Corrects a global pitch/roll error of a map by rotating it (about the map origin)
so walls are as vertical, and floors and ceilings as horizontal, as possible.
Only keyframe poses change. Requires a mostly rectilinear structure (indoors,
buildings); do not use it on outdoor or sloped scenes.

With --piecewise, the vertical is estimated on consecutive time windows of that
duration and interpolated in time, and each keyframe is rotated by its own
correction, with positions integrated again so the trajectory stays continuous.
This removes a vertical that drifts during the run, which one rotation cannot.
Windows without a reliable sigma of up, or with one above --max-window-sigma-deg,
are skipped and interpolated over, and a short last window joins the previous one.

With --by-submap, the vertical is estimated on each submap of a partition made
by `sm-cli partition`: keyframes that observe the same place, whenever they were
visited. The up of each keyframe comes from a smooth field over the graph of
shared content: the submap estimates, weighted by their covariances, against
a prior that keyframes seeing the same content have similar ups
(--up-step-sigma-deg), since a surface seen with two different corrections is
drawn twice, apart by their difference times the range.
Positions are then solved by least squares on the graph of consecutive
keyframes plus links between keyframes of the same place, so a place visited
twice is corrected alike, and its floor is not split in two as with a
correction that depends on time. Submaps without a reliable sigma, or with one
above --max-window-sigma-deg, follow their neighbors in the graph; submaps whose
walls contradict their floors (stairs, a ramp) use their walls alone; and the
map is refused if a used submap needs more than --max-correction-deg. Use it instead of --piecewise whenever the run
revisits places. The log reports the residual of the position solve and, as in
--piecewise, how much the height difference of keyframes that revisit a place
changed. A map that is already level is left alone: with --by-submap, unless
a submap, or all of them together, is at least --min-significance (default 2)
sigmas off vertical, counting a 0.1 deg systematic error of the references,
nothing is leveled and the command exits with an error. A tilt shared by the
whole map adds up over its submaps. The whole-map mode does the same only when
--min-significance is given.

Walls are checked against floors: where the walls alone and the floors alone
give up directions more than --max-walls-flats-deg apart (floors on a slope,
walls that lean, e.g. an open quarry) they contradict each other. The
whole-map command then refuses to level (and exits with an error, also with
--estimate-only); --piecewise and --by-submap refuse the map when that
happens in four or more of every five windows (submaps) with an estimate.
Windows whose walls alone do not give up (one wall direction) are reported
but never refused.

--input-tum levels a trajectory (e.g. the dense path of the run, in the same
clock as the keyframes) with the same correction as the keyframes, into
--output-tum. It is written with --estimate-only too.

With --pipeline, all point layers the pipeline outputs are merged and voxel
decimated with --voxel.

)XXX");

    return showErrorMsg ? 1 : 0;
}
}  // namespace
