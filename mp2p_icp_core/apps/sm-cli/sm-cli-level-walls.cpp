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
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <utility>

#include "sm-cli.h"

namespace
{
int printCommandsLevelWalls(bool showErrorMsg);

constexpr const char* ANALYSIS_LAYER = "level_walls_analysis";

// Per-keyframe voxel decimation, so the accumulated cloud stays small:
mrpt::containers::yaml defaultPipeline(double voxel)
{
    return mrpt::containers::yaml::FromText(mrpt::format(
        R"XXX(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: 'raw'
filters:
  - class_name: mp2p_icp_filters::FilterDecimateVoxels
    params:
      input_pointcloud_layer: 'raw'
      output_pointcloud_layer: 'decimated'
      voxel_filter_resolution: %f
      decimate_method: DecimateMethod::FirstPoint
  - class_name: mp2p_icp_filters::FilterDeleteLayer
    params:
      pointcloud_layer_to_remove: ['raw']
)XXX",
        voxel));
}

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
void checkTumOverlap(const TumPath& path, const std::vector<double>& kfTimes)
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
            "the correction of the nearest window is held.\n",
            outside, path.times.size());
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
    try
    {
        const auto                         pc = buildAnalysisCloud(sm, pipeline, first, last);
        std::vector<mrpt::math::TPoint3Df> points;
        const auto                         normals = normalsFromCloud(*pc, &points);
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

// --piecewise: a correction per time window, interpolated between windows.
int levelPiecewise(
    mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, bool estimateOnly,
    std::optional<TumPath> inputTum)
{
    const auto times = keyframeTimes(sm);
    ASSERTMSG_(times, "--piecewise needs sorted timestamps in all keyframes");
    if (inputTum)
    {
        checkTumOverlap(*inputTum, *times);
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

    // A sloped or quarried scene: nearly every window's walls contradict its
    // floors. A few such windows (a ramp, a sloped lawn) are not a reason to
    // stand down, and skipping them only leaves gaps to interpolate over.
    // Windows whose walls cannot tell (one wall direction, along a single
    // facade) are not counted: they say nothing against the floors.
    size_t estimated    = 0;
    size_t contradicted = 0;
    for (const auto& w : windows)
    {
        if (w.est)
        {
            estimated++;
            contradicted += w.est->walls_contradict_flats ? 1 : 0;
        }
    }
    if (estimated > 0 && contradicted * 5 >= estimated * 4)
    {
        setConsoleErrorColor();
        std::cerr << mrpt::format(
            "Error: in %zu of the %zu windows with an estimate the walls contradict the floors "
            "(--max-walls-flats-deg): sloped ground or leaning walls, not a scene of plumb walls "
            "and level floors. Nothing was leveled.\n",
            contradicted, estimated);
        setConsoleNormalColor();
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
        pipeline = defaultPipeline(cli->arg_voxel);
    }

    if (cli->arg_piecewise > 0 && cli->arg_windows > 0)
    {
        std::cout << "Warning: --windows is ignored with --piecewise.\n";
    }
    if (!cli->arg_windows_out.empty() && cli->arg_piecewise <= 0 && cli->arg_windows == 0)
    {
        std::cout << "Warning: --windows-out needs --windows or --piecewise: nothing will be "
                     "written to it.\n";
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

Walls are checked against floors: where the walls alone and the floors alone
give up directions more than --max-walls-flats-deg apart (floors on a slope,
walls that lean, e.g. an open quarry) they contradict each other. The
whole-map command then refuses to level (and exits with an error, also with
--estimate-only); --piecewise refuses the map when that happens in four or
more of every five windows with an estimate. Windows whose walls alone do not
give up (one wall direction) are reported but never refused.

--input-tum levels a trajectory (e.g. the dense path of the run, in the same
clock as the keyframes) with the same correction as the keyframes, into
--output-tum. It is written with --estimate-only too.

With --pipeline, all point layers the pipeline outputs are merged and voxel
decimated with --voxel.

)XXX");

    return showErrorMsg ? 1 : 0;
}
}  // namespace
