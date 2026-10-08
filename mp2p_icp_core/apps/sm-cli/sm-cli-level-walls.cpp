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
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>

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
    p.wall_nz = cli->arg_wall_nz;
    p.flat_nz = cli->arg_flat_nz;
    return p;
}

std::vector<mrpt::math::TVector3D> normalsFromCloud(const mrpt::maps::CPointsMap& pc)
{
    mp2p_icp_filters::PlanarNormalsParams p;
    p.search_radius = cli->arg_normal_radius;
    return mp2p_icp_filters::estimate_planar_normals(pc, p);
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

// Keyframe index ranges [first, last] of N windows of equal duration, or of
// equal keyframe count if any keyframe lacks a timestamp.
std::vector<std::pair<size_t, size_t>> timeWindows(const mrpt::maps::CSimpleMap& sm, size_t N)
{
    const size_t nKFs = sm.size();

    std::vector<double> times;
    times.reserve(nKFs);
    for (const auto& [pose, sf, twist] : sm)
    {
        const auto t = sf ? keyframeTime(*sf) : std::nullopt;
        if (!t)
        {
            break;
        }
        times.push_back(*t);
    }

    std::vector<size_t> windowOfKF(nKFs);
    // Windows must be contiguous keyframe ranges, so times must be sorted:
    if (times.size() == nKFs && std::is_sorted(times.begin(), times.end()) &&
        times.back() > times.front())
    {
        const double t0 = times.front();
        const double dt = (times.back() - t0) / static_cast<double>(N);
        for (size_t i = 0; i < nKFs; i++)
        {
            windowOfKF[i] = std::min(N - 1, static_cast<size_t>((times[i] - t0) / dt));
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

    std::vector<std::pair<size_t, size_t>> ranges;
    for (size_t i = 0; i < nKFs; i++)
    {
        if (i == 0 || windowOfKF[i] != windowOfKF[i - 1])
        {
            ranges.emplace_back(i, i);
        }
        ranges.back().second = i;
    }
    return ranges;
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
        "Tilt        : %.3f deg\n",
        est.wall_count, est.flat_count, est.up.x, est.up.y, est.up.z, est.tilt_deg());
    if (est.single_wall_direction)
    {
        std::cout << "Warning: all walls share one direction: the estimate relies on the floors "
                     "and ceilings.\n";
    }
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
        pipeline = mrpt::containers::yaml::FromFile(cli->arg_pipeline);
    }
    else
    {
        pipeline = defaultPipeline(cli->arg_voxel);
    }

    // Whole run:
    const auto pc = buildAnalysisCloud(sm, pipeline, 0, sm.size() - 1);
    std::cout << "Analysis cloud: " << pc->size() << " points.\n";

    const auto normals = normalsFromCloud(*pc);
    std::cout << "Planar normals: " << normals.size() << "\n";

    const auto est = mp2p_icp_filters::estimate_up_from_normals(normals, estimateParams());
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
        const auto ranges = timeWindows(sm, cli->arg_windows);
        for (size_t w = 0; w < ranges.size(); w++)
        {
            const auto [first, last] = ranges[w];
            std::cout << "\nWindow " << (w + 1) << "/" << ranges.size() << ": keyframes [" << first
                      << ", " << last << "]\n";
            try
            {
                const auto wPc  = buildAnalysisCloud(sm, pipeline, first, last);
                const auto wEst = mp2p_icp_filters::estimate_up_from_normals(
                    normalsFromCloud(*wPc), estimateParams());
                printEstimate(wEst);
            }
            catch (const std::exception& e)
            {
                std::cout << "Could not estimate: " << mrpt::exception_to_str(e) << "\n";
            }
        }
        std::cout << "\n";
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

Corrects a global pitch/roll error of a map by rotating it (about the map origin)
so walls are as vertical, and floors and ceilings as horizontal, as possible.
Only keyframe poses change. Requires a mostly rectilinear structure (indoors,
buildings); do not use it on outdoor or sloped scenes.

With --pipeline, all point layers the pipeline outputs are merged and voxel
decimated with --voxel.

)XXX");

    return showErrorMsg ? 1 : 0;
}
}  // namespace
