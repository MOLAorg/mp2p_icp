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

#pragma once

#include <mrpt/maps/CSimpleMap.h>

#include <CLI/CLI.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// We need all CLI11 options to be initialized in order for all translation
// units, that is why we use this holder structure:
struct cli_flags
{
    CLI::App cmd{"sm-cli"};

    // "command" positional catch-all: element 0 is the subcommand name
    // (cut/info/level/...), the rest are that subcommand's own positional
    // arguments (input/output filenames, etc.).
    std::vector<std::string> argCmd;

    std::string arg_verbosity_level;
    size_t      arg_from = 0;
    size_t      arg_to   = 0;
    std::string arg_min_corner;
    std::string arg_max_corner;
    std::string arg_output = "output";
    std::string arg_output_twist;
    bool        argHelp    = false;
    bool        argVersion = false;

    // level-walls:
    std::string arg_pipeline;
    double      arg_voxel              = 0.05;
    double      arg_normal_radius      = 0.4;
    double      arg_wall_nz            = 0.25;
    double      arg_flat_nz            = 0.95;
    double      arg_max_correction_deg = 5.0;
    bool        arg_estimate_only      = false;
    size_t      arg_windows            = 0;
    double      arg_piecewise          = 0;
    bool        arg_walls_only         = false;
    double      arg_cluster_size       = 2.0;
    double      arg_max_window_sigma   = 1.0;
    double      arg_max_walls_flats    = 2.0;
    std::string arg_windows_out;
    std::string arg_input_tum;
    std::string arg_output_tum;
    std::string arg_by_submap;
    double      arg_max_link_length = 5.0;
    double      arg_link_weight     = 10.0;
    double      arg_up_step_sigma   = 1.0;

    // partition:
    double      arg_sample_distance    = 2.0;
    double      arg_sample_angle_deg   = 30.0;
    double      arg_overlap_voxel      = 0.5;
    double      arg_overlap_max_range  = 0;
    double      arg_overlap_decimation = 0.1;
    double      arg_max_ncut           = 0.2;
    size_t      arg_min_submap_nodes   = 5;
    std::string arg_output_colored_map;

    // Option handles, kept to reproduce TCLAP's isSet() semantics exactly
    // (distinguishing "not passed" from "passed with the default value").
    CLI::Option* opt_from         = nullptr;
    CLI::Option* opt_to           = nullptr;
    CLI::Option* opt_min_corner   = nullptr;
    CLI::Option* opt_max_corner   = nullptr;
    CLI::Option* opt_output       = nullptr;
    CLI::Option* opt_output_twist = nullptr;
    CLI::Option* opt_verbosity    = nullptr;

    cli_flags()
    {
        // Custom -h/--help and --version handling below (mirrors "no
        // --help"/no auto-version in the former TCLAP::CmdLine setup):
        cmd.set_help_flag();

        cmd.add_option("command", argCmd, "Command to run. Run 'sm help' to list commands.");

        opt_verbosity = cmd.add_option(
            "-v,--verbosity", arg_verbosity_level,
            "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");

        opt_from = cmd.add_option("--from", arg_from, "First KF index");
        opt_to   = cmd.add_option("--to", arg_to, "Last KF index");

        opt_min_corner =
            cmd.add_option("--min-corner", arg_min_corner, "Bounding box minimum coordinates");
        opt_max_corner =
            cmd.add_option("--max-corner", arg_max_corner, "Bounding box maximum coordinates");

        opt_output = cmd.add_option("-o,--output", arg_output, "Output file");

        opt_output_twist = cmd.add_option(
            "--output-twist", arg_output_twist,
            "Output file for twist (linear and angular velocity). Output file will "
            "contain one row per frame, with these fields: "
            "'time vx vy vz wx wy wz'");

        cmd.add_option(
            "--pipeline", arg_pipeline,
            "level-walls: sm2mm-like YAML pipeline to convert keyframes into points "
            "(default: built-in voxel decimation)");
        cmd.add_option("--voxel", arg_voxel, "level-walls: voxel size of the analysis cloud [m]");
        cmd.add_option(
            "--normal-radius", arg_normal_radius,
            "level-walls: neighborhood radius for local normals [m]");
        cmd.add_option("--wall-nz", arg_wall_nz, "level-walls: |n.up| below this is a wall");
        cmd.add_option(
            "--flat-nz", arg_flat_nz, "level-walls: |n.up| above this is a floor or ceiling");
        cmd.add_option(
            "--max-correction-deg", arg_max_correction_deg,
            "level-walls: refuse corrections larger than this [deg]");
        cmd.add_flag(
            "--estimate-only", arg_estimate_only,
            "level-walls: only print the estimated rotation, do not write the output");
        cmd.add_option(
            "--windows", arg_windows,
            "level-walls: also report the estimate on N equal time windows");
        cmd.add_option(
            "--piecewise", arg_piecewise,
            "level-walls: level each time window of this duration [s] on its own, "
            "interpolating between windows, instead of one rotation of the whole map");
        cmd.add_flag(
            "--walls-only", arg_walls_only,
            "level-walls: use only the walls as vertical reference, not floors and ceilings");
        cmd.add_option(
            "--cluster-size", arg_cluster_size,
            "level-walls: cell size grouping the normals for the uncertainty of up [m]");
        cmd.add_option(
            "--max-window-sigma-deg", arg_max_window_sigma,
            "level-walls: with --piecewise, skip windows whose up is less certain than this "
            "[deg]");
        cmd.add_option(
            "--max-walls-flats-deg", arg_max_walls_flats,
            "level-walls: walls and floors, each on its own, more than this apart on up [deg] "
            "contradict each other (sloped floors, leaning walls): the whole map is then not "
            "leveled, and --piecewise refuses when 4 of 5 windows contradict; 0 disables");
        cmd.add_option(
            "--windows-out", arg_windows_out,
            "level-walls: write one row per time window (--windows or --piecewise) to this "
            "file");
        cmd.add_option(
            "--input-tum", arg_input_tum,
            "level-walls: a trajectory (TUM format, same clock as the keyframes) to level with "
            "the same correction as the keyframes, e.g. the dense path of the run");
        cmd.add_option(
            "--output-tum", arg_output_tum, "level-walls: where to save the leveled --input-tum");
        cmd.add_option(
            "--by-submap", arg_by_submap,
            "level-walls: level each submap of this partition file (from `sm-cli partition`) "
            "on its own, with a correction that depends on place, not on time");
        cmd.add_option(
            "--max-link-length", arg_max_link_length,
            "level-walls: with --by-submap, longest link between keyframes that observe the "
            "same place [m]");
        cmd.add_option(
            "--link-weight", arg_link_weight,
            "level-walls: with --by-submap, weight of those links against consecutive keyframes");
        cmd.add_option(
            "--up-step-sigma-deg", arg_up_step_sigma,
            "level-walls: with --by-submap, stiffness of the up field: how much up may change "
            "between two keyframes that see the same content [deg]");

        cmd.add_option(
            "--sample-distance", arg_sample_distance,
            "partition: distance between sampled keyframes (graph nodes) [m]");
        cmd.add_option(
            "--sample-angle-deg", arg_sample_angle_deg,
            "partition: rotation between sampled keyframes (graph nodes) [deg]");
        cmd.add_option(
            "--overlap-voxel", arg_overlap_voxel,
            "partition: voxel size of the shared content between keyframes [m]");
        cmd.add_option(
            "--overlap-max-range", arg_overlap_max_range,
            "partition: ignore points farther than this from each keyframe [m] (0: no limit)");
        cmd.add_option(
            "--overlap-decimation", arg_overlap_decimation,
            "partition: voxel decimation of each keyframe before its content is compared [m]");
        cmd.add_option(
            "--max-ncut", arg_max_ncut,
            "partition: split a group of keyframes only if the normalized cut is at most this");
        cmd.add_option(
            "--min-submap-nodes", arg_min_submap_nodes,
            "partition: fewest sampled keyframes on each side of a split");
        cmd.add_option(
            "--output-colored-map", arg_output_colored_map,
            "partition: also write a metric map (.mm) whose points have a 'submap' field");

        cmd.add_flag("-h,--help", argHelp, "Shows more detailed help for command");
        cmd.add_flag("--version", argVersion, "Shows program version and exits");
    }
};

extern std::unique_ptr<cli_flags> cli;

using cmd_t = std::function<int(void)>;

int  printListCommands();  // "help"
void printVersion();  // "--version"
int  commandCut();  // "cut"
int  commandInfo();  // "info"
int  commandLevel();  // "level"
int  commandLevelWalls();  // "level-walls"
int  commandPartition();  // "partition"
int  commandTrim();  // "trim"
int  commandJoin();  // "join"
int  commandTf();  // "tf"
int  commandExportKF();  // "export-keyframes"
int  commandExportRawlog();  // "export-rawlog"

mrpt::maps::CSimpleMap read_input_sm_from_cli(const std::string& fil);

/// True if both paths name the same file: the same path once resolved, or,
/// for existing files, the same file (hard links included)
bool same_file(const std::string& a, const std::string& b);

void setConsoleErrorColor();
void setConsoleNormalColor();
