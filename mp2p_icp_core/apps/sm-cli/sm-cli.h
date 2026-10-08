/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

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
int  commandTrim();  // "trim"
int  commandJoin();  // "join"
int  commandTf();  // "tf"
int  commandExportKF();  // "export-keyframes"
int  commandExportRawlog();  // "export-rawlog"

mrpt::maps::CSimpleMap read_input_sm_from_cli(const std::string& fil);

void setConsoleErrorColor();
void setConsoleNormalColor();
