/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CSimpleMap.h>

#include <functional>
#include <memory>
#include <string>

// We need all TCLAP objects to be initialized in order for all translation
// units, that is why we use this holder structure:
struct cli_flags
{
    TCLAP::CmdLine cmd{"sm-cli", ' ', "version", false /* no --help */};

    TCLAP::UnlabeledMultiArg<std::string> argCmd{
        "command", "Command to run. Run 'sm help' to list commands.", false, "",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",
        "verbosity",
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
        false,
        "",
        "INFO",
        cmd};

    TCLAP::ValueArg<size_t> arg_from{
        "", "from", "First KF index", false, 0, "KF index", cmd};

    TCLAP::ValueArg<size_t> arg_to{"",         "to", "Last KF index", false, 0,
                                   "KF index", cmd};

    TCLAP::ValueArg<std::string> arg_min_corner{
        "",    "min-corner", "Bounding box minimum coordinates",
        false, "",           "\"[xmin ymin zmin]\"",
        cmd};

    TCLAP::ValueArg<std::string> arg_max_corner{
        "",    "max-corner", "Bounding box maximum coordinates",
        false, "",           "\"[xmax ymax zmax]\"",
        cmd};

    TCLAP::ValueArg<std::string> arg_output{
        "o", "output", "Output file", false, "output", "output", cmd};

    TCLAP::SwitchArg argHelp{
        "h", "help", "Shows more detailed help for command", cmd};

    TCLAP::SwitchArg argVersion{
        "", "version", "Shows program version and exits", cmd};
};

extern std::unique_ptr<cli_flags> cli;

using cmd_t = std::function<int(void)>;

int  printListCommands();  // "help"
void printVersion();  // "--version"
int  commandCut();  // "cut"
int  commandInfo();  // "info"
int  commandLevel();  // "level"
int  commandTrim();  // "trim"
int  commandExportKF();  // "export-keyframes"
int  commandExportRawlog();  // "export-rawlog"

mrpt::maps::CSimpleMap read_input_sm_from_cli(const std::string& fil);

void setConsoleErrorColor();
void setConsoleNormalColor();
