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

    TCLAP::SwitchArg argHelp{
        "h", "help", "Shows more detailed help for command", cmd};

    TCLAP::SwitchArg argVersion{
        "", "version", "Shows program version and exits", cmd};
};

extern std::unique_ptr<cli_flags> cli;

using cmd_t = std::function<int(void)>;

int  printListCommands();  // "help"
void printVersion();  // "--version"
int  commandInfo();  // "info"

void setConsoleErrorColor();
void setConsoleNormalColor();
