/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   sm-cli-main.cpp
 * @brief  CLI tool to edit, visualize, and get stats from simplemap files.
 * @author Jose Luis Blanco Claraco
 * @date   Feb 7 , 2024
 */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // consoleColorAndStyle()

// register, for read_input_sm_from_cli()
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/obs/registerAllClasses.h>

#include <iostream>
#include <map>

#include "sm-cli.h"

// ======= Command handlers =======
std::unique_ptr<cli_flags> cli;

const std::map<std::string, cmd_t> cliCommands = {
    {"help", cmd_t(&printListCommands)},
    {"info", cmd_t(&commandInfo)},
    {"export-keyframes", cmd_t(&commandExportKF)},
    {"export-rawlog", cmd_t(&commandExportRawlog)},
    {"cut", cmd_t(&commandCut)},
    {"level", cmd_t(&commandLevel)},
    {"trim", cmd_t(&commandTrim)},
};

void setConsoleErrorColor()
{
    mrpt::system::consoleColorAndStyle(
        mrpt::system::ConsoleForegroundColor::RED);
}

void setConsoleNormalColor()
{
    mrpt::system::consoleColorAndStyle(
        mrpt::system::ConsoleForegroundColor::DEFAULT);
}

int main(int argc, char** argv)
{
    try
    {
        cli = std::make_unique<cli_flags>();

        if (!cli->cmd.parse(argc, argv))
        {
            printListCommands();
            return 1;
        }

        if (cli->argVersion.isSet())
        {
            printVersion();
            return 0;
        }

        // Take first unlabeled argument:
        std::string command;
        if (const auto& lst = cli->argCmd.getValue(); !lst.empty())
            command = lst.at(0);

        // Look up command in table:
        auto itCmd = cliCommands.find(command);

        if (!cli->argCmd.isSet() || itCmd == cliCommands.end())
        {
            if (!cli->argHelp.isSet())
            {
                setConsoleErrorColor();
                std::cerr << "Error: missing or unknown command.\n";
                setConsoleNormalColor();
            }
            printListCommands();
            return 1;
        }

        // Execute command:
        return (itCmd->second)();
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}

int printListCommands()
{
    fprintf(
        stderr,
        R"XXX(sm-cli v%s: CLI tool to manipulate and inspect 'simplemap's.

Available commands:
    sm-cli cut                Cut part of a .simplemap file into a new file.
    sm-cli export-keyframes   Export KF poses (opt: twist too) as TUM format.
    sm-cli export-rawlog      Export KFs as rawlog for inspection.
    sm-cli info               Analyze a .simplemap file.
    sm-cli level              Makes a .simplemap file level (horizontal).
    sm-cli trim               Extracts part of a .simplemap inside a given box.
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

Or use `sm-cli <COMMAND> --help` for further options
)XXX",
        MP2P_ICP_VERSION);
    return 0;
}

void printVersion()
{
    std::cout << "sm-cli v" << MP2P_ICP_VERSION << std::endl;
}

// Common part of most commands:
mrpt::maps::CSimpleMap read_input_sm_from_cli(const std::string& inFile)
{
    ASSERT_FILE_EXISTS_(inFile);

    const auto sizeBytes = mrpt::system::getFileSize(inFile);

    std::cout << "Loading: '" << inFile << "' of "
              << mrpt::system::unitsFormat(sizeBytes) << "B..." << std::endl;

    // register mrpt-obs classes, since we are not using them explicitly and
    // hence they are not auto-loading.
    mrpt::maps::registerAllClasses_mrpt_maps();
    mrpt::obs::registerAllClasses_mrpt_obs();

    mrpt::maps::CSimpleMap sm;

    bool loadOk = sm.loadFromFile(inFile);
    ASSERT_(loadOk);

    return sm;
}
