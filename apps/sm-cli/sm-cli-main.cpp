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
#include <mrpt/system/os.h>  // consoleColorAndStyle()

#include <iostream>
#include <map>

#include "sm-cli.h"

// ======= Command handlers =======
std::unique_ptr<cli_flags> cli;

const std::map<std::string, cmd_t> cliCommands = {
    {"help", cmd_t(&printListCommands)},
    {"info", cmd_t(&commandInfo)},
    {"level", cmd_t(&commandLevel)},
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
    sm-cli info               Analyze a .simplemap file.
    sm-cli level              Makes a .simplemap file level (horizontal).
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

Or use `sm <COMMAND> --help` for further options
)XXX",
        MP2P_ICP_VERSION);
    return 0;
}

void printVersion() { std::cout << "sm v" << MP2P_ICP_VERSION << std::endl; }
