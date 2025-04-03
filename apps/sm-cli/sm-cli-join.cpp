/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>

#include "sm-cli.h"

static int printCommandsJoin(bool showErrorMsg);

int commandJoin()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsJoin(false);
    if (lstCmds.size() < 2 || !cli->arg_output.isSet()) return printCommandsJoin(true);

    // Take second unlabeled argument:
    mrpt::maps::CSimpleMap outSM;

    for (size_t i = 1; i < lstCmds.size(); i++)
    {
        const std::string      file = lstCmds.at(i);
        mrpt::maps::CSimpleMap sm   = read_input_sm_from_cli(file);

        for (size_t k = 0; k < sm.size(); k++)
        {
            const auto kf = sm.get(k);
            outSM.insert(kf);
        }
    }

    const auto outFil = cli->arg_output.getValue();

    std::cout << "Writing merged simplemap with " << outSM.size() << " keyframes to '" << outFil
              << "'" << std::endl;

    outSM.saveToFile(outFil);

    return 0;
}

int printCommandsJoin(bool showErrorMsg)
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

    sm-cli join <filename_1> [<filename_2> ...] --output <MERGED.simplemap>

)XXX");

    return showErrorMsg ? 1 : 0;
}
