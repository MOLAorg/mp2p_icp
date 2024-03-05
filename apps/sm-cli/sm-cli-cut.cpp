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

static int printCommandsCut(bool showErrorMsg);

int commandCut()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsCut(false);
    if (lstCmds.size() != 2 || !cli->arg_from.isSet() || !cli->arg_to.isSet() ||
        !cli->arg_output.isSet())
        return printCommandsCut(true);

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    const mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(file);

    const auto idxFirst = cli->arg_from.getValue();
    const auto idxLast  = cli->arg_to.getValue();

    ASSERT_LT_(idxFirst, sm.size());
    ASSERT_LT_(idxLast, sm.size());

    mrpt::maps::CSimpleMap outSM;

    for (size_t i = idxFirst; i <= idxLast; i++)  //
        outSM.insert(sm.get(i));

    const auto outFil = cli->arg_output.getValue();

    std::cout << "Writing cut simplemap with " << outSM.size()
              << " keyframes to '" << outFil << "'" << std::endl;

    outSM.saveToFile(outFil);

    return 0;
}

int printCommandsCut(bool showErrorMsg)
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

    sm-cli cut <filename> --from <FIRST_KF_INDEX> --to <LAST_KF_INDEX> --output <OUTPUT.simplemap>

)XXX");

    return showErrorMsg ? 1 : 0;
}
