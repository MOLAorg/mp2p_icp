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
#include <mrpt/math/CLevenbergMarquardt.h>

#include <iostream>

#include "sm-cli.h"

static int printCommandsTf(bool showErrorMsg);

int commandTf()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsTf(false);
    if (lstCmds.size() != 4) return printCommandsTf(true);

    // Take second unlabeled argument:
    const std::string inFile  = lstCmds.at(1);
    const std::string outFile = lstCmds.at(2);
    const std::string strTf   = lstCmds.at(3);

    mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(inFile);

    ASSERT_(!sm.empty());

    const auto tf = mrpt::poses::CPose3D::FromString(strTf);
    std::cout << "tf to apply: " << tf << "\n";

    // Modify KFs:
    for (auto& [pose, sf, twist] : sm)
    {
        const auto p = pose->getMeanVal();
        // This changes both, the mean and the covariance:
        pose->changeCoordinatesReference(tf);
    }

    // save:
    std::cout << "Saving result to: '" << outFile << "... " << std::endl;
    sm.saveToFile(outFile);

    return 0;
}

int printCommandsTf(bool showErrorMsg)
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

    sm-cli tf <input.simplemap> <output.simplemap> "[x y z yaw_deg pitch_deg roll_deg]"

)XXX");

    return showErrorMsg ? 1 : 0;
}
