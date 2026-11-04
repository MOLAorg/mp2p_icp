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

#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>

#include "sm-cli.h"

static int printCommandsCut(bool showErrorMsg);

int commandCut()
{
    const auto& lstCmds = cli->argCmd;
    if (cli->argHelp)
    {
        return printCommandsCut(false);
    }
    if (lstCmds.size() != 2 || cli->opt_from->count() == 0 || cli->opt_to->count() == 0 ||
        cli->opt_output->count() == 0)
    {
        return printCommandsCut(true);
    }

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    const mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(file);

    const auto idxFirst = cli->arg_from;
    const auto idxLast  = cli->arg_to;

    ASSERT_LT_(idxFirst, sm.size());
    ASSERT_LT_(idxLast, sm.size());

    mrpt::maps::CSimpleMap outSM;

    for (size_t i = idxFirst; i <= idxLast; i++)
    {
        outSM.insert(sm.get(i));
    }

    const auto outFil = cli->arg_output;

    std::cout << "Writing cut simplemap with " << outSM.size() << " keyframes to '" << outFil << "'"
              << std::endl;

    const bool saveOk = outSM.saveToFile(outFil);
    ASSERTMSG_(saveOk, "Error writing output file");

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
