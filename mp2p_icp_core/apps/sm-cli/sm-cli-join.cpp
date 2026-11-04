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

static int printCommandsJoin(bool showErrorMsg);

int commandJoin()
{
    const auto& lstCmds = cli->argCmd;
    if (cli->argHelp)
    {
        return printCommandsJoin(false);
    }
    if (lstCmds.size() < 2 || cli->opt_output->count() == 0)
    {
        return printCommandsJoin(true);
    }

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

    const auto outFil = cli->arg_output;

    std::cout << "Writing merged simplemap with " << outSM.size() << " keyframes to '" << outFil
              << "'" << std::endl;

    const bool saveOk = outSM.saveToFile(outFil);
    ASSERTMSG_(saveOk, "Error writing output file");

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
