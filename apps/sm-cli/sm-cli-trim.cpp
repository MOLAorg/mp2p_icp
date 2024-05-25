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

static int printCommandsTrim(bool showErrorMsg);

int commandTrim()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsTrim(false);
    if (lstCmds.size() != 2 || !cli->arg_min_corner.isSet() ||
        !cli->arg_max_corner.isSet() || !cli->arg_output.isSet())
        return printCommandsTrim(true);

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    const auto cornerMin =
        mrpt::math::TPoint3D::FromString(cli->arg_min_corner.getValue());
    const auto cornerMax =
        mrpt::math::TPoint3D::FromString(cli->arg_max_corner.getValue());

    const auto bbox = mrpt::math::TBoundingBox(cornerMin, cornerMax);

    const mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(file);

    mrpt::maps::CSimpleMap outSM;

    for (const auto& [posePDF, sf, twist] : sm)
    {
        ASSERT_(posePDF);
        const auto p = posePDF->getMeanVal();

        if (!bbox.containsPoint(p.translation())) continue;

        outSM.insert(posePDF, sf, twist);
    }

    const auto outFil = cli->arg_output.getValue();

    std::cout << "Writing trimmed simplemap with " << outSM.size()
              << " keyframes to '" << outFil << "'" << std::endl;

    outSM.saveToFile(outFil);

    return 0;
}

int printCommandsTrim(bool showErrorMsg)
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

    sm-cli trim <filename> --min-corner "[xmin ymin zmin]" --max-corner "[xmax ymax zmax]" --output <OUTPUT.simplemap>

)XXX");

    return showErrorMsg ? 1 : 0;
}
