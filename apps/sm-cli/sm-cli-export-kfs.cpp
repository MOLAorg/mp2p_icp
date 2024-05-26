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
#include <mrpt/poses/CPose3DInterpolator.h>

#include <fstream>

#include "sm-cli.h"

static int printCommandsExportKF(bool showErrorMsg);

int commandExportKF()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsExportKF(false);
    if (lstCmds.size() != 2 || !cli->arg_output.isSet())
        return printCommandsExportKF(true);

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    const mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(file);

    const auto outFil      = cli->arg_output.getValue();
    const auto outTwistFil = cli->arg_output_twist.getValue();

    std::optional<std::ofstream> outTwist;
    if (!outTwistFil.empty())
    {
        outTwist.emplace(outTwistFil, std::ofstream::out);
    }

    mrpt::poses::CPose3DInterpolator kfs;

    size_t kfsWithoutTimestamp = 0;

    for (const auto& [kf, sf, twist] : sm)
    {
        const auto                             pose = kf->getMeanVal();
        std::optional<mrpt::Clock::time_point> tim;

        ASSERT_(sf);
        for (const auto& obs : *sf)
        {
            if (!obs) continue;
            auto t = obs->getTimeStamp();
            if (t == mrpt::system::InvalidTimeStamp()) continue;
            tim = t;
            break;
        }
        if (!tim)
        {
            ++kfsWithoutTimestamp;
            continue;
        }

        kfs.insert(*tim, pose);

        if (outTwist && twist)
        {
            const auto& tw = *twist;
            *outTwist << mrpt::format(
                "%.06f %f %f %f %f %f %f\n",  //
                mrpt::Clock::toDouble(*tim),  // time
                tw.vx, tw.vy, tw.vz,  // linear vel
                tw.wx, tw.wy, tw.wz  // angular vel
            );
        }
    }

    if (kfsWithoutTimestamp)
    {
        std::cerr << "Warning: " << kfsWithoutTimestamp
                  << " keyframes cannot be exported since they lack "
                     "observations with timestamp."
                  << std::endl;
    }

    std::cout << "Saving " << kfs.size() << " keyframe poses to: '" << outFil
              << "'" << std::endl;

    kfs.saveToTextFile_TUM(outFil);

    return 0;
}

int printCommandsExportKF(bool showErrorMsg)
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

    sm-cli export-keyframes <filename> --output <OUTPUT.tum> [--output-twist <TWIST.txt>]

)XXX");

    return showErrorMsg ? 1 : 0;
}
