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
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <sstream>

#include "sm-cli.h"

static int printCommandsExportRawlog(bool showErrorMsg);

int commandExportRawlog()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsExportRawlog(false);
    if (lstCmds.size() != 2 || !cli->arg_output.isSet())
        return printCommandsExportRawlog(true);

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    const mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(file);

    const auto outFil = cli->arg_output.getValue();

    mrpt::obs::CRawlog rawlog;

    for (const auto& [kf, sf, twist] : sm)
    {
        ASSERT_(kf);
        ASSERT_(sf);

        mrpt::obs::CSensoryFrame outSF;
        // regular observations:
        outSF += *sf;

        // plus:
        // 1) Robot pose:
        // 2) Twist:
        {
            auto obsPose = mrpt::obs::CObservationRobotPose::Create();

            obsPose->sensorLabel = "pose";
            obsPose->pose.copyFrom(*kf);
            outSF.insert(obsPose);
        }
        if (twist.has_value())
        {
            auto obsTwist = mrpt::obs::CObservationComment::Create();

            obsTwist->sensorLabel = "twist";
            std::stringstream ss;
            ss << "Twist stored in the simplemap keyframe:\n"
               << twist->asString();

            obsTwist->text = ss.str();
            outSF.insert(obsTwist);
        }

        rawlog.insert(outSF);
    }

    std::cout << "Saving rawlog with " << rawlog.size() << " entries to: '"
              << outFil << "'" << std::endl;

    rawlog.saveToRawLogFile(outFil);

    return 0;
}

int printCommandsExportRawlog(bool showErrorMsg)
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

    sm-cli export-rawlog <filename.simplemap> --output <OUTPUT.tum>

)XXX");

    return showErrorMsg ? 1 : 0;
}
