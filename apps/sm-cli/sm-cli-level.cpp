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

static int printCommandsLevel(bool showErrorMsg);

int commandLevel()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsLevel(false);
    if (lstCmds.size() != 3) return printCommandsLevel(true);

    // Take second unlabeled argument:
    const std::string inFile  = lstCmds.at(1);
    const std::string outFile = lstCmds.at(2);

    mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(inFile);

    ASSERT_(!sm.empty());

    std::vector<mrpt::poses::CPose3D> poses;  // all SF KeyFrame poses
    poses.reserve(sm.size());

    for (const auto& [pose, sf, twist] : sm)
    {
        const auto p = pose->getMeanVal();
        poses.push_back(p);
    }

    // Optimize them such as the vertical variation is minimized:

    // The error function F(x):
    auto myCostFunction = [&](const mrpt::math::CVectorDouble&                  x,
                              [[maybe_unused]] const mrpt::math::CVectorDouble& params,
                              mrpt::math::CVectorDouble&                        err)
    {
        const auto delta = mrpt::poses::CPose3D::FromYawPitchRoll(x[0], x[1], x[2]);

        const double z0 = poses.at(0).translation().z;

        // cost=non-horizontality after transformation:
        err.resize(0);
        std::transform(
            poses.cbegin(), poses.cend(), std::back_inserter(err),
            [&delta, z0](const mrpt::poses::CPose3D& p)
            {
                auto newPose = delta + p;
                return newPose.translation().z - z0;
            });
    };

    mrpt::math::CVectorDouble optimal_x, initial_x, params;

    mrpt::math::CLevenbergMarquardt::TResultInfo info;

    // x: [yaw pitch roll]
    initial_x.assign(3U, 0.0);

    const auto increments_x = mrpt::math::CVectorDouble::Constant(3, 1, 1e-6);

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli->arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli->arg_verbosity_level.getValue());
    }

    mrpt::math::CLevenbergMarquardt lm;
    lm.execute(optimal_x, initial_x, myCostFunction, increments_x, params, info, logLevel);

    std::cout << "Iterations: " << info.iterations_executed << std::endl;
    std::cout << "Squared error (initial->final): " << info.initial_sqr_err << " => "
              << info.final_sqr_err << std::endl;

    const auto delta =
        mrpt::poses::CPose3D::FromYawPitchRoll(optimal_x[0], optimal_x[1], optimal_x[2]);
    std::cout << "Final optimized rotation: " << delta << std::endl;

    // Modify KFs:
    for (auto& [pose, sf, twist] : sm)
    {
        const auto p = pose->getMeanVal();
        // This changes both, the mean and the covariance:
        pose->changeCoordinatesReference(delta);
    }

    // save:
    std::cout << "Saving result to: '" << outFile << "... " << std::endl;
    sm.saveToFile(outFile);

    return 0;
}

int printCommandsLevel(bool showErrorMsg)
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

    sm-cli level <input.simplemap> <output.simplemap>

)XXX");

    return showErrorMsg ? 1 : 0;
}
