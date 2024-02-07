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
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/obs/registerAllClasses.h>
#include <mrpt/system/filesystem.h>

#include "sm-cli.h"

static int printCommandsInfo(bool showErrorMsg);

int commandInfo()
{
    const auto& lstCmds = cli->argCmd.getValue();
    if (cli->argHelp.isSet()) return printCommandsInfo(false);
    if (lstCmds.size() != 2) return printCommandsInfo(true);

    // Take second unlabeled argument:
    const std::string file = lstCmds.at(1);

    ASSERT_FILE_EXISTS_(file);

    const auto sizeBytes = mrpt::system::getFileSize(file);

    std::cout << "Loading: '" << file << "' of "
              << mrpt::system::unitsFormat(sizeBytes) << "B..." << std::endl;

    // register mrpt-obs classes, since we are using them explicitly and hence
    // they are not auto-loading.
    mrpt::maps::registerAllClasses_mrpt_maps();
    mrpt::obs::registerAllClasses_mrpt_obs();

    mrpt::maps::CSimpleMap sm;

    bool loadOk = sm.loadFromFile(file);
    ASSERT_(loadOk);

    // estimate path bounding box:
    auto bbox     = mrpt::math::TBoundingBox::PlusMinusInfinity();
    bool hasTwist = false;
    std::optional<mrpt::Clock::time_point> timeMin, timeMax;

    std::map<std::string, std::string> obsTypes;
    std::map<std::string, size_t>      obsCount;

    for (const auto& [pose, sf, twist] : sm)
    {
        if (twist.has_value()) hasTwist = true;

        const auto p = pose->getMeanVal().asTPose();
        bbox.updateWithPoint(p.translation());

        if (!sf) continue;

        for (const auto& o : *sf)
        {
            ASSERT_(o);
            auto t = o->getTimeStamp();
            if (!timeMin || t < *timeMin) timeMin = t;
            if (!timeMax || t > *timeMax) timeMax = t;

            obsTypes[o->sensorLabel] = o->GetRuntimeClass()->className;
            obsCount[o->sensorLabel]++;
        }
    }

    std::cout << "\n";
    std::cout << "size_bytes:           " << sizeBytes << "\n";
    std::cout << "keyframe_count:       " << sm.size() << "\n";
    std::cout << "has_twist:            " << (hasTwist ? "true" : "false")
              << "\n";
    std::cout << "kf_bounding_box_min:  " << bbox.min.asString() << "\n";
    std::cout << "kf_bounding_box_max:  " << bbox.max.asString() << "\n";
    std::cout << "kf_bounding_box_span: " << (bbox.max - bbox.min).asString()
              << "\n";
    std::cout << "timestamp_first_utc:  "
              << (timeMin ? mrpt::system::dateTimeToString(*timeMin) : "None")
              << "\n";
    std::cout << "timestamp_last_utc:   "
              << (timeMax ? mrpt::system::dateTimeToString(*timeMax) : "None")
              << "\n";
    std::cout << "timestamp_span:       "
              << ((timeMin && timeMax)
                      ? mrpt::system::formatTimeInterval(
                            mrpt::system::timeDifference(*timeMax, *timeMin))
                      : "None")
              << "\n";
    std::cout << "observations:\n";

    for (const auto& [label, type] : obsTypes)
    {
        std::cout << "  - label: '" << label << "'\n";
        std::cout << "    class: '" << type << "'\n";
        std::cout << "    count: " << obsCount[label] << "\n";
    }

    return 0;
}

int printCommandsInfo(bool showErrorMsg)
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

    sm-cli info <filename>  Loads and analyze the given map

)XXX");

    return showErrorMsg ? 1 : 0;
}
