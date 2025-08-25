/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   mm-info/main.cpp
 * @brief  CLI tool to read a map and describe it
 * @author Jose Luis Blanco Claraco
 * @date   Feb 13, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/system/filesystem.h>

// CLI flags:
namespace
{
TCLAP::CmdLine cmd("mm-filter");

TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", true, "myMap.mm", "myMap.mm", cmd);
}  // namespace

void run_mm_info()
{
    const auto& filInput = argMapFile.getValue();

    ASSERT_FILE_EXISTS_(argMapFile.getValue());

    std::cout << "[mm-info] Reading input map from: '" << filInput << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-info] Done read map. Contents:\n" << mm.contents_summary() << std::endl;
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        run_mm_info();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
