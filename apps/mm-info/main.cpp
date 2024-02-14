/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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
static TCLAP::CmdLine cmd("mm-filter");

static TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", true, "myMap.mm", "myMap.mm",
    cmd);

void run_mm_info()
{
    const auto& filInput = argMapFile.getValue();

    ASSERT_FILE_EXISTS_(argMapFile.getValue());

    std::cout << "[mm-info] Reading input map from: '" << filInput << "'..."
              << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-info] Done read map. Contents:\n"
              << mm.contents_summary() << std::endl;
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
