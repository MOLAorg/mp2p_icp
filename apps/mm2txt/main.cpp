/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mm2txt/main.cpp
 * @brief  A CLI tool to export the layers of a metric map (`*.mm`) as CSV/TXT
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/system/filesystem.h>

// CLI flags:
static TCLAP::CmdLine cmd("mm2txt");

static TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", true, "myMap.mm", "myMap.mm",
    cmd);

static TCLAP::MultiArg<std::string> argLayers(
    "l", "layer",
    "Layer to export. If not provided, all will be exported. This argument can "
    "appear several times.",
    false, "layerName", cmd);

void run_mm2txt()
{
    using namespace std::string_literals;

    const auto& filInput = argMapFile.getValue();

    ASSERT_FILE_EXISTS_(argMapFile.getValue());

    std::cout << "[mm-info] Reading input map from: '" << filInput << "'..."
              << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-info] Done read map. Contents:\n"
              << mm.contents_summary() << std::endl;

    std::vector<std::string> layers;
    if (argLayers.isSet())
    {
        // only selected:
        for (const auto& s : argLayers.getValue()) layers.push_back(s);
    }
    else
    {  // all:
        for (const auto& [name, map] : mm.layers) layers.push_back(name);
    }

    const auto baseFilName = mrpt::system::extractFileName(filInput);

    // Export them:
    for (const auto& name : layers)
    {
        const std::string filName = baseFilName + "_"s + name + ".txt"s;

        std::cout << "Exporting layer: '" << name << "' to file '" << filName
                  << "'..." << std::endl;

        ASSERTMSG_(
            mm.layers.count(name) == 1, "Layer not found in metric map!");

        auto* pts = mp2p_icp::MapToPointsMap(*mm.layers.at(name));
        if (!pts)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' is of type '%s' which cannot be converted into a "
                "point cloud for exporting in TXT format.",
                name.c_str(), mm.layers.at(name)->GetRuntimeClass()->className);
        }

        if (auto* xyzirt =
                dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(pts);
            xyzirt)
        {
            xyzirt->saveXYZIRT_to_text_file(filName);
        }
        else if (auto* xyzi =
                     dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(pts);
                 xyzi)
        {
            xyzi->saveXYZI_to_text_file(filName);
        }
        else { pts->save3D_to_text_file(filName); }
    }
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        run_mm2txt();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
