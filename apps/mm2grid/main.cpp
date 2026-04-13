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

/**
 * @file   mm2grid/main.cpp
 * @brief  CLI tool to export a COccupancyGridMap2D layer from a metric map
 *         (*.mm) as a PNG image + ROS map_server / nav2_map_server YAML file.
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

// ---------------------------------------------------------------------------
// CLI flags
// ---------------------------------------------------------------------------
namespace
{
TCLAP::CmdLine cmd("mm2grid");

TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", true, "myMap.mm", "myMap.mm", cmd);

TCLAP::ValueArg<std::string> argLayer(
    "l", "layer",
    "Name of the layer to export (must be of type COccupancyGridMap2D). "
    "If not provided and the map contains exactly one occupancy-grid layer, "
    "that one will be used automatically.",
    false, "", "layerName", cmd);

TCLAP::ValueArg<std::string> argOutput(
    "o", "output",
    "Base name (without extension) for the output files. "
    "Two files will be created: <output>.png and <output>.yaml. "
    "Defaults to <input_basename>[_<layerName>].",
    false, "", "output_base", cmd);

TCLAP::ValueArg<double> argOccupiedThresh(
    "", "occupied-thresh",
    "Pixels with occupancy probability greater than this value will be "
    "considered occupied (black). Default: 0.65",
    false, 0.65, "0.65", cmd);

TCLAP::ValueArg<double> argFreeThresh(
    "", "free-thresh",
    "Pixels with occupancy probability less than this value will be "
    "considered free (white). Default: 0.196",
    false, 0.196, "0.196", cmd);

TCLAP::ValueArg<std::string> argMode(
    "", "mode",
    "Interpretation mode written into the YAML: 'trinary' (default), "
    "'scale', or 'raw'. See ROS map_server docs.",
    false, "trinary", "trinary|scale|raw", cmd);

TCLAP::SwitchArg argNegate(
    "", "negate",
    "If set, black in the image will be interpreted as free and white as "
    "occupied (negate: true in the YAML).",
    cmd);

TCLAP::ValueArg<std::string> arg_plugins(
    "", "load-plugins", "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

}  // namespace

// ---------------------------------------------------------------------------

void run_mm2grid()
{
    using namespace std::string_literals;

    // Load plugins:
    if (arg_plugins.isSet())
    {
        std::string errMsg;
        const auto  plugins = arg_plugins.getValue();
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
        {
            throw std::runtime_error(errMsg);
        }
    }

    const auto& filInput = argMapFile.getValue();
    ASSERT_FILE_EXISTS_(filInput);

    std::cout << "[mm2grid] Reading input map from: '" << filInput << "'...\n";
    const double t0 = mrpt::Clock::nowDouble();

    mp2p_icp::metric_map_t mm;

    const bool loadOk = mm.load_from_file(filInput);
    if (!loadOk)
    {
        THROW_EXCEPTION_FMT("Error loading input map file: '%s'", filInput.c_str());
    }

    const double t1 = mrpt::Clock::nowDouble();
    std::cout << "[mm2grid] Done in " << (t1 - t0) << " s. Contents:\n"
              << mm.contents_summary() << "\n";

    // ------------------------------------------------------------------
    // Locate the target layer
    // ------------------------------------------------------------------
    std::string layerName;

    if (argLayer.isSet())
    {
        layerName = argLayer.getValue();
        ASSERTMSG_(
            mm.layers.count(layerName) != 0,
            mrpt::format("Layer '%s' not found in metric map.", layerName.c_str()));
    }
    else
    {
        // Auto-detect: find the first (and only) COccupancyGridMap2D layer.
        std::vector<std::string> gridLayers;
        for (const auto& [name, map] : mm.layers)
        {
            if (dynamic_cast<const mrpt::maps::COccupancyGridMap2D*>(map.get()))
            {
                gridLayers.push_back(name);
            }
        }

        if (gridLayers.empty())
        {
            THROW_EXCEPTION(
                "No COccupancyGridMap2D layer found in the metric map. "
                "Use --layer to specify the correct layer name.");
        }

        if (gridLayers.size() > 1)
        {
            THROW_EXCEPTION_FMT(
                "More than one COccupancyGridMap2D layer found (%zu). "
                "Please use --layer to pick one.",
                gridLayers.size());
        }

        layerName = gridLayers.front();
        std::cout << "[mm2grid] Auto-selected layer: '" << layerName << "'\n";
    }

    // ------------------------------------------------------------------
    // Cast to COccupancyGridMap2D
    // ------------------------------------------------------------------
    const auto* gridPtr =
        dynamic_cast<const mrpt::maps::COccupancyGridMap2D*>(mm.layers.at(layerName).get());

    ASSERTMSG_(
        gridPtr != nullptr,
        mrpt::format(
            "Layer '%s' is of type '%s', not COccupancyGridMap2D.", layerName.c_str(),
            mm.layers.at(layerName)->GetRuntimeClass()->className));

    const mrpt::maps::COccupancyGridMap2D& grid = *gridPtr;

    // ------------------------------------------------------------------
    // Build output file base name
    // ------------------------------------------------------------------
    std::string outBase;
    if (argOutput.isSet())
    {
        outBase = argOutput.getValue();
    }
    else
    {
        const std::string baseName = mrpt::system::extractFileName(filInput);
        outBase                    = baseName + "_" + layerName;
    }

    const std::string pngFile  = outBase + ".png";
    const std::string yamlFile = outBase + ".yaml";

    // ------------------------------------------------------------------
    // Save the occupancy grid as a PNG image.
    //
    // ROS map_server convention (trinary / scale modes):
    //   - White  (255) = free
    //   - Black  (  0) = occupied
    //   - Gray  (205 by default) = unknown
    //
    // COccupancyGridMap2D::getAsImage() produces a grayscale image where
    //   - White  = free   (probability close to 0)
    //   - Black  = occupied (probability close to 1)
    //   - Gray   = unknown (probability ~0.5)
    // which is exactly the convention map_server expects (when negate=false).
    //
    // The image origin is the BOTTOM-LEFT corner of the map (verticalFlip=false
    // means row-0 of the image corresponds to the lowest Y in the map), which
    // matches what map_server expects when "origin" lists the bottom-left world
    // coordinate.
    // ------------------------------------------------------------------
    {
        mrpt::img::CImage img;
        // verticalFlip=false  → row 0 == bottom of map (correct for ROS)
        // forceRGB=false      → grayscale, smaller file
        // tricolor=false      → full gray-scale gradient, not just 3 levels
        grid.getAsImage(img, /*verticalFlip=*/false, /*forceRGB=*/false, /*tricolor=*/false);

        std::cout << "[mm2grid] Saving PNG image to '" << pngFile << "'...\n";
        if (!img.saveToFile(pngFile))
        {
            THROW_EXCEPTION_FMT("Could not save image to '%s'", pngFile.c_str());
        }
    }

    // ------------------------------------------------------------------
    // Build and save the ROS map_server YAML
    //
    // Reference format (nav2_map_server / map_server):
    //
    //   image: map.png
    //   resolution: 0.050000
    //   origin: [-10.000000, -10.000000, 0.000000]
    //   negate: 0
    //   occupied_thresh: 0.65
    //   free_thresh: 0.196
    //   mode: trinary          # optional, nav2_map_server only
    //
    // "origin" is the real-world pose (x, y, yaw) of the BOTTOM-LEFT pixel.
    // ------------------------------------------------------------------
    {
        // Validate mode value
        const std::string modeStr = argMode.getValue();
        if (modeStr != "trinary" && modeStr != "scale" && modeStr != "raw")
        {
            THROW_EXCEPTION_FMT(
                "Invalid --mode value '%s'. Must be 'trinary', 'scale', or 'raw'.",
                modeStr.c_str());
        }

        mrpt::containers::yaml doc = mrpt::containers::yaml::Map();

        // Use just the filename part so the YAML is relocatable alongside the PNG.
        doc["image"]      = mrpt::system::extractFileName(pngFile) + ".png";
        doc["resolution"] = static_cast<double>(grid.getResolution());

        // origin = [x_min, y_min, 0.0]  (bottom-left corner, yaw=0)
        doc["origin"] = mrpt::containers::yaml::Sequence(
            {static_cast<double>(grid.getXMin()), static_cast<double>(grid.getYMin()), 0.0});

        doc["negate"]          = argNegate.isSet() ? 1 : 0;
        doc["occupied_thresh"] = argOccupiedThresh.getValue();
        doc["free_thresh"]     = argFreeThresh.getValue();
        doc["mode"]            = modeStr;

        std::cout << "[mm2grid] Saving YAML to '" << yamlFile << "'...\n";

        std::ofstream yamlOut(yamlFile);

        if (!yamlOut.is_open())
        {
            THROW_EXCEPTION_FMT("Could not open '%s' for writing.", yamlFile.c_str());
        }

        doc.printAsYAML(yamlOut);
    }

    std::cout << "[mm2grid] Done.\n"
              << "  PNG  : " << pngFile << "\n"
              << "  YAML : " << yamlFile << "\n";
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    try
    {
        if (!cmd.parse(argc, argv))
        {
            return 1;
        }
        run_mm2grid();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
