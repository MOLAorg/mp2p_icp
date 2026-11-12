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
 * @file   mm2las/main.cpp
 * @brief  A CLI tool to export the layers of a metric map (*.mm) as LAS 1.4
 * @author Jose Luis Blanco Claraco, Claude AI
 * @date   Jan 19, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/point_cloud_export.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <optional>
#include <stdexcept>

// CLI flags
CLI::App cmd{"mm2las"};

std::string arg_input;
std::string arg_out_prefix;
std::string argExportFields;
std::string argSystemId           = "mm2las";
std::string argGeneratingSoftware = "MOLA mm2las";
std::string argFrame              = "map";
std::string arg_plugins;

int main(int argc, char** argv)
{
    cmd.add_option("-i,--input", arg_input, "Input .mm file")->required();
    cmd.add_option("-o,--output", arg_out_prefix, "Prefix for output .las files");
    cmd.add_option(
        "--export-fields", argExportFields,
        "Comma-separated list of fields to export (e.g., 'x,y,z,intensity,rgb'). "
        "If not provided, all available fields will be exported as Extended VLRs.");
    cmd.add_option("--system-id", argSystemId, "System Identifier for LAS header (max 32 chars)");
    cmd.add_option(
        "--generating-software", argGeneratingSoftware,
        "Generating Software for LAS header (max 32 chars)");
    cmd.add_option(
        "--frame", argFrame,
        "Coordinate frame for exported points: 'map' (default) uses the map local frame; "
        "'enu' transforms points to the East-North-Up frame (requires georeferencing data in "
        "the map); "
        "'geodetic' exports as WGS-84 lon/lat/alt with EPSG:4979 CRS embedded in the LAS file "
        "(requires georeferencing data). If per-point latitude/longitude/altitude fields already "
        "exist "
        "in the map, they are used directly; otherwise, the conversion is computed on the fly.");
    cmd.add_option(
        "-l,--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        // Load plugins:
        if (!arg_plugins.empty())
        {
            std::string errMsg;
            const auto& plugins = arg_plugins;
            std::cout << "Loading plugin(s): " << plugins << std::endl;
            if (!mrpt::system::loadPluginModules(plugins, errMsg))
            {
                throw std::runtime_error(errMsg);
            }
        }

        mp2p_icp::metric_map_t mm;
        mm.load_from_file(arg_input);

        // Handle --frame option
        std::optional<mrpt::poses::CPose3D>                   T_enu_to_map;
        bool                                                  exportGeodetic = false;
        std::optional<mp2p_icp::metric_map_t::Georeferencing> geodeticExport;
        {
            const auto& frame = argFrame;
            if (frame == "enu")
            {
                if (!mm.georeferencing.has_value())
                {
                    throw std::runtime_error(
                        "Cannot use --frame enu: the input map does not contain georeferencing "
                        "information.");
                }
                T_enu_to_map = mm.georeferencing->T_enu_to_map.mean;
                std::cout << "[mm2las] Exporting points in ENU frame." << std::endl;
            }
            else if (frame == "geodetic")
            {
                exportGeodetic = true;
                // Provide georeferencing context when available (needed for on-the-fly
                // conversion). If absent, saveToLas will use per-point lat/lon/alt fields
                // directly, or fail with a clear error if those are also absent.
                if (mm.georeferencing.has_value() && !mm.georeferencing->geo_coord.isClear())
                {
                    geodeticExport = mm.georeferencing;
                    std::cout << "[mm2las] Exporting points in geodetic WGS-84 coordinates "
                                 "(EPSG:4979)."
                              << std::endl;
                    std::cout << "[mm2las] Reference: lat="
                              << mm.georeferencing->geo_coord.lat.getDecimalValue()
                              << " lon=" << mm.georeferencing->geo_coord.lon.getDecimalValue()
                              << " alt=" << mm.georeferencing->geo_coord.height << " m"
                              << std::endl;
                }
                else
                {
                    std::cout
                        << "[mm2las] Exporting points in geodetic WGS-84 coordinates (EPSG:4979) "
                           "using per-point latitude/longitude/altitude fields."
                        << std::endl;
                }
            }
            else if (frame != "map")
            {
                throw std::runtime_error(
                    "Invalid --frame value. Must be 'map', 'enu', or 'geodetic'.");
            }
        }

        std::string prefix = arg_out_prefix.empty()
                                 ? mrpt::system::fileNameChangeExtension(arg_input, "")
                                 : arg_out_prefix;

        std::string systemId           = argSystemId;
        std::string generatingSoftware = argGeneratingSoftware;

        // Parse export fields if specified
        std::vector<std::string> selectedFields;
        if (!argExportFields.empty())
        {
            const auto&              fieldsStr = argExportFields;
            std::vector<std::string> tokens;
            mrpt::system::tokenize(fieldsStr, ", \t", tokens);

            // Remove empty tokens
            for (auto& token : tokens)
            {
                if (!token.empty())
                {
                    selectedFields.push_back(token);
                }
            }

            if (selectedFields.empty())
            {
                throw std::runtime_error("--export-fields specified but no valid fields found!");
            }

            std::cout << "Exporting selected fields: ";
            for (size_t i = 0; i < selectedFields.size(); i++)
            {
                std::cout << selectedFields[i];
                if (i + 1 < selectedFields.size())
                {
                    std::cout << ", ";
                }
            }
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Exporting all available fields (LAS 1.4 Point Format 8 + extra "
                         "dimensions)"
                      << std::endl;
        }

        for (const auto& [name, layer] : mm.layers)
        {
            const auto* pts = mp2p_icp::MapToPointsMapView(*layer);
            if (!pts)
            {
                continue;
            }
            if (!mp2p_icp::MapToPointsMap(*layer))
            {
                std::cout << "Note: layer '" << name
                          << "' is exported through its point cloud view, "
                             "which the map class may have subsampled.\n";
            }

            std::string out = prefix + "_" + name + ".las";
            std::cout << "Exporting '" << name << "' to " << out << " (" << pts->size()
                      << " points)..." << std::endl;
            mp2p_icp::las_export_options_t opts;
            opts.fields              = selectedFields;
            opts.system_id           = systemId;
            opts.generating_software = generatingSoftware;
            opts.transform           = T_enu_to_map;
            opts.geodetic            = exportGeodetic;
            opts.georeferencing      = geodeticExport;
            opts.verbose             = true;
            mp2p_icp::save_point_cloud_to_las(*pts, out, opts);
        }

        std::cout << "Done." << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}