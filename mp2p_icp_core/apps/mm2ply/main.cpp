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
 * @file   mm2ply/main.cpp
 * @brief  A CLI tool to export the layers of a metric map (*.mm) as PLY
 * @author Jose Luis Blanco Claraco
 * @date   Dec 28, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/point_cloud_export.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <optional>
#include <stdexcept>

// CLI flags
CLI::App cmd{"mm2ply"};

std::string arg_input;
std::string arg_out_prefix;
bool        arg_binary = false;
std::string argExportFields;
bool        argIgnoreMissingFields = false;
std::string argFrame               = "map";
std::string arg_plugins;

int main(int argc, char** argv)
{
    cmd.add_option("-i,--input", arg_input, "Input .mm file")->required();
    cmd.add_option("-o,--output", arg_out_prefix, "Prefix for output .ply files");
    cmd.add_flag("-b,--binary", arg_binary, "Export in binary format (default: ASCII)");
    cmd.add_option(
        "--export-fields", argExportFields,
        "Comma-separated list of fields to export (e.g., 'x,y,z,intensity'). If not "
        "provided, all available fields will be exported. Fields will be exported in "
        "the specified order.");
    cmd.add_flag(
        "--ignore-missing-fields", argIgnoreMissingFields,
        "If defined, the lack of any of the --export-fields in the map will be considered a "
        "warning "
        "instead of an error; missing fields are skipped.");
    cmd.add_option(
        "--frame", argFrame,
        "Coordinate frame for exported points: 'map' (default) uses the map local frame; "
        "'enu' transforms points to the East-North-Up frame (requires georeferencing data in the "
        "map).");
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
        std::optional<mrpt::poses::CPose3D> T_enu_to_map;
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
                std::cout << "[mm2ply] Exporting points in ENU frame." << std::endl;
            }
            else if (frame != "map")
            {
                throw std::runtime_error("Invalid --frame value. Must be 'map' or 'enu'.");
            }
        }

        std::string prefix = arg_out_prefix.empty()
                                 ? mrpt::system::fileNameChangeExtension(arg_input, "")
                                 : arg_out_prefix;

        // Parse export fields if specified
        std::vector<std::string> selectedFields;
        if (!argExportFields.empty())
        {
            const auto&              fieldsStr = argExportFields;
            std::vector<std::string> tokens;
            mrpt::system::tokenize(fieldsStr, ", \t", tokens);

            // Remove empty tokens and trim
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

            std::cout << "Exporting only selected fields: ";
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
            std::vector<std::string> validFields;

            // Validate selected fields exist in this point cloud
            if (!selectedFields.empty())
            {
                const auto f_names = pts->getPointFieldNames_float();
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                const auto d_names   = pts->getPointFieldNames_double();
                const auto u16_names = pts->getPointFieldNames_uint16();
                const auto u8_names  = pts->getPointFieldNames_uint8();
#endif

                // Filter selectedFields to only include fields that exist
                for (const auto& field : selectedFields)
                {
                    bool found = (field == "x" || field == "y" || field == "z");

                    if (!found)
                    {
                        for (const auto& n : f_names)
                        {
                            if (n == field)
                            {
                                found = true;
                                break;
                            }
                        }
                    }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                    if (!found)
                    {
                        for (const auto& n : d_names)
                        {
                            if (n == field)
                            {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (!found)
                    {
                        for (const auto& n : u16_names)
                        {
                            if (n == field)
                            {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (!found)
                    {
                        for (const auto& n : u8_names)
                        {
                            if (n == field)
                            {
                                found = true;
                                break;
                            }
                        }
                    }
#endif

                    if (!found)
                    {
                        std::cerr << "Warning: Field '" << field << "' not found in layer '" << name
                                  << "'. Available fields: x, y, z";
                        for (const auto& fn : f_names)
                        {
                            std::cerr << ", " << fn;
                        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                        for (const auto& fn : d_names)
                        {
                            std::cerr << ", " << fn;
                        }
                        for (const auto& fn : u16_names)
                        {
                            std::cerr << ", " << fn;
                        }
                        for (const auto& fn : u8_names)
                        {
                            std::cerr << ", " << fn;
                        }
#endif
                        std::cerr << " - skipping this field." << std::endl;
                        if (!argIgnoreMissingFields)
                        {
                            throw std::runtime_error(
                                "Field '" + field +
                                "' specified in --export-fields not found in layer '" + name + "'");
                        }
                    }
                    else
                    {
                        validFields.push_back(field);
                    }
                }
                if (validFields.empty())
                {
                    std::cerr << "Warning: None of the requested fields exist in layer '" << name
                              << "'. Skipping export for this layer." << std::endl;
                    continue;
                }
            }

            std::string out = prefix + "_" + name + ".ply";
            std::cout << "Exporting '" << name << "' to " << out << "..." << std::endl;
            mp2p_icp::ply_export_options_t opts;
            opts.binary    = arg_binary;
            opts.fields    = validFields;
            opts.transform = T_enu_to_map;
            mp2p_icp::save_point_cloud_to_ply(*pts, out, opts);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}