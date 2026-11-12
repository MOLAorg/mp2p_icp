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
 * @file   mm2txt/main.cpp
 * @brief  A CLI tool to export the layers of a metric map (`*.mm`) as CSV/TXT
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/point_cloud_export.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <stdexcept>

#if MRPT_VERSION < 0x030000  // <3.0.0
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

namespace
{
// CLI flags:
CLI::App cmd{"mm2txt"};

std::string              argMapFile;
std::vector<std::string> argLayers;
std::string              argExportFields;
bool                     argIgnoreMissingFields = false;
bool                     argNoHeader            = false;
std::string              argFrame               = "map";
std::string              arg_plugins;

}  // namespace

void run_mm2txt()
{
    using namespace std::string_literals;

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

    const auto& filInput = argMapFile;

    ASSERT_FILE_EXISTS_(argMapFile);

    std::cout << "[mm2txt] Reading input map from: '" << filInput << "'..." << std::endl;
    const double mm_t0 = mrpt::Clock::nowDouble();

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    const double mm_t1 = mrpt::Clock::nowDouble();
    std::cout << "[mm2txt] Done read map in " << (mm_t1 - mm_t0) << " sec. Contents:\n"
              << mm.contents_summary() << "\n";

    std::vector<std::string> layers;
    if (!argLayers.empty())
    {
        // only selected:
        for (const auto& s : argLayers)
        {
            layers.push_back(s);
        }
    }
    else
    {  // all:
        for (const auto& [name, map] : mm.layers)
        {
            layers.push_back(name);
        }
    }

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
            THROW_EXCEPTION("--export-fields specified but no valid fields found!");
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

    // Handle --frame option
    std::optional<mrpt::poses::CPose3D> T_enu_to_map;
    {
        const auto& frame = argFrame;
        if (frame == "enu")
        {
            ASSERTMSG_(
                mm.georeferencing.has_value(),
                "Cannot use --frame enu: the input map does not contain georeferencing "
                "information.");
            T_enu_to_map = mm.georeferencing->T_enu_to_map.mean;
            std::cout << "[mm2txt] Exporting points in ENU frame.\n";
        }
        else
        {
            ASSERTMSG_(frame == "map", "Invalid --frame value. Must be 'map' or 'enu'.");
        }
    }

    const auto printSelectedFieldsWarning = [&selectedFields]()
    {
        if (!selectedFields.empty())
        {
            std::cerr << "Warning: --export-fields not supported for legacy point map types, "
                         "exporting all fields.\n";
        }
    };

    const auto baseFilName = mrpt::system::extractFileName(filInput);

    // Export them:
    for (const auto& name : layers)
    {
        const std::string filName = baseFilName + "_"s + name + ".txt"s;

        std::cout << "Exporting layer: '" << name << "' to file '" << filName << "'..."
                  << std::endl;

        ASSERTMSG_(mm.layers.count(name) == 1, "Layer not found in metric map!");

        const auto* pts = mp2p_icp::MapToPointsMapView(*mm.layers.at(name));
        if (!pts)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' is of type '%s' which cannot be converted into a "
                "point cloud for exporting in TXT format.",
                name.c_str(), mm.layers.at(name)->GetRuntimeClass()->className);
        }
        if (!mp2p_icp::MapToPointsMap(*mm.layers.at(name)))
        {
            std::cout << "Note: layer '" << name
                      << "' is exported through its point cloud view, "
                         "which the map class may have subsampled.\n";
        }

        const auto* genxyz = dynamic_cast<const mrpt::maps::CGenericPointsMap*>(pts);
        if (T_enu_to_map.has_value() && !genxyz)
        {
            THROW_EXCEPTION_FMT(
                "--frame enu is only supported for layers of type CGenericPointsMap, but layer "
                "'%s' is of type '%s'.",
                name.c_str(), mm.layers.at(name)->GetRuntimeClass()->className);
        }

        if (genxyz)
        {
            // Validate selected fields exist in this point cloud
            std::vector<std::string> validFields;
            if (!selectedFields.empty())
            {
                const auto& floatFields  = genxyz->float_fields();
                const auto& uint16Fields = genxyz->uint16_fields();
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                const auto& doubleFields = genxyz->double_fields();
                const auto& uint8Fields  = genxyz->uint8_fields();
#endif

                for (const auto& field : selectedFields)
                {
                    bool found =
                        (field == "x" || field == "y" || field == "z" || floatFields.count(field) ||
                         uint16Fields.count(field));
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                    found = found || doubleFields.count(field) || uint8Fields.count(field);
#endif

                    if (!found)
                    {
                        std::cerr << "Warning: Field '" << field << "' not found in layer '" << name
                                  << "'. Available fields: x, y, z";
                        for (const auto& [fname, _] : floatFields)
                        {
                            std::cerr << ", " << fname;
                        }
                        for (const auto& [fname, _] : uint16Fields)
                        {
                            std::cerr << ", " << fname;
                        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                        for (const auto& [fname, _] : doubleFields)
                        {
                            std::cerr << ", " << fname;
                        }
                        for (const auto& [fname, _] : uint8Fields)
                        {
                            std::cerr << ", " << fname;
                        }
#endif
                        std::cerr << " - skipping this field." << std::endl;
                        if (!argIgnoreMissingFields)
                        {
                            THROW_EXCEPTION_FMT(
                                "Field '%s' specified in --export-fields not found in layer '%s'",
                                field.c_str(), name.c_str());
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
            else
            {
                validFields = selectedFields;
            }

            mp2p_icp::txt_export_options_t opts;
            opts.header    = !argNoHeader;
            opts.fields    = validFields;
            opts.transform = T_enu_to_map;
            mp2p_icp::save_point_cloud_to_txt(*genxyz, filName, opts);
        }
#if MRPT_VERSION < 0x030000  // <3.0.0
        else
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(pts); xyzirt)
            {
                printSelectedFieldsWarning();
                xyzirt->saveXYZIRT_to_text_file(filName);
            }
            else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(pts); xyzi)
            {
                printSelectedFieldsWarning();

                xyzi->saveXYZI_to_text_file(filName);
            }
            else
            {
                printSelectedFieldsWarning();
                if (!pts->save3D_to_text_file(filName))
                {
                    THROW_EXCEPTION_FMT("Error saving to text file '%s'", filName.c_str());
                }
            }
#pragma GCC diagnostic pop
        }
#else
        else
        {
            printSelectedFieldsWarning();
            if (!pts->save3D_to_text_file(filName))
            {
                THROW_EXCEPTION_FMT("Error saving to text file '%s'", filName.c_str());
            }
        }
#endif
    }
}

int main(int argc, char** argv)
{
    cmd.add_option("input", argMapFile, "Load this metric map file (*.mm)")->required();
    cmd.add_option(
        "-l,--layer", argLayers,
        "Layer to export. If not provided, all will be exported. This argument can "
        "appear several times.");
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
    cmd.add_flag(
        "--no-header", argNoHeader,
        "Do not write the first line with the field names, so the output is a plain numeric "
        "matrix (e.g. for numpy.loadtxt()).");
    cmd.add_option(
        "--frame", argFrame,
        "Coordinate frame for exported points: 'map' (default) uses the map local frame; "
        "'enu' transforms points to the East-North-Up frame (requires georeferencing data in the "
        "map).");
    cmd.add_option(
        "--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        run_mm2txt();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}