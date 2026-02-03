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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#if MRPT_VERSION < 0x030000  // <3.0.0
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

namespace
{
// CLI flags:
TCLAP::CmdLine cmd("mm2txt");

TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", true, "myMap.mm", "myMap.mm", cmd);

TCLAP::MultiArg<std::string> argLayers(
    "l", "layer",
    "Layer to export. If not provided, all will be exported. This argument can "
    "appear several times.",
    false, "layerName", cmd);

TCLAP::ValueArg<std::string> argExportFields(
    "", "export-fields",
    "Comma-separated list of fields to export (e.g., 'x,y,z,intensity'). If not "
    "provided, all available fields will be exported. Fields will be exported in "
    "the specified order.",
    false, "", "field1,field2,...", cmd);

TCLAP::SwitchArg argIgnoreMissingFields(
    "", "ignore-missing-fields",
    "If defined, the lack of any of the --export-fields in the map will be considered a warning "
    "instead of an error; missing fields are skipped.",
    cmd);

bool saveToTxt(
    const mrpt::maps::CGenericPointsMap& pts, const std::string& fileName, bool printHeader,
    const std::vector<std::string>& selectedFields = {})
{
    FILE* f = mrpt::system::os::fopen(fileName.c_str(), "wt");
    if (!f)
    {
        return false;
    }

    // Data:
    const auto& floatFields  = pts.float_fields();
    const auto& uint16Fields = pts.uint16_fields();
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    const auto& doubleFields = pts.double_fields();
    const auto& uint8Fields  = pts.uint8_fields();
#endif

    // Determine which fields to export and in what order
    std::vector<std::string> fieldsToExport;

    if (selectedFields.empty())
    {
        // Export all fields in default order
        fieldsToExport.push_back("x");
        fieldsToExport.push_back("y");
        fieldsToExport.push_back("z");

        for (const auto& [name, _] : floatFields)
        {
            fieldsToExport.push_back(std::string(name));
        }
        for (const auto& [name, _] : uint16Fields)
        {
            fieldsToExport.push_back(std::string(name));
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (const auto& [name, _] : doubleFields)
        {
            fieldsToExport.push_back(std::string(name));
        }
        for (const auto& [name, _] : uint8Fields)
        {
            fieldsToExport.push_back(std::string(name));
        }
#endif
    }
    else
    {
        // Use selected fields in specified order
        fieldsToExport = selectedFields;
    }

    // header?
    if (printHeader)
    {
        for (size_t i = 0; i < fieldsToExport.size(); i++)
        {
            const auto& fieldName = fieldsToExport[i];
            mrpt::system::os::fprintf(
                f, "%.*s%s", static_cast<int>(fieldName.length()), fieldName.data(),
                (i + 1 < fieldsToExport.size()) ? " " : "");
        }
        mrpt::system::os::fprintf(f, "\n");
    }

    // print fields:
    const auto& xs = pts.getPointsBufferRef_x();
    const auto& ys = pts.getPointsBufferRef_y();
    const auto& zs = pts.getPointsBufferRef_z();
    if (xs.empty())
    {
        mrpt::system::os::fclose(f);
        return true;
    }

    const std::size_t n = xs.size();

    for (size_t i = 0; i < n; i++)
    {
        for (size_t fieldIdx = 0; fieldIdx < fieldsToExport.size(); fieldIdx++)
        {
            const auto& fieldName = fieldsToExport[fieldIdx];

            // Check coordinate fields
            if (fieldName == "x")
            {
                mrpt::system::os::fprintf(f, "%.8f", xs.at(i));
            }
            else if (fieldName == "y")
            {
                mrpt::system::os::fprintf(f, "%.8f", ys.at(i));
            }
            else if (fieldName == "z")
            {
                mrpt::system::os::fprintf(f, "%.8f", zs.at(i));
            }
            // Check float fields
            else if (floatFields.count(fieldName))
            {
                mrpt::system::os::fprintf(f, "%.8e", floatFields.at(fieldName).at(i));
            }
            // Check uint16 fields
            else if (uint16Fields.count(fieldName))
            {
                mrpt::system::os::fprintf(f, "%u", uint16Fields.at(fieldName).at(i));
            }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            // Check double fields
            else if (doubleFields.count(fieldName))
            {
                mrpt::system::os::fprintf(f, "%.16le", doubleFields.at(fieldName).at(i));
            }
            // Check uint8 fields
            else if (uint8Fields.count(fieldName))
            {
                mrpt::system::os::fprintf(
                    f, "%i", static_cast<int>(uint8Fields.at(fieldName).at(i)));
            }
#endif
            else
            {
                // Field not found - this should have been caught earlier
                mrpt::system::os::fprintf(f, "0");
            }

            // Add separator or newline
            mrpt::system::os::fprintf(f, "%s", (fieldIdx + 1 < fieldsToExport.size()) ? " " : "");
        }
        mrpt::system::os::fprintf(f, "\n");
    }

    mrpt::system::os::fclose(f);
    return true;
}

}  // namespace

void run_mm2txt()
{
    using namespace std::string_literals;

    const auto& filInput = argMapFile.getValue();

    ASSERT_FILE_EXISTS_(argMapFile.getValue());

    std::cout << "[mm-info] Reading input map from: '" << filInput << "'..." << std::endl;
    const double mm_t0 = mrpt::Clock::nowDouble();

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    const double mm_t1 = mrpt::Clock::nowDouble();
    std::cout << "[mm-info] Done read map in " << (mm_t1 - mm_t0) << " sec. Contents:\n"
              << mm.contents_summary() << "\n";

    std::vector<std::string> layers;
    if (argLayers.isSet())
    {
        // only selected:
        for (const auto& s : argLayers.getValue())
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
    if (argExportFields.isSet())
    {
        const auto&              fieldsStr = argExportFields.getValue();
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

        auto* pts = mp2p_icp::MapToPointsMap(*mm.layers.at(name));
        if (!pts)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' is of type '%s' which cannot be converted into a "
                "point cloud for exporting in TXT format.",
                name.c_str(), mm.layers.at(name)->GetRuntimeClass()->className);
        }

        if (auto* genxyz = dynamic_cast<const mrpt::maps::CGenericPointsMap*>(pts); genxyz)
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
                        if (!argIgnoreMissingFields.isSet())
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

            bool printHeader = true;
            saveToTxt(*genxyz, filName, printHeader, validFields);
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
                pts->save3D_to_text_file(filName);
            }
#pragma GCC diagnostic pop
        }
#else
        else
        {
            printSelectedFieldsWarning();
            pts->save3D_to_text_file(filName);
        }
#endif
    }
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        run_mm2txt();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}