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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <fstream>
#include <iostream>

// CLI flags
TCLAP::CmdLine cmd("mm2ply");

TCLAP::ValueArg<std::string> arg_input("i", "input", "Input .mm file", true, "", "map.mm", cmd);

TCLAP::ValueArg<std::string> arg_out_prefix(
    "o", "output", "Prefix for output .ply files", false, "", "out", cmd);

TCLAP::SwitchArg arg_binary("b", "binary", "Export in binary format (default: ASCII)", cmd, false);

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

// ----------------------------------------------------------------
// PLY Export Logic using CPointsMap field-generic API
// ----------------------------------------------------------------
void saveToPly(
    const mrpt::maps::CPointsMap& pc, const std::string& filename, bool binary,
    const std::vector<std::string>& selectedFields = {}, bool ignoreMissingFields = false)
{
    std::ofstream f;
    f.open(filename, binary ? std::ios::binary : std::ios::out);
    if (!f.is_open())
    {
        throw std::runtime_error("Could not open file: " + filename);
    }

    const size_t N = pc.size();

    // 1. Discover Fields
    const auto f_names = pc.getPointFieldNames_float();
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    const auto d_names   = pc.getPointFieldNames_double();
    const auto u16_names = pc.getPointFieldNames_uint16();
    const auto u8_names  = pc.getPointFieldNames_uint8();
#endif

    // Determine which fields to export and in what order
    std::vector<std::string> fieldsToExport;

    if (selectedFields.empty())
    {
        // Export all fields in default order
        fieldsToExport.push_back("x");
        fieldsToExport.push_back("y");
        fieldsToExport.push_back("z");

        for (const auto& n : f_names)
        {
            if (n != "x" && n != "y" && n != "z")
            {
                fieldsToExport.push_back(std::string(n));
            }
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (const auto& n : d_names)
        {
            fieldsToExport.push_back(std::string(n));
        }
        for (const auto& n : u16_names)
        {
            fieldsToExport.push_back(std::string(n));
        }
        for (const auto& n : u8_names)
        {
            fieldsToExport.push_back(std::string(n));
        }
#endif
    }
    else
    {
        // Use selected fields in specified order
        fieldsToExport = selectedFields;
    }

    // 2. Write Header
    f << "ply" << std::endl;
    f << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0" << std::endl;
    f << "element vertex " << N << std::endl;

    auto mapPlyName = [](std::string_view n) -> std::string
    {
        if (n == "color_r" || n == "color_rf")
        {
            return "red";
        }
        if (n == "color_g" || n == "color_gf")
        {
            return "green";
        }
        if (n == "color_b" || n == "color_bf")
        {
            return "blue";
        }
        return std::string(n);
    };

    auto getFieldType = [&](const std::string& fieldName) -> std::string
    {
        if (fieldName == "x" || fieldName == "y" || fieldName == "z")
        {
            return "float";
        }
        for (const auto& n : f_names)
        {
            if (n == fieldName)
            {
                return "float";
            }
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (const auto& n : d_names)
        {
            if (n == fieldName)
            {
                return "double";
            }
        }
        for (const auto& n : u16_names)
        {
            if (n == fieldName)
            {
                return "ushort";
            }
        }
        for (const auto& n : u8_names)
        {
            if (n == fieldName)
            {
                return "uchar";
            }
        }
#endif
        return "float";  // default
    };

    for (const auto& fieldName : fieldsToExport)
    {
        f << "property " << getFieldType(fieldName) << " " << mapPlyName(fieldName) << "\n";
    }

    f << "end_header\n";

    // 3. Prepare Accessors
    const auto &xs = pc.getPointsBufferRef_x(), &ys = pc.getPointsBufferRef_y(),
               &zs = pc.getPointsBufferRef_z();

    struct FieldAccessor
    {
        std::string name;
        enum
        {
            FLOAT,
            DOUBLE,
            UINT16,
            UINT8
        } type = FLOAT;

        const void* bufPtr = nullptr;
    };

    std::vector<FieldAccessor> accessors;

    for (const auto& fieldName : fieldsToExport)
    {
        FieldAccessor acc;
        acc.name = fieldName;

        if (fieldName == "x" || fieldName == "y" || fieldName == "z")
        {
            acc.type   = FieldAccessor::FLOAT;
            acc.bufPtr = (fieldName == "x")   ? (const void*)&xs
                         : (fieldName == "y") ? (const void*)&ys
                                              : (const void*)&zs;
        }
        else
        {
            bool found = false;
            for (const auto& n : f_names)
            {
                if (n == fieldName)
                {
                    acc.type   = FieldAccessor::FLOAT;
                    acc.bufPtr = pc.getPointsBufferRef_float_field(n);
                    found      = true;
                    break;
                }
            }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            if (!found)
            {
                for (const auto& n : d_names)
                {
                    if (n == fieldName)
                    {
                        acc.type   = FieldAccessor::DOUBLE;
                        acc.bufPtr = pc.getPointsBufferRef_double_field(n);
                        found      = true;
                        break;
                    }
                }
            }
            if (!found)
            {
                for (const auto& n : u16_names)
                {
                    if (n == fieldName)
                    {
                        acc.type   = FieldAccessor::UINT16;
                        acc.bufPtr = pc.getPointsBufferRef_uint16_field(n);
                        found      = true;
                        break;
                    }
                }
            }
            if (!found)
            {
                for (const auto& n : u8_names)
                {
                    if (n == fieldName)
                    {
                        acc.type   = FieldAccessor::UINT8;
                        acc.bufPtr = pc.getPointsBufferRef_uint8_field(n);
                        found      = true;
                        break;
                    }
                }
            }
#endif
            if (!found)
            {
                if (!ignoreMissingFields)
                {
                    throw std::runtime_error("Field not found: " + fieldName);
                }
                // Mark this accessor as invalid (nullptr) - will output zeros
                acc.bufPtr = nullptr;
            }
        }
        accessors.push_back(acc);
    }

    // 4. Data Loop
    auto write_val = [&](auto val, const char* fmt)
    {
        if (binary)
        {
            f.write(reinterpret_cast<const char*>(&val), sizeof(val));
        }
        else
        {
            f << mrpt::format(fmt, val) << " ";
        }
    };

    for (size_t i = 0; i < N; ++i)
    {
        for (const auto& acc : accessors)
        {
            // Handle missing fields (when ignoreMissingFields is true)
            if (acc.bufPtr == nullptr)
            {
                // Output zero for missing fields
                switch (acc.type)
                {
                    case FieldAccessor::FLOAT:
                        write_val(0.0f, "%.8e");
                        break;
#if MRPT_VERSION >= 0x020f03
                    case FieldAccessor::DOUBLE:
                        write_val(0.0, "%.16le");
                        break;
                    case FieldAccessor::UINT16:
                        write_val(static_cast<uint16_t>(0), "%u");
                        break;
                    case FieldAccessor::UINT8:
                        write_val(static_cast<uint8_t>(0), "%i");
                        break;
#endif
                    default:
                        write_val(0.0f, "%.8e");
                        break;
                }
                continue;
            }

            switch (acc.type)
            {
                default:
                    throw std::runtime_error("Error: unknown field type");

                case FieldAccessor::FLOAT:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<float>*>(acc.bufPtr);
                    write_val(buf[i], "%.8e");
                    break;
                }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                case FieldAccessor::DOUBLE:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<double>*>(acc.bufPtr);
                    write_val(buf[i], "%.16le");
                    break;
                }
                case FieldAccessor::UINT16:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint16_t>*>(acc.bufPtr);
                    write_val(buf[i], "%u");
                    break;
                }
                case FieldAccessor::UINT8:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint8_t>*>(acc.bufPtr);
                    write_val(buf[i], "%i");
                    break;
                }
#endif
            }
        }
        if (!binary)
        {
            f << "\n";
        }
    }
}

int main(int argc, char** argv)
{
    try
    {
        if (!cmd.parse(argc, argv))
        {
            return 0;
        }
        mp2p_icp::metric_map_t mm;
        mm.load_from_file(arg_input.getValue());

        std::string prefix = arg_out_prefix.getValue().empty()
                                 ? mrpt::system::fileNameChangeExtension(arg_input.getValue(), "")
                                 : arg_out_prefix.getValue();

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
            auto* pts = mp2p_icp::MapToPointsMap(*layer);
            if (!pts)
            {
                continue;
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
                        if (!argIgnoreMissingFields.isSet())
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
            saveToPly(*pts, out, arg_binary.getValue(), validFields);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}