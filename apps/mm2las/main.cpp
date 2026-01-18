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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>

// CLI flags
TCLAP::CmdLine cmd("mm2las");

TCLAP::ValueArg<std::string> arg_input("i", "input", "Input .mm file", true, "", "map.mm", cmd);

TCLAP::ValueArg<std::string> arg_out_prefix(
    "o", "output", "Prefix for output .las files", false, "", "out", cmd);

TCLAP::ValueArg<std::string> argExportFields(
    "", "export-fields",
    "Comma-separated list of fields to export (e.g., 'x,y,z,intensity,rgb'). "
    "If not provided, all available fields will be exported as Extended VLRs.",
    false, "", "field1,field2,...", cmd);

TCLAP::ValueArg<std::string> argSystemId(
    "", "system-id", "System Identifier for LAS header (max 32 chars)", false, "mm2las", "string",
    cmd);

TCLAP::ValueArg<std::string> argGeneratingSoftware(
    "", "generating-software", "Generating Software for LAS header (max 32 chars)", false,
    "MOLA mm2las", "string", cmd);

// ----------------------------------------------------------------
// LAS 1.4 Format Structures
// ----------------------------------------------------------------

#pragma pack(push, 1)

struct LASHeader_1_4
{
    char     file_signature[4]         = {'L', 'A', 'S', 'F'};
    uint16_t file_source_id            = 0;
    uint16_t global_encoding           = 0x10;  // Using 1.4 WKT standard (not older GeoTIFF)
    uint32_t project_id_guid_data_1    = 0;
    uint16_t project_id_guid_data_2    = 0;
    uint16_t project_id_guid_data_3    = 0;
    uint8_t  project_id_guid_data_4[8] = {0};
    uint8_t  version_major             = 1;
    uint8_t  version_minor             = 4;
    char     system_identifier[32]     = "mm2las";
    char     generating_software[32]   = "MOLA mm2las";
    uint16_t file_creation_day_of_year = 0;
    uint16_t file_creation_year        = 0;
    uint16_t header_size               = 375;  // LAS 1.4 header size
    uint32_t offset_to_point_data      = 375;
    uint32_t number_of_variable_length_records    = 0;
    uint8_t  point_data_format_id                 = 8;  // Point Format 8
    uint16_t point_data_record_length             = 38;  // Format 8 base size
    uint32_t legacy_number_of_point_records       = 0;
    uint32_t legacy_number_of_points_by_return[5] = {0};
    double   x_scale_factor                       = 0.001;  // 1mm precision
    double   y_scale_factor                       = 0.001;
    double   z_scale_factor                       = 0.001;
    double   x_offset                             = 0.0;
    double   y_offset                             = 0.0;
    double   z_offset                             = 0.0;
    double   max_x                                = 0.0;
    double   min_x                                = 0.0;
    double   max_y                                = 0.0;
    double   min_y                                = 0.0;
    double   max_z                                = 0.0;
    double   min_z                                = 0.0;
    // LAS 1.4 additions:
    uint64_t start_of_waveform_data_packet_record    = 0;
    uint64_t start_of_first_evlr                     = 0;
    uint32_t number_of_evlrs                         = 0;
    uint64_t extended_number_of_point_records        = 0;
    uint64_t extended_number_of_points_by_return[15] = {0};
};

// Point Data Record Format 8: XYZ + RGB + NIR (38 bytes)
struct LASPointFormat8
{
    int32_t  x;
    int32_t  y;
    int32_t  z;
    uint16_t intensity;
    uint8_t  return_byte;  // Return Number (4 bits), Number of Returns (4 bits)
    uint8_t  flags;  // Classification Flags (4 bits), Scanner Channel (2 bits),
                    // Scan Direction Flag (1 bit), Edge of Flight Line (1 bit)
    uint8_t  classification;
    uint8_t  user_data;
    int16_t  scan_angle;  // -30000 to +30000 (scaled)
    uint16_t point_source_id;
    double   gps_time;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t nir;  // Near Infrared
};

// Variable Length Record (VLR) header
struct VLRHeader
{
    uint16_t reserved        = 0;
    char     user_id[16]     = {0};
    uint16_t record_id       = 0;
    uint16_t record_length   = 0;
    char     description[32] = {0};
};

// Extended Variable Length Record (EVLR) header - used for extra dimensions
struct EVLRHeader
{
    uint16_t reserved        = 0;
    char     user_id[16]     = {0};
    uint16_t record_id       = 0;
    uint64_t record_length   = 0;
    char     description[32] = {0};
};

#pragma pack(pop)

// ----------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------

struct FieldInfo
{
    std::string name;
    enum Type
    {
        FLOAT,
        DOUBLE,
        UINT16,
        UINT8,
        INT32
    } type             = FLOAT;
    const void* bufPtr = nullptr;
    size_t      size   = 0;  // size in bytes for serialization
};

// ----------------------------------------------------------------
// LAS 1.4 Export Logic with Extra Dimensions
// ----------------------------------------------------------------
void saveToLas(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const std::vector<std::string>& selectedFields, const std::string& systemId,
    const std::string& generatingSoftware)
{
    const size_t N = pc.size();
    if (N == 0)
    {
        std::cerr << "Warning: Empty point cloud, skipping." << std::endl;
        return;
    }

    std::ofstream f;
    f.open(filename, std::ios::binary);
    if (!f.is_open())
    {
        throw std::runtime_error("Could not open file: " + filename);
    }

    // 1. Discover available fields
    const auto f_names = pc.getPointFieldNames_float();
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    const auto d_names   = pc.getPointFieldNames_double();
    const auto u16_names = pc.getPointFieldNames_uint16();
    const auto u8_names  = pc.getPointFieldNames_uint8();
#endif

    // Standard LAS 1.4 Point Format 8 fields
    const std::vector<std::string> standardFields = {"x",
                                                     "y",
                                                     "z",
                                                     "intensity",
                                                     "return_number",
                                                     "number_of_returns",
                                                     "classification",
                                                     "red",
                                                     "green",
                                                     "blue",
                                                     "nir",
                                                     "gps_time",
                                                     "scan_angle",
                                                     "user_data",
                                                     "point_source_id"};

    std::map<std::string, FieldInfo> fieldMap;
    std::vector<std::string>         extraFields;  // Non-standard fields

    const auto &xs = pc.getPointsBufferRef_x(), &ys = pc.getPointsBufferRef_y(),
               &zs = pc.getPointsBufferRef_z();

    // 2. Collect all fields to export
    std::vector<std::string> fieldsToExport;

    if (selectedFields.empty())
    {
        // Export all available fields
        fieldsToExport.push_back("x");
        fieldsToExport.push_back("y");
        fieldsToExport.push_back("z");

        for (const auto& n : f_names)
        {
            if (n == "color_rf")
            {
                fieldsToExport.push_back("red");
            }
            else if (n == "color_gf")
            {
                fieldsToExport.push_back("green");
            }
            else if (n == "color_bf")
            {
                fieldsToExport.push_back("blue");
            }
            else
            {
                fieldsToExport.push_back(std::string(n));
            }
        }
#if MRPT_VERSION >= 0x020f03
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
            if (n == "color_r")
            {
                fieldsToExport.push_back("red");
            }
            else if (n == "color_g")
            {
                fieldsToExport.push_back("green");
            }
            else if (n == "color_b")
            {
                fieldsToExport.push_back("blue");
            }
            else
            {
                fieldsToExport.push_back(std::string(n));
            }
        }
#endif
    }
    else
    {
        fieldsToExport = selectedFields;
    }

    // 3. Process each field
    bool hasRGB       = false;
    bool hasNIR       = false;
    bool hasIntensity = false;
    bool hasGPSTime   = false;

    for (const auto& fieldName : fieldsToExport)
    {
        FieldInfo info;
        info.name = fieldName;

        bool isStandardField = std::find(standardFields.begin(), standardFields.end(), fieldName) !=
                               standardFields.end();

        if (fieldName == "x")
        {
            info.type   = FieldInfo::FLOAT;
            info.bufPtr = &xs;
            info.size   = sizeof(float);
        }
        else if (fieldName == "y")
        {
            info.type   = FieldInfo::FLOAT;
            info.bufPtr = &ys;
            info.size   = sizeof(float);
        }
        else if (fieldName == "z")
        {
            info.type   = FieldInfo::FLOAT;
            info.bufPtr = &zs;
            info.size   = sizeof(float);
        }
        else if (fieldName == "intensity")
        {
            hasIntensity = true;
            bool found   = false;
            for (const auto& n : f_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::FLOAT;
                    info.bufPtr = pc.getPointsBufferRef_float_field(n);
                    info.size   = sizeof(float);
                    found       = true;
                    break;
                }
            }
            if (!found)
            {
                std::cerr << "Warning: intensity field not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "gps_time")
        {
            hasGPSTime = true;
            bool found = false;
#if MRPT_VERSION >= 0x020f03
            for (const auto& n : d_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::DOUBLE;
                    info.bufPtr = pc.getPointsBufferRef_double_field(n);
                    info.size   = sizeof(double);
                    found       = true;
                    break;
                }
            }
#endif
            if (!found)
            {
                std::cerr << "Warning: gps_time field not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "red" || fieldName == "green" || fieldName == "blue")
        {
            hasRGB     = false;
            bool found = false;
#if MRPT_VERSION >= 0x020f03
            for (const auto& n : u16_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::UINT16;
                    info.bufPtr = pc.getPointsBufferRef_uint16_field(n);
                    info.size   = sizeof(uint16_t);
                    found       = true;
                    break;
                }
            }
            if (!found)
            {
                // Try uint8
                std::string uint8Name = "color_" + fieldName.substr(0, 1);
                for (const auto& n : u8_names)
                {
                    if (n == uint8Name)
                    {
                        info.type   = FieldInfo::UINT8;
                        info.bufPtr = pc.getPointsBufferRef_uint8_field(n);
                        info.size   = sizeof(uint8_t);
                        found       = true;
                        break;
                    }
                }
            }
#endif
            // Try float (normalized 0-1)
            if (!found)
            {
                std::string floatName = "color_" + fieldName.substr(0, 1) + "f";
                for (const auto& n : f_names)
                {
                    if (n == floatName)
                    {
                        info.type   = FieldInfo::FLOAT;
                        info.bufPtr = pc.getPointsBufferRef_float_field(n);
                        info.size   = sizeof(float);
                        found       = true;
                        break;
                    }
                }
            }
            if (!found)
            {
                std::cerr << "Warning: RGB field '" << fieldName << "' not found." << std::endl;
                continue;
            }
            hasRGB = true;
        }
        else if (fieldName == "nir")
        {
            hasNIR     = true;
            bool found = false;
#if MRPT_VERSION >= 0x020f03
            for (const auto& n : u16_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::UINT16;
                    info.bufPtr = pc.getPointsBufferRef_uint16_field(n);
                    info.size   = sizeof(uint16_t);
                    found       = true;
                    break;
                }
            }
#endif
            if (!found)
            {
                std::cerr << "Warning: NIR field not found." << std::endl;
                continue;
            }
        }
        else if (
            fieldName == "classification" || fieldName == "return_number" ||
            fieldName == "number_of_returns" || fieldName == "user_data")
        {
#if MRPT_VERSION >= 0x020f03
            bool found = false;
            for (const auto& n : u8_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::UINT8;
                    info.bufPtr = pc.getPointsBufferRef_uint8_field(n);
                    info.size   = sizeof(uint8_t);
                    found       = true;
                    break;
                }
            }
            if (!found)
            {
                std::cerr << "Warning: Field '" << fieldName << "' not found." << std::endl;
                continue;
            }
#else
            std::cerr << "Warning: Field '" << fieldName << "' requires MRPT >= 2.15.3"
                      << std::endl;
            continue;
#endif
        }
        else if (fieldName == "scan_angle")
        {
#if MRPT_VERSION >= 0x020f03
            bool found = false;
            for (const auto& n : f_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::FLOAT;
                    info.bufPtr = pc.getPointsBufferRef_float_field(n);
                    info.size   = sizeof(float);
                    found       = true;
                    break;
                }
            }
            if (!found)
            {
                std::cerr << "Warning: scan_angle field not found." << std::endl;
                continue;
            }
#endif
        }
        else if (fieldName == "point_source_id")
        {
#if MRPT_VERSION >= 0x020f03
            bool found = false;
            for (const auto& n : u16_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::UINT16;
                    info.bufPtr = pc.getPointsBufferRef_uint16_field(n);
                    info.size   = sizeof(uint16_t);
                    found       = true;
                    break;
                }
            }
            if (!found)
            {
                std::cerr << "Warning: point_source_id field not found." << std::endl;
                continue;
            }
#endif
        }
        // Non-standard field - will be added as extra dimension
        else if (!isStandardField)

        {
            bool found = false;
            for (const auto& n : f_names)
            {
                if (n == fieldName)
                {
                    info.type   = FieldInfo::FLOAT;
                    info.bufPtr = pc.getPointsBufferRef_float_field(n);
                    info.size   = sizeof(float);
                    found       = true;
                    break;
                }
            }
#if MRPT_VERSION >= 0x020f03
            if (!found)
            {
                for (const auto& n : d_names)
                {
                    if (n == fieldName)
                    {
                        info.type   = FieldInfo::DOUBLE;
                        info.bufPtr = pc.getPointsBufferRef_double_field(n);
                        info.size   = sizeof(double);
                        found       = true;
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
                        info.type   = FieldInfo::UINT16;
                        info.bufPtr = pc.getPointsBufferRef_uint16_field(n);
                        info.size   = sizeof(uint16_t);
                        found       = true;
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
                        info.type   = FieldInfo::UINT8;
                        info.bufPtr = pc.getPointsBufferRef_uint8_field(n);
                        info.size   = sizeof(uint8_t);
                        found       = true;
                        break;
                    }
                }
            }
#endif
            if (!found)
            {
                std::cerr << "Warning: Extra field '" << fieldName << "' not found." << std::endl;
                continue;
            }
            extraFields.push_back(fieldName);
        }

        fieldMap[fieldName] = info;
    }

    // 4. Calculate bounds
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < N; ++i)
    {
        min_x = std::min(min_x, static_cast<double>(xs[i]));
        max_x = std::max(max_x, static_cast<double>(xs[i]));
        min_y = std::min(min_y, static_cast<double>(ys[i]));
        max_y = std::max(max_y, static_cast<double>(ys[i]));
        min_z = std::min(min_z, static_cast<double>(zs[i]));
        max_z = std::max(max_z, static_cast<double>(zs[i]));
    }

    // 5. Calculate extra bytes for extra dimensions
    size_t extraBytesPerPoint = 0;
    for (const auto& fname : extraFields)
    {
        extraBytesPerPoint += fieldMap[fname].size;
    }

    // 6. Write LAS Header
    LASHeader_1_4 header;

    // Set custom system identifier and generating software
    std::strncpy(header.system_identifier, systemId.c_str(), 31);
    header.system_identifier[31] = '\0';
    std::strncpy(header.generating_software, generatingSoftware.c_str(), 31);
    header.generating_software[31] = '\0';

    header.point_data_format_id     = 8;  // Always use Format 8
    header.point_data_record_length = 38 + static_cast<uint16_t>(extraBytesPerPoint);

    header.extended_number_of_point_records = N;

    header.min_x = min_x;
    header.max_x = max_x;
    header.min_y = min_y;
    header.max_y = max_y;
    header.min_z = min_z;
    header.max_z = max_z;

    // Set offsets to minimum values
    header.x_offset = min_x;
    header.y_offset = min_y;
    header.z_offset = min_z;

    // If we have extra dimensions, we need VLR for "Extra Bytes"
    if (extraBytesPerPoint > 0)
    {
        header.number_of_variable_length_records = 1;
        header.offset_to_point_data = 375 + sizeof(VLRHeader) + (extraFields.size() * 192);
    }
    else
    {
        header.offset_to_point_data = 375;
    }

    std::cout << "LAS 1.4 Point Format 8:" << std::endl;
    std::cout << "  Base size: 38 bytes" << std::endl;
    std::cout << "  Extra dimensions: " << extraFields.size() << " (" << extraBytesPerPoint
              << " bytes)" << std::endl;
    std::cout << "  Total record size: " << header.point_data_record_length << " bytes"
              << std::endl;

    // Write header
    f.write(reinterpret_cast<const char*>(&header), sizeof(LASHeader_1_4));

    // 7. Write VLR for Extra Bytes if needed
    if (extraBytesPerPoint > 0)
    {
        VLRHeader vlr;
        std::strncpy(vlr.user_id, "LASF_Spec", 15);
        vlr.record_id = 4;  // Extra Bytes
        vlr.record_length =
            static_cast<uint16_t>(extraFields.size() * 192);  // 192 bytes per extra dimension
        std::strncpy(vlr.description, "Extra Dimension Descriptions", 31);

        f.write(reinterpret_cast<const char*>(&vlr), sizeof(VLRHeader));

        // Write Extra Bytes records (192 bytes each)
        for (const auto& fname : extraFields)
        {
            uint8_t extraBytesRecord[192] = {0};

            const auto& info = fieldMap[fname];

            // Data type (bytes 2-3)
            uint8_t dataType = 0;
            switch (info.type)
            {
                case FieldInfo::UINT8:
                    dataType = 1;
                    break;
                case FieldInfo::UINT16:
                    dataType = 3;
                    break;
                case FieldInfo::INT32:
                    dataType = 5;
                    break;
                case FieldInfo::FLOAT:
                    dataType = 9;
                    break;
                case FieldInfo::DOUBLE:
                    dataType = 10;
                    break;
            }
            extraBytesRecord[2] = dataType;

            // Name (bytes 4-35)
            std::strncpy(reinterpret_cast<char*>(extraBytesRecord + 4), fname.c_str(), 31);

            f.write(reinterpret_cast<const char*>(extraBytesRecord), 192);
        }
    }

    // 8. Helper to get field value
    auto getFieldValue = [&](const std::string& fieldName, size_t i) -> double
    {
        if (fieldMap.find(fieldName) == fieldMap.end())
        {
            return 0.0;
        }
        const auto& info = fieldMap[fieldName];

        switch (info.type)
        {
            case FieldInfo::FLOAT:
            {
                const auto& buf = *static_cast<const mrpt::aligned_std_vector<float>*>(info.bufPtr);
                return static_cast<double>(buf[i]);
            }
#if MRPT_VERSION >= 0x020f03
            case FieldInfo::DOUBLE:
            {
                const auto& buf =
                    *static_cast<const mrpt::aligned_std_vector<double>*>(info.bufPtr);
                return buf[i];
            }
            case FieldInfo::UINT16:
            {
                const auto& buf =
                    *static_cast<const mrpt::aligned_std_vector<uint16_t>*>(info.bufPtr);
                return static_cast<double>(buf[i]);
            }
            case FieldInfo::UINT8:
            {
                const auto& buf =
                    *static_cast<const mrpt::aligned_std_vector<uint8_t>*>(info.bufPtr);
                return static_cast<double>(buf[i]);
            }
#endif
            default:
                return 0.0;
        }
    };

    // 9. Write point data
    for (size_t i = 0; i < N; ++i)
    {
        LASPointFormat8 pt = {};

        // Scale coordinates
        pt.x = static_cast<int32_t>((xs[i] - header.x_offset) / header.x_scale_factor);
        pt.y = static_cast<int32_t>((ys[i] - header.y_offset) / header.y_scale_factor);
        pt.z = static_cast<int32_t>((zs[i] - header.z_offset) / header.z_scale_factor);

        // Intensity
        if (hasIntensity)
        {
            double val   = getFieldValue("intensity", i);
            pt.intensity = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, val)));
        }

        // Return byte (4 bits each for return number and total returns)
        uint8_t return_num  = fieldMap.count("return_number")
                                  ? static_cast<uint8_t>(getFieldValue("return_number", i))
                                  : 1;
        uint8_t num_returns = fieldMap.count("number_of_returns")
                                  ? static_cast<uint8_t>(getFieldValue("number_of_returns", i))
                                  : 1;
        pt.return_byte      = (return_num & 0x0F) | ((num_returns & 0x0F) << 4);

        // Classification
        if (fieldMap.count("classification"))
        {
            pt.classification = static_cast<uint8_t>(getFieldValue("classification", i));
        }

        // User data
        if (fieldMap.count("user_data"))
        {
            pt.user_data = static_cast<uint8_t>(getFieldValue("user_data", i));
        }

        // Scan angle (scaled)
        if (fieldMap.count("scan_angle"))
        {
            double angle = getFieldValue("scan_angle", i);
            pt.scan_angle =
                static_cast<int16_t>(std::min(30000.0, std::max(-30000.0, angle * 100.0)));
        }

        // Point source ID
        if (fieldMap.count("point_source_id"))
        {
            pt.point_source_id = static_cast<uint16_t>(getFieldValue("point_source_id", i));
        }

        // GPS Time
        if (hasGPSTime)
        {
            pt.gps_time = getFieldValue("gps_time", i);
        }

        // RGB
        if (hasRGB)
        {
            double r = 0, g = 0, b = 0;

            if (fieldMap.count("red"))
            {
                const auto& info = fieldMap["red"];
                if (info.type == FieldInfo::UINT8)
                {
                    r = getFieldValue("red", i);
                }
                else if (info.type == FieldInfo::UINT16)
                {
                    r = getFieldValue("red", i) / 256.0;  // Already 16-bit
                }
                else if (info.type == FieldInfo::FLOAT)
                {
                    r = getFieldValue("red", i) * 255.0;  // Normalized 0-1
                }
            }

            if (fieldMap.count("green"))
            {
                const auto& info = fieldMap["green"];
                if (info.type == FieldInfo::UINT8)
                {
                    g = getFieldValue("green", i);
                }
                else if (info.type == FieldInfo::UINT16)
                {
                    g = getFieldValue("green", i) / 256.0;
                }
                else if (info.type == FieldInfo::FLOAT)
                {
                    g = getFieldValue("green", i) * 255.0;
                }
            }

            if (fieldMap.count("blue"))
            {
                const auto& info = fieldMap["blue"];
                if (info.type == FieldInfo::UINT8)
                {
                    b = getFieldValue("blue", i);
                }
                else if (info.type == FieldInfo::UINT16)
                {
                    b = getFieldValue("blue", i) / 256.0;
                }
                else if (info.type == FieldInfo::FLOAT)
                {
                    b = getFieldValue("blue", i) * 255.0;
                }
            }

            // Convert to 16-bit (LAS stores RGB in range 0-65535)
            pt.red   = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, r * 256.0)));
            pt.green = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, g * 256.0)));
            pt.blue  = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, b * 256.0)));
        }

        // NIR
        if (hasNIR)
        {
            pt.nir = static_cast<uint16_t>(getFieldValue("nir", i));
        }

        // Write base point data (38 bytes)
        f.write(reinterpret_cast<const char*>(&pt), sizeof(LASPointFormat8));

        // Write extra dimension data
        for (const auto& fname : extraFields)
        {
            const auto& info = fieldMap[fname];
            switch (info.type)
            {
                case FieldInfo::FLOAT:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<float>*>(info.bufPtr);
                    float val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(float));
                    break;
                }
#if MRPT_VERSION >= 0x020f03
                case FieldInfo::DOUBLE:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<double>*>(info.bufPtr);
                    double val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(double));
                    break;
                }
                case FieldInfo::UINT16:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint16_t>*>(info.bufPtr);
                    uint16_t val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(uint16_t));
                    break;
                }
                case FieldInfo::UINT8:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint8_t>*>(info.bufPtr);
                    uint8_t val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(uint8_t));
                    break;
                }
#endif
                default:
                    break;
            }
        }
    }

    f.close();
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

        std::string systemId           = argSystemId.getValue();
        std::string generatingSoftware = argGeneratingSoftware.getValue();

        // Parse export fields if specified
        std::vector<std::string> selectedFields;
        if (argExportFields.isSet())
        {
            const auto&              fieldsStr = argExportFields.getValue();
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
            auto* pts = mp2p_icp::MapToPointsMap(*layer);
            if (!pts)
            {
                continue;
            }

            std::string out = prefix + "_" + name + ".las";
            std::cout << "Exporting '" << name << "' to " << out << " (" << pts->size()
                      << " points)..." << std::endl;
            saveToLas(*pts, out, selectedFields, systemId, generatingSoftware);
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