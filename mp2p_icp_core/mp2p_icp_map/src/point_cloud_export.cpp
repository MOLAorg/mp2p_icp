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
 * @file   point_cloud_export.cpp
 * @brief  Writers of point clouds, with all their per-point fields, to PLY, LAS and TXT files
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mp2p_icp/point_cloud_export.h>
#include <mrpt/core/format.h>
#include <mrpt/system/os.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

namespace
{
bool contains(const std::vector<std::string>& names, const std::string& name)
{
    return std::find(names.begin(), names.end(), name) != names.end();
}

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

#pragma pack(pop)

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

// WKT2:2019 string for EPSG:4979 (WGS 84 geographic 3D).
// IMPORTANT: Axis order is longitude-first (ORDER[1]) to match LAS point data
// layout where X=longitude, Y=latitude, Z=ellipsoidal height. The canonical
// EPSG:4979 definition uses lat-first, but that would cause CRS-aware readers
// to swap X/Y. WKT1 cannot represent 3D geographic CRS, so WKT2 is required.
constexpr const char* kWKT_EPSG4979 =
    "GEOGCRS[\"WGS 84\","
    "ENSEMBLE[\"World Geodetic System 1984 ensemble\","
    "MEMBER[\"World Geodetic System 1984 (Transit)\"],"
    "MEMBER[\"World Geodetic System 1984 (G730)\"],"
    "MEMBER[\"World Geodetic System 1984 (G873)\"],"
    "MEMBER[\"World Geodetic System 1984 (G1150)\"],"
    "MEMBER[\"World Geodetic System 1984 (G1674)\"],"
    "MEMBER[\"World Geodetic System 1984 (G1762)\"],"
    "MEMBER[\"World Geodetic System 1984 (G2139)\"],"
    "ELLIPSOID[\"WGS 84\",6378137,298.257223563,"
    "LENGTHUNIT[\"metre\",1]],"
    "ENSEMBLEACCURACY[2.0]],"
    "PRIMEM[\"Greenwich\",0,ANGLEUNIT[\"degree\",0.0174532925199433]],"
    "CS[ellipsoidal,3],"
    "AXIS[\"geodetic longitude (Lon)\",east,ORDER[1],ANGLEUNIT[\"degree\",0.0174532925199433]],"
    "AXIS[\"geodetic latitude (Lat)\",north,ORDER[2],ANGLEUNIT[\"degree\",0.0174532925199433]],"
    "AXIS[\"ellipsoidal height (h)\",up,ORDER[3],LENGTHUNIT[\"metre\",1]],"
    "USAGE[SCOPE[\"Geodesy.\"],AREA[\"World.\"],"
    "BBOX[-90,-180,90,180]],"
    "ID[\"EPSG\",4979]]";

}  // namespace

// ----------------------------------------------------------------
// PLY
// ----------------------------------------------------------------
void mp2p_icp::save_point_cloud_to_ply(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const ply_export_options_t& options)
{
    const bool  binary              = options.binary;
    const bool  ignoreMissingFields = options.ignore_missing_fields;
    const auto& T_enu_to_map        = options.transform;

    std::ofstream f;
    f.open(filename, binary ? std::ios::binary : std::ios::out);
    if (!f.is_open())
    {
        throw std::runtime_error("Could not open file: " + filename);
    }

    const size_t N = pc.size();

    // 1. Discover Fields
    const auto f_names   = pc.getPointFieldNames_float();
    const auto d_names   = pc.getPointFieldNames_double();
    const auto u16_names = pc.getPointFieldNames_uint16();
    const auto u8_names  = pc.getPointFieldNames_uint8();

    // Determine which fields to export and in what order
    std::vector<std::string> fieldsToExport;

    if (options.fields.empty())
    {
        // Export all fields in default order
        fieldsToExport.push_back("x");
        fieldsToExport.push_back("y");
        fieldsToExport.push_back("z");

        for (const auto& n : f_names)
        {
            if (n != "x" && n != "y" && n != "z")
            {
                fieldsToExport.push_back(n);
            }
        }
        for (const auto& n : d_names)
        {
            fieldsToExport.push_back(n);
        }
        for (const auto& n : u16_names)
        {
            fieldsToExport.push_back(n);
        }
        for (const auto& n : u8_names)
        {
            fieldsToExport.push_back(n);
        }
    }
    else
    {
        // Use selected fields in specified order
        fieldsToExport = options.fields;
    }

    // 2. Write Header
    f << "ply" << std::endl;
    f << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0" << std::endl;
    f << "element vertex " << N << std::endl;

    auto mapPlyName = [](const std::string& n) -> std::string
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
        return n;
    };

    auto getFieldType = [&](const std::string& fieldName) -> std::string
    {
        if (fieldName == "x" || fieldName == "y" || fieldName == "z")
        {
            return "float";
        }
        if (contains(f_names, fieldName))
        {
            return "float";
        }
        if (contains(d_names, fieldName))
        {
            return "double";
        }
        if (contains(u16_names, fieldName))
        {
            return "ushort";
        }
        if (contains(u8_names, fieldName))
        {
            return "uchar";
        }
        return "float";  // default
    };

    for (const auto& fieldName : fieldsToExport)
    {
        f << "property " << getFieldType(fieldName) << " " << mapPlyName(fieldName) << "\n";
    }

    f << "end_header\n";

    // 3. Prepare Accessors
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

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
            acc.bufPtr = (fieldName == "x")   ? static_cast<const void*>(&xs)
                         : (fieldName == "y") ? static_cast<const void*>(&ys)
                                              : static_cast<const void*>(&zs);
        }
        else if (contains(f_names, fieldName))
        {
            acc.type   = FieldAccessor::FLOAT;
            acc.bufPtr = pc.getPointsBufferRef_float_field(fieldName);
        }
        else if (contains(d_names, fieldName))
        {
            acc.type   = FieldAccessor::DOUBLE;
            acc.bufPtr = pc.getPointsBufferRef_double_field(fieldName);
        }
        else if (contains(u16_names, fieldName))
        {
            acc.type   = FieldAccessor::UINT16;
            acc.bufPtr = pc.getPointsBufferRef_uint16_field(fieldName);
        }
        else if (contains(u8_names, fieldName))
        {
            acc.type   = FieldAccessor::UINT8;
            acc.bufPtr = pc.getPointsBufferRef_uint8_field(fieldName);
        }
        else
        {
            if (!ignoreMissingFields)
            {
                throw std::runtime_error("Field not found: " + fieldName);
            }
            // Mark this accessor as invalid (nullptr) - will output zeros
            acc.bufPtr = nullptr;
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
        // On-the-fly coordinate transform if exporting in ENU frame
        double tx = 0;
        double ty = 0;
        double tz = 0;
        if (T_enu_to_map.has_value())
        {
            T_enu_to_map->composePoint(xs[i], ys[i], zs[i], tx, ty, tz);
        }

        for (const auto& acc : accessors)
        {
            // Handle missing fields (when ignoreMissingFields is true)
            if (acc.bufPtr == nullptr)
            {
                // Output zero for missing fields
                write_val(0.0f, "%.8e");
                continue;
            }

            // Use transformed coordinates if in ENU mode
            if (T_enu_to_map.has_value() && (acc.name == "x" || acc.name == "y" || acc.name == "z"))
            {
                const float val =
                    static_cast<float>(acc.name == "x" ? tx : (acc.name == "y" ? ty : tz));
                write_val(val, "%.8e");
                continue;
            }

            switch (acc.type)
            {
                case FieldAccessor::FLOAT:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<float>*>(acc.bufPtr);
                    write_val(buf[i], "%.8e");
                    break;
                }
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
                default:
                    throw std::runtime_error("Error: unknown field type");
            }
        }
        if (!binary)
        {
            f << "\n";
        }
    }
}

// ----------------------------------------------------------------
// LAS 1.4, with extra dimensions
// ----------------------------------------------------------------
void mp2p_icp::save_point_cloud_to_las(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const las_export_options_t& options)
{
    const auto& T_enu_to_map   = options.transform;
    const bool  exportGeodetic = options.geodetic;
    const auto& geodetic       = options.georeferencing;
    const bool  verbose        = options.verbose;

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
    const auto f_names   = pc.getPointFieldNames_float();
    const auto d_names   = pc.getPointFieldNames_double();
    const auto u16_names = pc.getPointFieldNames_uint16();
    const auto u8_names  = pc.getPointFieldNames_uint8();

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

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    // Detect pre-existing per-point geodetic fields
    bool hasPerPointGeodetic = false;

    const mrpt::aligned_std_vector<double>* latBuf = nullptr;
    const mrpt::aligned_std_vector<double>* lonBuf = nullptr;
    const mrpt::aligned_std_vector<double>* altBuf = nullptr;

    if (exportGeodetic)
    {
        if (contains(d_names, "latitude") && contains(d_names, "longitude") &&
            contains(d_names, "altitude"))
        {
            latBuf = pc.getPointsBufferRef_double_field("latitude");
            lonBuf = pc.getPointsBufferRef_double_field("longitude");
            altBuf = pc.getPointsBufferRef_double_field("altitude");
            if (latBuf && lonBuf && altBuf && latBuf->size() >= N && lonBuf->size() >= N &&
                altBuf->size() >= N)
            {
                hasPerPointGeodetic = true;
                if (verbose)
                {
                    std::cout
                        << "  [geodetic] Using pre-existing per-point latitude/longitude/altitude "
                           "fields."
                        << std::endl;
                }
            }
        }
        if (!hasPerPointGeodetic && verbose)
        {
            std::cout << "  [geodetic] Per-point geodetic fields not found; computing on the fly."
                      << std::endl;
        }
    }

    // Preflight: geodetic mode requires either per-point buffers or a georeferencing context
    // for on-the-fly conversion. Fail early with a clear message instead of a null deref later.
    if (exportGeodetic && !hasPerPointGeodetic && !geodetic.has_value())
    {
        throw std::runtime_error(
            "Cannot export in geodetic mode: no per-point latitude/longitude/altitude fields "
            "found and no georeferencing context was provided for on-the-fly conversion.");
    }

    // 2. Collect all fields to export
    std::vector<std::string> fieldsToExport;

    if (options.fields.empty())
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
                fieldsToExport.push_back(n);
            }
        }
        for (const auto& n : d_names)
        {
            fieldsToExport.push_back(n);
        }
        for (const auto& n : u16_names)
        {
            fieldsToExport.push_back(n);
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
                fieldsToExport.push_back(n);
            }
        }
    }
    else
    {
        fieldsToExport = options.fields;
    }

    // 3. Process each field
    bool hasRGB       = false;
    bool hasNIR       = false;
    bool hasIntensity = false;
    bool hasGPSTime   = false;

    // Looks for a field among the names of the given type:
    const auto setIfFound = [&pc](
                                FieldInfo& info, const std::vector<std::string>& names,
                                const std::string& name, FieldInfo::Type type) -> bool
    {
        if (!contains(names, name))
        {
            return false;
        }
        info.type = type;
        switch (type)
        {
            case FieldInfo::FLOAT:
                info.bufPtr = pc.getPointsBufferRef_float_field(name);
                info.size   = sizeof(float);
                break;
            case FieldInfo::DOUBLE:
                info.bufPtr = pc.getPointsBufferRef_double_field(name);
                info.size   = sizeof(double);
                break;
            case FieldInfo::UINT16:
                info.bufPtr = pc.getPointsBufferRef_uint16_field(name);
                info.size   = sizeof(uint16_t);
                break;
            case FieldInfo::UINT8:
                info.bufPtr = pc.getPointsBufferRef_uint8_field(name);
                info.size   = sizeof(uint8_t);
                break;
            default:
                return false;
        }
        return true;
    };

    for (const auto& fieldName : fieldsToExport)
    {
        FieldInfo info;
        info.name = fieldName;

        const bool isStandardField = contains(standardFields, fieldName);

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
            if (!setIfFound(info, f_names, fieldName, FieldInfo::FLOAT))
            {
                std::cerr << "Warning: intensity field not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "gps_time")
        {
            hasGPSTime = true;
            if (!setIfFound(info, d_names, fieldName, FieldInfo::DOUBLE))
            {
                std::cerr << "Warning: gps_time field not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "red" || fieldName == "green" || fieldName == "blue")
        {
            hasRGB = false;
            // Try uint16, then uint8, then float (normalized 0-1):
            const bool found =
                setIfFound(info, u16_names, fieldName, FieldInfo::UINT16) ||
                setIfFound(info, u8_names, "color_" + fieldName.substr(0, 1), FieldInfo::UINT8) ||
                setIfFound(
                    info, f_names, "color_" + fieldName.substr(0, 1) + "f", FieldInfo::FLOAT);
            if (!found)
            {
                std::cerr << "Warning: RGB field '" << fieldName << "' not found." << std::endl;
                continue;
            }
            hasRGB = true;
        }
        else if (fieldName == "nir")
        {
            hasNIR = true;
            if (!setIfFound(info, u16_names, fieldName, FieldInfo::UINT16))
            {
                std::cerr << "Warning: NIR field not found." << std::endl;
                continue;
            }
        }
        else if (
            fieldName == "classification" || fieldName == "return_number" ||
            fieldName == "number_of_returns" || fieldName == "user_data")
        {
            if (!setIfFound(info, u8_names, fieldName, FieldInfo::UINT8))
            {
                std::cerr << "Warning: Field '" << fieldName << "' not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "scan_angle")
        {
            if (!setIfFound(info, f_names, fieldName, FieldInfo::FLOAT))
            {
                std::cerr << "Warning: scan_angle field not found." << std::endl;
                continue;
            }
        }
        else if (fieldName == "point_source_id")
        {
            if (!setIfFound(info, u16_names, fieldName, FieldInfo::UINT16))
            {
                std::cerr << "Warning: point_source_id field not found." << std::endl;
                continue;
            }
        }
        // Non-standard field - will be added as extra dimension
        else if (!isStandardField)
        {
            const bool found = setIfFound(info, f_names, fieldName, FieldInfo::FLOAT) ||
                               setIfFound(info, d_names, fieldName, FieldInfo::DOUBLE) ||
                               setIfFound(info, u16_names, fieldName, FieldInfo::UINT16) ||
                               setIfFound(info, u8_names, fieldName, FieldInfo::UINT8);
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

    // Helper lambda: compute geodetic coords for a point
    // Returns {longitude, latitude, altitude} (LAS X=lon, Y=lat, Z=alt)
    auto pointToGeodetic = [&](size_t i) -> mrpt::math::TPoint3D
    {
        if (hasPerPointGeodetic)
        {
            return {(*lonBuf)[i], (*latBuf)[i], (*altBuf)[i]};
        }
        // On-the-fly conversion: map -> ENU -> geocentric -> geodetic
        const mrpt::math::TPoint3D ptENU =
            geodetic->T_enu_to_map.mean.composePoint({xs[i], ys[i], zs[i]});

        mrpt::topography::TGeocentricCoords geocentric;
        mrpt::topography::ENUToGeocentric(
            ptENU, geodetic->geo_coord, geocentric,
            mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

        mrpt::topography::TGeodeticCoords geod;
        mrpt::topography::geocentricToGeodetic(
            geocentric, geod, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

        return {geod.lon.getDecimalValue(), geod.lat.getDecimalValue(), geod.height};
    };

    // Cache on-the-fly geodetic results so the write loop does not recompute them.
    // The fast-path (hasPerPointGeodetic) reads existing buffers cheaply and needs no cache.
    std::vector<mrpt::math::TPoint3D> geodeticCache;
    if (exportGeodetic && !hasPerPointGeodetic)
    {
        geodeticCache.resize(N);
    }

    for (size_t i = 0; i < N; ++i)
    {
        double px = 0;
        double py = 0;
        double pz = 0;
        if (exportGeodetic)
        {
            // In geodetic mode: X=longitude, Y=latitude, Z=altitude
            const mrpt::math::TPoint3D g = pointToGeodetic(i);
            if (!hasPerPointGeodetic)
            {
                geodeticCache[i] = g;
            }
            px = g.x;
            py = g.y;
            pz = g.z;
        }
        else
        {
            px = xs[i];
            py = ys[i];
            pz = zs[i];
            if (T_enu_to_map.has_value())
            {
                T_enu_to_map->composePoint(xs[i], ys[i], zs[i], px, py, pz);
            }
        }
        min_x = std::min(min_x, px);
        max_x = std::max(max_x, px);
        min_y = std::min(min_y, py);
        max_y = std::max(max_y, py);
        min_z = std::min(min_z, pz);
        max_z = std::max(max_z, pz);
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
    std::strncpy(header.system_identifier, options.system_id.c_str(), 31);
    header.system_identifier[31] = '\0';
    std::strncpy(header.generating_software, options.generating_software.c_str(), 31);
    header.generating_software[31] = '\0';

    // Bit 4 (0x10) signals a WKT CRS VLR is present; only set it when we actually write one.
    if (!exportGeodetic)
    {
        header.global_encoding &= ~static_cast<uint16_t>(0x10);
    }

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

    // In geodetic mode, use finer scale for degrees and set appropriate offsets
    if (exportGeodetic)
    {
        // Scale factor tradeoffs for geodetic coordinates:
        // LAS stores coordinates as int32, so the usable range from the offset
        // is +/-(2^31 * scale), about +/-2.147e9 * scale.
        //
        // At 1e-8 degrees: ~1.1mm precision at equator (cos(lat) shrinks this
        // towards poles). Range: +/-21.47 degrees from offset, more than enough
        // for any single point cloud extent.
        //
        // At 1e-7 degrees: ~1.1cm precision, +/-214.7 deg range (exceeds globe).
        // At 1e-9 degrees: ~0.11mm precision, but only +/-2.15 deg range (~239km
        // at equator), which could overflow for very large-area datasets.
        //
        // 1e-8 is the sweet spot: mm-level precision with ample range (~2,388km).
        header.x_scale_factor = 1e-8;  // longitude
        header.y_scale_factor = 1e-8;  // latitude
        header.z_scale_factor = 0.001;  // altitude in meters, 1mm precision
    }

    // Verify that the coordinate range fits in int32 with the chosen scale.
    // LAS stores coordinates as (coord - offset) / scale cast to int32, so the
    // maximum representable span from the offset is INT32_MAX * scale.
    {
        const double int32_max     = static_cast<double>(std::numeric_limits<int32_t>::max());
        const double max_span_x    = int32_max * header.x_scale_factor;
        const double max_span_y    = int32_max * header.y_scale_factor;
        const double max_span_z    = int32_max * header.z_scale_factor;
        const double actual_span_x = max_x - min_x;
        const double actual_span_y = max_y - min_y;
        const double actual_span_z = max_z - min_z;
        if (actual_span_x > max_span_x || actual_span_y > max_span_y || actual_span_z > max_span_z)
        {
            throw std::runtime_error(
                "Point cloud extent overflows int32 with the chosen LAS scale factors.\n"
                "  X span: " +
                std::to_string(actual_span_x) + " (max " + std::to_string(max_span_x) + ")\n" +
                "  Y span: " + std::to_string(actual_span_y) + " (max " +
                std::to_string(max_span_y) + ")\n" + "  Z span: " + std::to_string(actual_span_z) +
                " (max " + std::to_string(max_span_z) + ")\n" +
                "Use a coarser scale factor or split the point cloud into smaller tiles.");
        }
    }

    // Calculate VLR sizes
    uint32_t numVLRs       = 0;
    uint32_t vlrTotalBytes = 0;

    // WKT CRS VLR (geodetic mode)
    const size_t wktLen = exportGeodetic ? (std::strlen(kWKT_EPSG4979) + 1) : 0;  // +1 null
    if (exportGeodetic)
    {
        numVLRs++;
        vlrTotalBytes += sizeof(VLRHeader) + wktLen;
    }

    // Extra Bytes VLR
    if (extraBytesPerPoint > 0)
    {
        numVLRs++;
        vlrTotalBytes += sizeof(VLRHeader) + (extraFields.size() * 192);
    }

    header.number_of_variable_length_records = numVLRs;
    header.offset_to_point_data              = 375 + vlrTotalBytes;

    if (verbose)
    {
        std::cout << "LAS 1.4 Point Format 8:" << std::endl;
        std::cout << "  Base size: 38 bytes" << std::endl;
        std::cout << "  Extra dimensions: " << extraFields.size() << " (" << extraBytesPerPoint
                  << " bytes)" << std::endl;
        std::cout << "  Total record size: " << header.point_data_record_length << " bytes"
                  << std::endl;
    }

    // Write header
    f.write(reinterpret_cast<const char*>(&header), sizeof(LASHeader_1_4));

    // 7a. Write WKT CRS VLR (geodetic mode)
    if (exportGeodetic)
    {
        VLRHeader wktVlr;
        std::strncpy(wktVlr.user_id, "LASF_Projection", 16);
        wktVlr.record_id     = 2112;  // OGC WKT CRS
        wktVlr.record_length = static_cast<uint16_t>(wktLen);
        std::strncpy(wktVlr.description, "OGC WKT CRS (EPSG:4979)", 31);

        f.write(reinterpret_cast<const char*>(&wktVlr), sizeof(VLRHeader));
        f.write(kWKT_EPSG4979, static_cast<std::streamsize>(wktLen));

        if (verbose)
        {
            std::cout << "  [geodetic] Embedded EPSG:4979 WKT CRS in LAS file." << std::endl;
        }
    }

    // 7b. Write VLR for Extra Bytes if needed
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
        const auto it = fieldMap.find(fieldName);
        if (it == fieldMap.end())
        {
            return 0.0;
        }
        const auto& info = it->second;

        switch (info.type)
        {
            case FieldInfo::FLOAT:
            {
                const auto& buf = *static_cast<const mrpt::aligned_std_vector<float>*>(info.bufPtr);
                return static_cast<double>(buf[i]);
            }
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
            default:
                return 0.0;
        }
    };

    // Converts a color channel into the 0-255 range, by its storage type:
    auto getColorValue = [&](const std::string& fieldName, size_t i) -> double
    {
        const auto it = fieldMap.find(fieldName);
        if (it == fieldMap.end())
        {
            return 0.0;
        }
        switch (it->second.type)
        {
            case FieldInfo::UINT8:
                return getFieldValue(fieldName, i);
            case FieldInfo::UINT16:
                return getFieldValue(fieldName, i) / 256.0;  // Already 16-bit
            case FieldInfo::FLOAT:
                return getFieldValue(fieldName, i) * 255.0;  // Normalized 0-1
            default:
                return 0.0;
        }
    };

    // 9. Write point data
    for (size_t i = 0; i < N; ++i)
    {
        LASPointFormat8 pt = {};

        // Coordinate transform depending on export mode
        double px = 0;
        double py = 0;
        double pz = 0;
        if (exportGeodetic)
        {
            const mrpt::math::TPoint3D g =
                !hasPerPointGeodetic ? geodeticCache[i] : pointToGeodetic(i);
            px = g.x;  // longitude
            py = g.y;  // latitude
            pz = g.z;  // altitude
        }
        else
        {
            px = xs[i];
            py = ys[i];
            pz = zs[i];
            if (T_enu_to_map.has_value())
            {
                T_enu_to_map->composePoint(xs[i], ys[i], zs[i], px, py, pz);
            }
        }

        // Scale coordinates
        pt.x = static_cast<int32_t>((px - header.x_offset) / header.x_scale_factor);
        pt.y = static_cast<int32_t>((py - header.y_offset) / header.y_scale_factor);
        pt.z = static_cast<int32_t>((pz - header.z_offset) / header.z_scale_factor);

        // Intensity
        if (hasIntensity)
        {
            const double val = getFieldValue("intensity", i);
            pt.intensity     = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, val)));
        }

        // Return byte (4 bits each for return number and total returns)
        const uint8_t return_num = fieldMap.count("return_number")
                                       ? static_cast<uint8_t>(getFieldValue("return_number", i))
                                       : 1;
        const uint8_t num_returns =
            fieldMap.count("number_of_returns")
                ? static_cast<uint8_t>(getFieldValue("number_of_returns", i))
                : 1;
        pt.return_byte = (return_num & 0x0F) | ((num_returns & 0x0F) << 4);

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
            const double angle = getFieldValue("scan_angle", i);
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
            const double r = getColorValue("red", i);
            const double g = getColorValue("green", i);
            const double b = getColorValue("blue", i);

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
                    const float val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(float));
                    break;
                }
                case FieldInfo::DOUBLE:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<double>*>(info.bufPtr);
                    const double val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(double));
                    break;
                }
                case FieldInfo::UINT16:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint16_t>*>(info.bufPtr);
                    const uint16_t val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(uint16_t));
                    break;
                }
                case FieldInfo::UINT8:
                {
                    const auto& buf =
                        *static_cast<const mrpt::aligned_std_vector<uint8_t>*>(info.bufPtr);
                    const uint8_t val = buf[i];
                    f.write(reinterpret_cast<const char*>(&val), sizeof(uint8_t));
                    break;
                }
                default:
                    break;
            }
        }
    }

    f.close();
}

// ----------------------------------------------------------------
// TXT
// ----------------------------------------------------------------
void mp2p_icp::save_point_cloud_to_txt(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const txt_export_options_t& options)
{
    const auto& T_enu_to_map = options.transform;

    FILE* f = mrpt::system::os::fopen(filename.c_str(), "wt");
    if (!f)
    {
        throw std::runtime_error("Could not open file: " + filename);
    }

    // Per-point fields, besides x,y,z:
    const auto floatNames  = pc.getPointFieldNames_float_except_xyz();
    const auto uint16Names = pc.getPointFieldNames_uint16();
    const auto doubleNames = pc.getPointFieldNames_double();
    const auto uint8Names  = pc.getPointFieldNames_uint8();

    // Determine which fields to export and in what order
    std::vector<std::string> fieldsToExport;

    if (options.fields.empty())
    {
        // Export all fields in default order
        fieldsToExport = {"x", "y", "z"};
        for (const auto* names : {&floatNames, &uint16Names, &doubleNames, &uint8Names})
        {
            fieldsToExport.insert(fieldsToExport.end(), names->begin(), names->end());
        }
    }
    else
    {
        // Use selected fields in specified order
        fieldsToExport = options.fields;
    }

    // header?
    if (options.header)
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

    // Resolve the buffer of each field once:
    struct FieldAccessor
    {
        enum
        {
            X,
            Y,
            Z,
            FLOAT,
            UINT16,
            DOUBLE,
            UINT8,
            MISSING
        } type = MISSING;

        const void* bufPtr = nullptr;
    };
    std::vector<FieldAccessor> accessors;
    for (const auto& fieldName : fieldsToExport)
    {
        FieldAccessor acc;
        if (fieldName == "x")
        {
            acc.type = FieldAccessor::X;
        }
        else if (fieldName == "y")
        {
            acc.type = FieldAccessor::Y;
        }
        else if (fieldName == "z")
        {
            acc.type = FieldAccessor::Z;
        }
        else if (contains(floatNames, fieldName))
        {
            acc.type   = FieldAccessor::FLOAT;
            acc.bufPtr = pc.getPointsBufferRef_float_field(fieldName);
        }
        else if (contains(uint16Names, fieldName))
        {
            acc.type   = FieldAccessor::UINT16;
            acc.bufPtr = pc.getPointsBufferRef_uint16_field(fieldName);
        }
        else if (contains(doubleNames, fieldName))
        {
            acc.type   = FieldAccessor::DOUBLE;
            acc.bufPtr = pc.getPointsBufferRef_double_field(fieldName);
        }
        else if (contains(uint8Names, fieldName))
        {
            acc.type   = FieldAccessor::UINT8;
            acc.bufPtr = pc.getPointsBufferRef_uint8_field(fieldName);
        }
        if (acc.type >= FieldAccessor::FLOAT && acc.bufPtr == nullptr)
        {
            acc.type = FieldAccessor::MISSING;
        }
        accessors.push_back(acc);
    }

    // print fields:
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const std::size_t n = xs.size();

    for (size_t i = 0; i < n; i++)
    {
        // On-the-fly coordinate transform if exporting in ENU frame
        double tx = xs[i];
        double ty = ys[i];
        double tz = zs[i];
        if (T_enu_to_map.has_value())
        {
            T_enu_to_map->composePoint(xs[i], ys[i], zs[i], tx, ty, tz);
        }

        for (size_t fieldIdx = 0; fieldIdx < accessors.size(); fieldIdx++)
        {
            const auto& acc = accessors[fieldIdx];
            switch (acc.type)
            {
                case FieldAccessor::X:
                    mrpt::system::os::fprintf(f, "%.8f", tx);
                    break;
                case FieldAccessor::Y:
                    mrpt::system::os::fprintf(f, "%.8f", ty);
                    break;
                case FieldAccessor::Z:
                    mrpt::system::os::fprintf(f, "%.8f", tz);
                    break;
                case FieldAccessor::FLOAT:
                    mrpt::system::os::fprintf(
                        f, "%.8e",
                        (*static_cast<const mrpt::aligned_std_vector<float>*>(acc.bufPtr))[i]);
                    break;
                case FieldAccessor::UINT16:
                    mrpt::system::os::fprintf(
                        f, "%u",
                        (*static_cast<const mrpt::aligned_std_vector<uint16_t>*>(acc.bufPtr))[i]);
                    break;
                case FieldAccessor::DOUBLE:
                    mrpt::system::os::fprintf(
                        f, "%.16le",
                        (*static_cast<const mrpt::aligned_std_vector<double>*>(acc.bufPtr))[i]);
                    break;
                case FieldAccessor::UINT8:
                    mrpt::system::os::fprintf(
                        f, "%i",
                        static_cast<int>((*static_cast<const mrpt::aligned_std_vector<uint8_t>*>(
                            acc.bufPtr))[i]));
                    break;
                default:
                    // Field not found
                    mrpt::system::os::fprintf(f, "0");
                    break;
            }

            // Add separator or newline
            mrpt::system::os::fprintf(f, "%s", (fieldIdx + 1 < accessors.size()) ? " " : "");
        }
        mrpt::system::os::fprintf(f, "\n");
    }

    mrpt::system::os::fclose(f);
}
