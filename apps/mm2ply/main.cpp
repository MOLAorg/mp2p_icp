/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_| \___/|_| \__,_| https://github.com/MOLAorg/mola
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
#include <mrpt/version.h>

#include <fstream>
#include <iostream>

// CLI flags
TCLAP::CmdLine cmd("mm2ply");

TCLAP::ValueArg<std::string> arg_input("i", "input", "Input .mm file", true, "", "map.mm", cmd);

TCLAP::ValueArg<std::string> arg_out_prefix(
    "o", "output", "Prefix for output .ply files", false, "", "out", cmd);

TCLAP::SwitchArg arg_binary("b", "binary", "Export in binary format (default: ASCII)", cmd, false);

// ----------------------------------------------------------------
// PLY Export Logic using CPointsMap field-generic API
// ----------------------------------------------------------------
void saveToPly(const mrpt::maps::CPointsMap& pc, const std::string& filename, bool binary)
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

    // 2. Write Header
    f << "ply" << std::endl;
    f << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0" << std::endl;
    f << "element vertex " << N << std::endl;
    f << "property float x\nproperty float y\nproperty float z" << std::endl;

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

    for (const auto& n : f_names)
    {
        if (n != "x" && n != "y" && n != "z")
        {
            f << "property float " << mapPlyName(n) << "\n";
        }
    }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    for (const auto& n : d_names)
    {
        f << "property double " << mapPlyName(n) << "\n";
    }
    for (const auto& n : u16_names)
    {
        f << "property ushort " << mapPlyName(n) << "\n";
    }
    for (const auto& n : u8_names)
    {
        f << "property uchar " << mapPlyName(n) << "\n";
    }
#endif
    f << "end_header\n";

    // 3. Prepare Accessors
    const auto &xs = pc.getPointsBufferRef_x(), &ys = pc.getPointsBufferRef_y(),
               &zs = pc.getPointsBufferRef_z();

    std::vector<const mrpt::aligned_std_vector<float>*> f_bufs;
    for (const auto& n : f_names)
    {
        if (n != "x" && n != "y" && n != "z")
        {
            f_bufs.push_back(pc.getPointsBufferRef_float_field(n));
        }
    }

#if MRPT_VERSION >= 0x020f03  // 2.15.3
    std::vector<const mrpt::aligned_std_vector<double>*> d_bufs;
    for (const auto& n : d_names)
    {
        d_bufs.push_back(pc.getPointsBufferRef_double_field(n));
    }

    std::vector<const mrpt::aligned_std_vector<uint16_t>*> u16_bufs;
    for (const auto& n : u16_names)
    {
        u16_bufs.push_back(pc.getPointsBufferRef_uint16_field(n));
    }

    std::vector<const mrpt::aligned_std_vector<uint8_t>*> u8_bufs;
    for (const auto& n : u8_names)
    {
        u8_bufs.push_back(pc.getPointsBufferRef_uint8_field(n));
    }
#endif

    // 4. Data Loop
    auto write_val = [&](auto val)
    {
        if (binary)
        {
            f.write(reinterpret_cast<const char*>(&val), sizeof(val));
        }
        else
        {
            f << +val << " ";
        }
    };

    for (size_t i = 0; i < N; ++i)
    {
        write_val(xs[i]);
        write_val(ys[i]);
        write_val(zs[i]);
        for (auto b : f_bufs)
        {
            write_val((*b)[i]);
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (auto b : d_bufs)
        {
            write_val((*b)[i]);
        }
        for (auto b : u16_bufs)
        {
            write_val((*b)[i]);
        }
        for (auto b : u8_bufs)
        {
            write_val((*b)[i]);
        }
#endif
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

        for (const auto& [name, layer] : mm.layers)
        {
            auto* pts = mp2p_icp::MapToPointsMap(*layer);
            if (!pts)
            {
                continue;
            }

            std::string out = prefix + "_" + name + ".ply";
            std::cout << "Exporting '" << name << "' to " << out << "..." << std::endl;
            saveToPly(*pts, out, arg_binary.getValue());
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}