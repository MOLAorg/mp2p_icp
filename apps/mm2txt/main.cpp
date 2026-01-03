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
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/version.h>

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

bool saveToTxt(
    const mrpt::maps::CGenericPointsMap& pts, const std::string& fileName, bool printHeader)
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

    // header?
    if (printHeader)
    {
        // xyz are mandatory:
        mrpt::system::os::fprintf(f, "x y z ");

        for (const auto& [name, _] : floatFields)
        {
            mrpt::system::os::fprintf(f, "%.*s ", static_cast<int>(name.length()), name.data());
        }
        for (const auto& [name, _] : uint16Fields)
        {
            mrpt::system::os::fprintf(f, "%.*s ", static_cast<int>(name.length()), name.data());
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (const auto& [name, _] : doubleFields)
        {
            mrpt::system::os::fprintf(f, "%.*s ", static_cast<int>(name.length()), name.data());
        }
        for (const auto& [name, _] : uint8Fields)
        {
            mrpt::system::os::fprintf(f, "%.*s ", static_cast<int>(name.length()), name.data());
        }
#endif
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

    const std::size_t n = floatFields.begin()->second.size();

    for (size_t i = 0; i < n; i++)
    {
        // X Y Z are mandatory fields:
        mrpt::system::os::fprintf(f, "%f %f %f ", xs.at(i), ys.at(i), zs.at(i));

        for (const auto& [_, values] : floatFields)
        {
            mrpt::system::os::fprintf(f, "%f ", values.at(i));
        }
        for (const auto& [_, values] : uint16Fields)
        {
            mrpt::system::os::fprintf(f, "%u ", values.at(i));
        }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
        for (const auto& [_, values] : doubleFields)
        {
            mrpt::system::os::fprintf(f, "%lf ", values.at(i));
        }
        for (const auto& [_, values] : uint8Fields)
        {
            mrpt::system::os::fprintf(f, "%i ", static_cast<int>(values.at(i)));
        }
#endif
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

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-info] Done read map. Contents:\n" << mm.contents_summary() << std::endl;

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
            bool printHeader = true;
            saveToTxt(*genxyz, filName, printHeader);
        }
        else if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(pts); xyzirt)
        {
            xyzirt->saveXYZIRT_to_text_file(filName);
        }
        else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(pts); xyzi)
        {
            xyzi->saveXYZI_to_text_file(filName);
        }
        else
        {
            pts->save3D_to_text_file(filName);
        }
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
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
