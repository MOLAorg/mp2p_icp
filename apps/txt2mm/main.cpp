/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   txt2mm/main.cpp
 * @brief  CLI tool to convert pointclouds from CSV/TXT files to mp2p_icp mm
 * @author Jose Luis Blanco Claraco
 * @date   Feb 14, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#if MRPT_VERSION >= 0x20f00  // 2.15.0
#include <mrpt/maps/CGenericPointsMap.h>
#else
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

const char* VALID_FORMATS = "(xyz|xyzi|xyzirt|xyzrgb|xyzrgb_normalized)";

using namespace std::string_literals;

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"txt2mm"};

    TCLAP::ValueArg<std::string> argInput{
        "i",
        "input",
        "Path to input TXT or CSV file. One point per row. Columns separated "
        "by "
        "spaces or commas. See docs for supported formats.",
        true,
        "input.txt",
        "input.txt",
        cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o", "output", "Output file to write to.", true, "out.mm", "out.mm", cmd};

    TCLAP::ValueArg<std::string> argFormat{
        "f",
        "format",
        "Point cloud format. Mandatory flag.\n"s
        "Options: "s +
            VALID_FORMATS,
        true,
        "xyz",
        VALID_FORMATS,
        cmd};

    TCLAP::ValueArg<std::string> argLayer{
        "l", "layer", "Target layer name (Default: \"raw\").", false, "raw", "raw", cmd};

    TCLAP::ValueArg<int> argIndexXYZ{
        "",    "column-x", "Column index for the X coordinate in the input data (Default: 0).",
        false, 0,          "column index",
        cmd};

    TCLAP::ValueArg<int> argIndexI{
        "",
        "column-i",
        "Column index for the Intensity channel in the input data (Default: "
        "3).",
        false,
        3,
        "column index",
        cmd};

    TCLAP::ValueArg<int> argIndexR{
        "",    "column-r", "Column index for the Ring channel in the input data (Default: 4).",
        false, 4,          "column index",
        cmd};

    TCLAP::ValueArg<int> argIndexT{
        "",
        "column-t",
        "Column index for the Timestamp channel in the input data (Default: "
        "5).",
        false,
        5,
        "column index",
        cmd};

    TCLAP::ValueArg<uint64_t> argID{
        "", "id", "Metric map numeric ID (Default: none).", false, 0, "[ID]", cmd};

    TCLAP::ValueArg<std::string> argLabel{
        "", "label", "Metric map label string (Default: none).", false, "label", "[label]", cmd};
};

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        const auto& f = cli.argInput.getValue();
        ASSERT_FILE_EXISTS_(f);

        std::cout << "Reading data from '" << f << "'..." << std::endl;

        mrpt::math::CMatrixFloat data;
        data.loadFromTextFile(f);

        const size_t nRows = data.size().at(0), nCols = data.size().at(1);

        std::cout << "Done: " << nRows << " rows, " << nCols << " columns." << std::endl;

        mrpt::maps::CPointsMap::Ptr pc;
        const auto                  format = cli.argFormat.getValue();

        const auto idxX = cli.argIndexXYZ.getValue();
        const auto idxI = cli.argIndexI.getValue();
        const auto idxR = cli.argIndexR.getValue();
        const auto idxT = cli.argIndexT.getValue();

        if (format == "xyz")
        {
            ASSERT_GE_(nCols, 3U);
            pc = mrpt::maps::CSimplePointsMap::Create();
            pc->reserve(nRows);
            if (nCols > 3)
            {
                std::cout << "Warning: Only the first 3 columns from the file "
                             "will be used for the output format 'xyz'"
                          << std::endl;
            }

            for (size_t i = 0; i < nRows; i++)
            {
                pc->insertPointFast(data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
            }
        }
        else if (format == "xyzi")
        {
            ASSERT_GE_(nCols, 4U);
#if MRPT_VERSION >= 0x20f00  // 2.15.0
            auto pts = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float("intensity");
#else
            auto pts = mrpt::maps::CPointsMapXYZI::Create();
#endif
            pts->reserve(nRows);
            if (nCols > 4)
            {
                std::cout << "Warning: Only the first 4 columns from the file "
                             "will be used for the output format 'xyzi'"
                          << std::endl;
            }

            for (size_t i = 0; i < nRows; i++)
            {
                pts->insertPointFast(data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
#if MRPT_VERSION >= 0x20f00  // 2.15.0
                pts->insertPointField_float("intensity", data(i, idxI));
#else
                pts->insertPointField_Intensity(data(i, idxI));
#endif
            }

            pc = pts;
        }
        else if (format == "xyzirt")
        {
            ASSERT_GE_(nCols, 6U);
#if MRPT_VERSION >= 0x20f00  // 2.15.0
            auto pts = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float("intensity");
            pts->registerField_uint16("ring");
            pts->registerField_float("timestamp");
#else
            auto pts = mrpt::maps::CPointsMapXYZI::Create();
#endif
            pts->reserve(nRows);
            if (nCols > 6)
            {
                std::cout << "Warning: Only the first 6 columns from the file "
                             "will be used for the output format 'xyzirt'"
                          << std::endl;
            }

            for (size_t i = 0; i < nRows; i++)
            {
                pts->insertPointFast(data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
#if MRPT_VERSION >= 0x20f00  // 2.15.0
                pts->insertPointField_float("intensity", data(i, idxI));
                pts->insertPointField_uint16("ring", static_cast<uint16_t>(data(i, idxR)));
                pts->insertPointField_float("timestamp", data(i, idxT));
#else
                pts->insertPointField_Intensity(data(i, idxI));
                pts->insertPointField_Ring(data(i, idxR));
                pts->insertPointField_Timestamp(data(i, idxT));
#endif
            }

            pc = pts;
        }
        else if (format == "xyzrgb" || format == "xyzrgb_normalized")
        {
            ASSERT_GE_(nCols, 6U);
            const bool rgb_normalized = (format == "xyzrgb_normalized");
#if MRPT_VERSION >= 0x20f00  // 2.15.0
            auto pts = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float("color_r");
            pts->registerField_float("color_g");
            pts->registerField_float("color_b");
#else
            auto pts = mrpt::maps::CColouredPointsMap::Create();
#endif
            pts->reserve(nRows);
            if (nCols > 6)
            {
                std::cout << "Warning: Only the first 6 columns from the file "
                             "will be used for the output format 'xyzrgb'"
                          << std::endl;
            }

            const size_t idxRed = 3, idxGreen = 4, idxBlue = 5;

            for (size_t i = 0; i < nRows; i++)
            {
                pts->insertPointFast(data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
                // Compute RGB values first (normalized floats in [0,1]):
                float r_val, g_val, b_val;
                if (rgb_normalized)
                {
                    r_val = data(i, idxRed);
                    g_val = data(i, idxGreen);
                    b_val = data(i, idxBlue);
                }
                else
                {
                    r_val = mrpt::u8tof(static_cast<uint8_t>(data(i, idxRed)));
                    g_val = mrpt::u8tof(static_cast<uint8_t>(data(i, idxGreen)));
                    b_val = mrpt::u8tof(static_cast<uint8_t>(data(i, idxBlue)));
                }

#if MRPT_VERSION >= 0x20f00  // 2.15.0
                pts->insertPointField_float("color_r", r_val);
                pts->insertPointField_float("color_g", g_val);
                pts->insertPointField_float("color_b", b_val);
#else
                pts->insertPointField_color_R(r_val);
                pts->insertPointField_color_G(g_val);
                pts->insertPointField_color_B(b_val);
#endif
            }

            pc = pts;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Invalid --format set to '%s'. Valid values: %s", format.c_str(), VALID_FORMATS);
        }

        // Save as mm file:
        mp2p_icp::metric_map_t mm;
        mm.layers["raw"] = std::move(pc);

        if (cli.argID.isSet())
        {
            mm.id = cli.argID.getValue();
        }
        if (cli.argLabel.isSet())
        {
            mm.label = cli.argLabel.getValue();
        }

        std::cout << "Map contents: " << mm.contents_summary() << std::endl;
        std::cout << "Saving map to: " << cli.argOutput.getValue() << std::endl;

        if (!mm.save_to_file(cli.argOutput.getValue()))
        {
            THROW_EXCEPTION_FMT(
                "Error writing to target file '%s'", cli.argOutput.getValue().c_str());
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
