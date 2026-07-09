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
 * @file   txt2mm/main.cpp
 * @brief  CLI tool to convert pointclouds from CSV/TXT files to mp2p_icp mm
 * @author Jose Luis Blanco Claraco
 * @date   Feb 14, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <optional>

const char* VALID_FORMATS = "(xyz|xyzi|xyzirt|xyzrgb|xyzrgb_normalized)";

// Replicated here from CGenericPointsMap until MRPT 2.15.3 is minimum required version:
constexpr static const char* POINT_FIELD_INTENSITY = "intensity";
constexpr static const char* POINT_FIELD_RING_ID   = "ring";
constexpr static const char* POINT_FIELD_TIMESTAMP = "t";

using namespace std::string_literals;

// CLI flags:
struct Cli
{
    CLI::App cmd{"txt2mm"};

    std::string argInput;
    std::string argOutput;
    std::string argFormat;
    std::string argLayer = "raw";

    int argIndexXYZ = 0;
    int argIndexI   = 3;
    int argIndexR   = 4;
    int argIndexT   = 5;

    std::optional<uint64_t> argID;
    std::string             argLabel;

    Cli()
    {
        cmd.add_option(
               "-i,--input", argInput,
               "Path to input TXT or CSV file. One point per row. Columns separated "
               "by "
               "spaces or commas. See docs for supported formats.")
            ->required();
        cmd.add_option("-o,--output", argOutput, "Output file to write to.")->required();
        cmd.add_option(
               "-f,--format", argFormat,
               "Point cloud format. Mandatory flag.\n"s + "Options: "s + VALID_FORMATS)
            ->required();
        cmd.add_option("-l,--layer", argLayer, "Target layer name (Default: \"raw\").");
        cmd.add_option(
            "--column-x", argIndexXYZ,
            "Column index for the X coordinate in the input data (Default: 0).");
        cmd.add_option(
            "--column-i", argIndexI,
            "Column index for the Intensity channel in the input data (Default: "
            "3).");
        cmd.add_option(
            "--column-r", argIndexR,
            "Column index for the Ring channel in the input data (Default: 4).");
        cmd.add_option(
            "--column-t", argIndexT,
            "Column index for the Timestamp channel in the input data (Default: "
            "5).");
        cmd.add_option("--id", argID, "Metric map numeric ID (Default: none).");
        cmd.add_option("--label", argLabel, "Metric map label string (Default: none).");
    }
};

int main(int argc, char** argv)
{
    Cli cli;

    CLI11_PARSE(cli.cmd, argc, argv);

    try
    {
        const auto& f = cli.argInput;
        ASSERT_FILE_EXISTS_(f);

        std::cout << "Reading data from '" << f << "'..." << std::endl;

        mrpt::math::CMatrixFloat data;
        data.loadFromTextFile(f);

        const size_t nRows = data.size().at(0), nCols = data.size().at(1);

        std::cout << "Done: " << nRows << " rows, " << nCols << " columns." << std::endl;

        mrpt::maps::CPointsMap::Ptr pc;
        const auto                  format = cli.argFormat;

        const auto idxX = cli.argIndexXYZ;
        const auto idxI = cli.argIndexI;
        const auto idxR = cli.argIndexR;
        const auto idxT = cli.argIndexT;

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
            auto pts = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float(POINT_FIELD_INTENSITY);
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
                pts->insertPointField_float(POINT_FIELD_INTENSITY, data(i, idxI));
            }

            pc = pts;
        }
        else if (format == "xyzirt")
        {
            ASSERT_GE_(nCols, 6U);
            auto pts = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float(POINT_FIELD_INTENSITY);
            pts->registerField_uint16(POINT_FIELD_RING_ID);
            pts->registerField_float(POINT_FIELD_TIMESTAMP);
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
                pts->insertPointField_float(POINT_FIELD_INTENSITY, data(i, idxI));
                pts->insertPointField_uint16(
                    POINT_FIELD_RING_ID, static_cast<uint16_t>(data(i, idxR)));
                pts->insertPointField_float(POINT_FIELD_TIMESTAMP, data(i, idxT));
            }

            pc = pts;
        }
        else if (format == "xyzrgb" || format == "xyzrgb_normalized")
        {
            ASSERT_GE_(nCols, 6U);
            const bool rgb_normalized = (format == "xyzrgb_normalized");
            auto       pts            = mrpt::maps::CGenericPointsMap::Create();
            pts->registerField_float("color_r");
            pts->registerField_float("color_g");
            pts->registerField_float("color_b");
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

                pts->insertPointField_float("color_r", r_val);
                pts->insertPointField_float("color_g", g_val);
                pts->insertPointField_float("color_b", b_val);
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

        if (cli.argID.has_value())
        {
            mm.id = cli.argID.value();
        }
        if (!cli.argLabel.empty())
        {
            mm.label = cli.argLabel;
        }

        std::cout << "Map contents: " << mm.contents_summary() << std::endl;
        std::cout << "Saving map to: " << cli.argOutput << std::endl;

        if (!mm.save_to_file(cli.argOutput))
        {
            THROW_EXCEPTION_FMT("Error writing to target file '%s'", cli.argOutput.c_str());
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
