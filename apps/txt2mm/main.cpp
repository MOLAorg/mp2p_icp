/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   txt2mm/main.cpp
 * @brief  CLI tool to convert pointclouds from CSV/TXT files to mp2p_icp mm
 * @author Jose Luis Blanco Claraco
 * @date   Feb 14, 2024
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

// CLI flags:
static TCLAP::CmdLine cmd("txt2mm");

static TCLAP::ValueArg<std::string> argInput(
    "i", "input",
    "Path to input TXT or CSV file. One point per row. Columns separated by "
    "spaces or commas. See docs for supported formats.",
    true, "input.txt", "input.txt", cmd);

static TCLAP::ValueArg<std::string> argOutput(
    "o", "output", "Output file to write to.", true, "out.mm", "out.mm", cmd);

static TCLAP::ValueArg<std::string> argFormat(
    "f", "format",
    "Point cloud format. Mandatory flag.\n"
    "Options: (xyz|xyzi|xyzirt)",
    true, "xyz", "(xyz|xyzi|xyzirt)", cmd);

static TCLAP::ValueArg<std::string> argLayer(
    "l", "layer", "Target layer name (Default: \"raw\").", false, "raw", "raw",
    cmd);

static TCLAP::ValueArg<int> argIndexXYZ(
    "", "column-x",
    "Column index for the X coordinate in the input data (Default: 0).", false,
    0, "column index", cmd);

static TCLAP::ValueArg<int> argIndexI(
    "", "column-i",
    "Column index for the Intensity channel in the input data (Default: 3).",
    false, 3, "column index", cmd);

static TCLAP::ValueArg<int> argIndexR(
    "", "column-r",
    "Column index for the Ring channel in the input data (Default: 4).", false,
    4, "column index", cmd);

static TCLAP::ValueArg<int> argIndexT(
    "", "column-t",
    "Column index for the Timestamp channel in the input data (Default: 5).",
    false, 5, "column index", cmd);

static TCLAP::ValueArg<uint64_t> argID(
    "", "id", "Metric map numeric ID (Default: none).", false, 0, "[ID]", cmd);

static TCLAP::ValueArg<std::string> argLabel(
    "", "label", "Metric map label string (Default: none).", false, "label",
    "[label]", cmd);

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        const auto& f = argInput.getValue();
        ASSERT_FILE_EXISTS_(f);

        std::cout << "Reading data from '" << f << "'..." << std::endl;

        mrpt::math::CMatrixFloat data;
        data.loadFromTextFile(f);

        const size_t nRows = data.size().at(0), nCols = data.size().at(1);

        std::cout << "Done: " << nRows << " rows, " << nCols << " columns."
                  << std::endl;

        mrpt::maps::CPointsMap::Ptr pc;
        const auto                  format = argFormat.getValue();

        const auto idxX = argIndexXYZ.getValue();
        const auto idxI = argIndexI.getValue();
        const auto idxR = argIndexR.getValue();
        const auto idxT = argIndexT.getValue();

        if (format == "xyz")
        {
            ASSERT_GE_(nCols, 3U);
            pc = mrpt::maps::CSimplePointsMap::Create();
            pc->reserve(nRows);
            if (nCols > 3)
                std::cout << "Warning: Only the first 3 columns from the file "
                             "will be used for the output format 'xyz'"
                          << std::endl;

            for (size_t i = 0; i < nRows; i++)
                pc->insertPointFast(
                    data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
        }
        else if (format == "xyzi")
        {
            ASSERT_GE_(nCols, 4U);
            auto pts = mrpt::maps::CPointsMapXYZI::Create();
            pts->reserve(nRows);
            if (nCols > 4)
                std::cout << "Warning: Only the first 4 columns from the file "
                             "will be used for the output format 'xyzi'"
                          << std::endl;

            for (size_t i = 0; i < nRows; i++)
            {
                pts->insertPointFast(
                    data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
                pts->insertPointField_Intensity(data(i, idxI));
            }

            pc = pts;
        }
        else if (format == "xyzirt")
        {
            ASSERT_GE_(nCols, 6U);
            auto pts = mrpt::maps::CPointsMapXYZI::Create();
            pts->reserve(nRows);
            if (nCols > 6)
                std::cout << "Warning: Only the first 6 columns from the file "
                             "will be used for the output format 'xyzirt'"
                          << std::endl;

            for (size_t i = 0; i < nRows; i++)
            {
                pts->insertPointFast(
                    data(i, idxX + 0), data(i, idxX + 1), data(i, idxX + 2));
                pts->insertPointField_Intensity(data(i, idxI));
                pts->insertPointField_Ring(data(i, idxR));
                pts->insertPointField_Timestamp(data(i, idxT));
            }

            pc = pts;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Invalid --format set to '%s'. Valid values: (xyz|xyzi|xyzirt)",
                format.c_str());
        }

        // Save as mm file:
        mp2p_icp::metric_map_t mm;
        mm.layers["raw"] = std::move(pc);

        if (argID.isSet()) mm.id = argID.getValue();
        if (argLabel.isSet()) mm.label = argLabel.getValue();

        std::cout << "Map contents: " << mm.contents_summary() << std::endl;
        std::cout << "Saving map to: " << argOutput.getValue() << std::endl;

        if (!mm.save_to_file(argOutput.getValue()))
            THROW_EXCEPTION_FMT(
                "Error writing to target file '%s'",
                argOutput.getValue().c_str());
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
