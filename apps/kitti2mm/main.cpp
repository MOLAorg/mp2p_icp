/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   kitti2mm/main.cpp
 * @brief  CLI tool to convert a KITTI dataset LIDAR .bin file into mp2p_icp mm
 * @author Jose Luis Blanco Claraco
 * @date   Jan 3, 2022
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

// CLI flags:
static TCLAP::CmdLine cmd("kitti2mm");

static TCLAP::ValueArg<std::string> argInput(
    "i", "input", "KITTI .bin pointcloud file.", true, "kitti-00.bin",
    "kitti-00.bin", cmd);

static TCLAP::ValueArg<std::string> argOutput(
    "o", "output", "Output file to write to.", true, "out.mm", "out.mm", cmd);

static TCLAP::ValueArg<std::string> argLayer(
    "l", "layer", "Target layer name (Default: \"raw\").", false, "raw", "raw",
    cmd);

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

        auto obs = mrpt::obs::CObservationPointCloud::Create();
        obs->setAsExternalStorage(
            f, mrpt::obs::CObservationPointCloud::ExternalStorageFormat::
                   KittiBinFile);
        obs->load();  // force loading now from disk
        ASSERTMSG_(
            obs->pointcloud,
            mrpt::format("Error loading kitti scan file: '%s'", f.c_str()));

        // Save as mm file:
        mp2p_icp::metric_map_t mm;
        mm.layers["raw"] = std::move(obs->pointcloud);

        if (argID.isSet()) mm.id = argID.getValue();
        if (argLabel.isSet()) mm.label = argLabel.getValue();

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
