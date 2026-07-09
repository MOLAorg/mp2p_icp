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
 * @file   kitti2mm/main.cpp
 * @brief  CLI tool to convert a KITTI dataset LIDAR .bin file into mp2p_icp mm
 * @author Jose Luis Blanco Claraco
 * @date   Jan 3, 2022
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

#include <CLI/CLI.hpp>

// CLI flags:
static CLI::App cmd{"kitti2mm"};

static std::string argInput;
static std::string argOutput;
static std::string argLayer = "raw";
static uint64_t    argID    = 0;
static std::string argLabel = "label";

int main(int argc, char** argv)
{
    cmd.add_option("-i,--input", argInput, "KITTI .bin pointcloud file.")->required();
    cmd.add_option("-o,--output", argOutput, "Output file to write to.")->required();
    cmd.add_option("-l,--layer", argLayer, "Target layer name (Default: \"raw\").");
    auto* optID = cmd.add_option("--id", argID, "Metric map numeric ID (Default: none).");
    auto* optLabel =
        cmd.add_option("--label", argLabel, "Metric map label string (Default: none).");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        const auto& f = argInput;

        auto obs = mrpt::obs::CObservationPointCloud::Create();
        obs->setAsExternalStorage(
            f, mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile);
        obs->load();  // force loading now from disk
        ASSERTMSG_(obs->pointcloud, mrpt::format("Error loading kitti scan file: '%s'", f.c_str()));

        // Save as mm file:
        mp2p_icp::metric_map_t mm;
        mm.layers["raw"] = std::move(obs->pointcloud);

        if (optID->count() > 0)
        {
            mm.id = argID;
        }
        if (optLabel->count() > 0)
        {
            mm.label = argLabel;
        }

        if (!mm.save_to_file(argOutput))
        {
            THROW_EXCEPTION_FMT("Error writing to target file '%s'", argOutput.c_str());
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
