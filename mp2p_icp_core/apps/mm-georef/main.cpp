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
 * @file   mm-georef/main.cpp
 * @brief  CLI tool to manipulate geo-referencing information
 * @author Jose Luis Blanco Claraco
 * @date   Feb 17, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <fstream>
#include <stdexcept>

namespace
{

bool is_binary_file(const std::string& fil)
{
    return mrpt::system::extractFileExtension(fil) == "georef";
}

// CLI flags:
struct Cli
{
    CLI::App cmd{"mm-georef"};

    std::string argMap;
    std::string argGeoRef;
    bool        argExtract = false;
    bool        argInject  = false;
    std::string arg_plugins;

    Cli()
    {
        cmd.add_option("-m,--mao", argMap, "Input/Output .mm file to operate on")->required();
        cmd.add_option(
               "-g,--georef", argGeoRef,
               "Input/Output file with geo-referencing metadata, in binary format "
               "(`*.georef`) or yaml (*.yaml,*.yml) ")
            ->required();
        cmd.add_flag(
            "--extract-from-map", argExtract,
            "Reads the geo-referencing data from the map and saves it to a .georef "
            "file");
        cmd.add_flag(
            "--inject-to-map", argInject,
            "Reads the geo-referencing data from an input file and stores it into "
            "the existing map file.file");
        cmd.add_option(
            "-l,--load-plugins", arg_plugins,
            "One or more (comma separated) *.so files to load as plugins");
    }
};

void run_mm_extract(Cli& cli)
{
    const auto& filInput = cli.argMap;

    std::cout << "[mm-georef] Reading input map from: '" << filInput << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-georef] Done read map:" << mm.contents_summary() << std::endl;
    ASSERT_(!mm.empty());

    std::cout << "[mm-georef] Done. Output map: " << mm.contents_summary() << std::endl;

    // Save as .georef file:
    const auto& filOut = cli.argGeoRef;
    std::cout << "[mm-georef] Writing geo-referencing metamap to: '" << filOut << "'..."
              << std::endl;

    if (is_binary_file(filOut))
    {
        mrpt::io::CFileGZOutputStream f(filOut);
        auto                          arch = mrpt::serialization::archiveFrom(f);
        arch << mm.georeferencing;
    }
    else
    {
        const auto    yamlData = mp2p_icp::ToYAML(mm.georeferencing);
        std::ofstream of(filOut);
        ASSERT_(of.is_open());
        of << yamlData;
    }
}

void run_mm_inject(Cli& cli)
{
    // Load .georef file:
    const auto& filIn = cli.argGeoRef;
    std::cout << "[mm-georef] Reading geo-referencing metamap from: '" << filIn << "'..."
              << std::endl;

    std::optional<mp2p_icp::metric_map_t::Georeferencing> g;

    if (is_binary_file(filIn))
    {
        mrpt::io::CFileGZInputStream f(filIn);
        auto                         arch = mrpt::serialization::archiveFrom(f);
        arch >> g;
    }
    else
    {
        const auto yamlData = mrpt::containers::yaml::FromFile(filIn);

        g = mp2p_icp::FromYAML(yamlData);
    }

    const auto& filMap = cli.argMap;

    std::cout << "[mm-georef] Reading input map from: '" << filMap << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;

    bool readOk = mm.load_from_file(filMap);
    if (!readOk)
    {
        THROW_EXCEPTION_FMT("Error reading map file: '%s'", filMap.c_str());
    }

    std::cout << "[mm-georef] Done read map: " << mm.contents_summary() << std::endl;

    // Set geo-ref data:
    mm.georeferencing = g;

    std::cout << "[mm-georef] Set georeferencing data. Updated map data: " << mm.contents_summary()
              << std::endl;

    std::cout << "[mm-georef] Saving updated map to: '" << filMap << "'..." << std::endl;

    bool ok = mm.save_to_file(filMap);
    if (!ok)
    {
        THROW_EXCEPTION_FMT("Error saving updated map to file: '%s'", filMap.c_str());
    }
}

void run_mm_georef(Cli& cli)
{
    // Load plugins:
    if (!cli.arg_plugins.empty())
    {
        std::string errMsg;
        const auto& plugins = cli.arg_plugins;
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
        {
            throw std::runtime_error(errMsg);
        }
    }

    if (cli.argExtract)
    {
        return run_mm_extract(cli);
    }
    if (cli.argInject)
    {
        return run_mm_inject(cli);
    }

    THROW_EXCEPTION(
        "One of either '--extract-from-map' or '--inject-to-map' flags must be "
        "provided.");
}
}  // namespace

int main(int argc, char** argv)
{
    Cli cli;

    CLI11_PARSE(cli.cmd, argc, argv);

    try
    {
        run_mm_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
