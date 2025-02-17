/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mm-georef/main.cpp
 * @brief  CLI tool to manipulate geo-referencing information
 * @author Jose Luis Blanco Claraco
 * @date   Feb 17, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

namespace
{

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"mm-georef"};

    TCLAP::ValueArg<std::string> argMap{
        "m",  "mao",       "Input/Output .mm file to operate on",
        true, "theMap.mm", "theMap.mm",
        cmd};

    TCLAP::ValueArg<std::string> argGeoRef{
        "g",
        "georef",
        "Input/Output binary `.georef` file with geo-referencing metadata",
        true,
        "myMap.georef",
        "myMap.georef",
        cmd};

    TCLAP::SwitchArg argExtract{
        "", "extract-from-map",
        "Reads the geo-referencing data from the map and saves it to a .georef "
        "file",
        cmd};

    TCLAP::SwitchArg argInject{
        "", "inject-to-map",
        "Reads the geo-referencing data from an input file and stores it into "
        "the existing map file."
        "file",
        cmd};

    TCLAP::ValueArg<std::string> arg_plugins{
        "l",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins",
        false,
        "foobar.so",
        "foobar.so",
        cmd};
};

void run_mm_extract(Cli& cli)
{
    const auto& filInput = cli.argMap.getValue();

    std::cout << "[mm-georef] Reading input map from: '" << filInput << "'..."
              << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-georef] Done read map:" << mm.contents_summary()
              << std::endl;
    ASSERT_(!mm.empty());

    std::cout << "[mm-georef] Done. Output map: " << mm.contents_summary()
              << std::endl;

    // Save as .georef file:
    const auto filOut = cli.argGeoRef.getValue();
    std::cout << "[mm-georef] Writing geo-referencing metamap to: '" << filOut
              << "'..." << std::endl;

    mrpt::io::CFileGZOutputStream f(filOut);
    auto                          arch = mrpt::serialization::archiveFrom(f);
    arch << mm.georeferencing;
}

void run_mm_inject(Cli& cli)
{
    // Load .georef file:
    const auto filIn = cli.argGeoRef.getValue();
    std::cout << "[mm-georef] Reading geo-referencing metamap from: '" << filIn
              << "'..." << std::endl;

    mrpt::io::CFileGZInputStream f(filIn);
    auto                         arch = mrpt::serialization::archiveFrom(f);

    std::optional<mp2p_icp::metric_map_t::Georeferencing> g;
    arch >> g;

    const auto& filMap = cli.argMap.getValue();

    std::cout << "[mm-georef] Reading input map from: '" << filMap << "'..."
              << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filMap);

    std::cout << "[mm-georef] Done read map: " << mm.contents_summary()
              << std::endl;

    // Set geo-ref data:
    mm.georeferencing = g;

    std::cout << "[mm-georef] Set georeferencing data. Updated map data: "
              << mm.contents_summary() << std::endl;

    std::cout << "[mm-georef] Saving updated map to: '" << filMap << "'..."
              << std::endl;

    mm.save_to_file(filMap);
}

void run_mm_georef(Cli& cli)
{
    // Load plugins:
    if (cli.arg_plugins.isSet())
    {
        std::string errMsg;
        const auto  plugins = cli.arg_plugins.getValue();
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
            throw std::runtime_error(errMsg);
    }

    if (cli.argExtract.isSet()) { return run_mm_extract(cli); }
    else if (cli.argInject.isSet()) { return run_mm_inject(cli); }

    THROW_EXCEPTION(
        "One of either '--extract-from-map' or '--inject-to-map' flags must be "
        "provided.");
}
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

        run_mm_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
