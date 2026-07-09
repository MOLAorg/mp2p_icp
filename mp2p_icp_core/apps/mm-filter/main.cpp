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
 * @file   mm-filter/main.cpp
 * @brief  CLI tool to apply filter pipelines to mm files
 * @author Jose Luis Blanco Claraco
 * @date   Feb 13, 2024
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/system/filesystem.h>

#include <CLI/CLI.hpp>
#include <stdexcept>

namespace
{

// CLI flags:
struct Cli
{
    CLI::App cmd{"mm-filter"};

    std::string argInput;
    std::string argOutput;
    std::string arg_plugins;
    std::string argPipeline;
    std::string argRename;
    std::string arg_verbosity_level;

    Cli()
    {
        cmd.add_option("-i,--input", argInput, "Input .mm file")->required();
        cmd.add_option("-o,--output", argOutput, "Output .mm file to write to")->required();
        cmd.add_option(
            "-l,--load-plugins", arg_plugins,
            "One or more (comma separated) *.so files to load as plugins");
        cmd.add_option(
            "-p,--pipeline", argPipeline,
            "YAML file with the mp2p_icp_filters pipeline to load. It must "
            "contain a `filters:` section."
            "See the app README for examples:\n"
            "https://github.com/MOLAorg/mp2p_icp/tree/develop/mp2p_icp_core/apps/mm-filter");
        cmd.add_option(
            "--rename-layer", argRename,
            "Alternative operation: instead of applying a pipeline, just renames a "
            "layer from NAME to NEW_NAME.");
        cmd.add_option(
            "-v,--verbosity", arg_verbosity_level,
            "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");
    }
};

void run_mm_filter(Cli& cli)
{
    ASSERTMSG_(
        !cli.argPipeline.empty() || !cli.argRename.empty(),
        "It is mandatory to set at least one of these CLI arguments (run with "
        "--help) for further details: --pipeline or --rename");

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

    const auto& filInput = cli.argInput;

    if (!cli.argPipeline.empty())
    {
        ASSERT_FILE_EXISTS_(cli.argPipeline);
    }

    std::cout << "[mm-filter] Reading input map from: '" << filInput << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-filter] Done read map:" << mm.contents_summary() << std::endl;
    ASSERT_(!mm.empty());

    // Load pipeline:
    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (!cli.arg_verbosity_level.empty())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level);
    }

    if (!cli.argPipeline.empty())
    {
        const auto pipeline =
            mp2p_icp_filters::filter_pipeline_from_yaml_file(cli.argPipeline, logLevel);

        // Apply:
        std::cout << "[mm-filter] Applying filter pipeline..." << std::endl;

        mp2p_icp_filters::apply_filter_pipeline(pipeline, mm);
    }
    else
    {
        ASSERT_(!cli.argRename.empty());
        const auto&              s = cli.argRename;
        std::vector<std::string> names;
        mrpt::system::tokenize(s, "|", names);
        ASSERTMSG_(names.size() == 2, "Expected format: --rename \"OLD_NAME|NEW_NAME\"");

        const auto oldName = names[0];
        const auto newName = names[1];

        ASSERT_(mm.layers.count(oldName) == 1);
        ASSERT_(mm.layers.count(newName) == 0);

        mm.layers[newName] = mm.layers[oldName];
        mm.layers.erase(oldName);
    }

    std::cout << "[mm-filter] Done. Output map: " << mm.contents_summary() << std::endl;

    // Save as mm file:
    const auto& filOut = cli.argOutput;
    std::cout << "[mm-filter] Writing metric map to: '" << filOut << "'..." << std::endl;

    if (!mm.save_to_file(filOut))
    {
        THROW_EXCEPTION_FMT("Error writing to target file '%s'", filOut.c_str());
    }
}
}  // namespace

int main(int argc, char** argv)
{
    Cli cli;

    CLI11_PARSE(cli.cmd, argc, argv);

    try
    {
        run_mm_filter(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
