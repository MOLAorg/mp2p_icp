/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mm-filter/main.cpp
 * @brief  CLI tool to apply filter pipelines to mm files
 * @author Jose Luis Blanco Claraco
 * @date   Feb 13, 2024
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/system/filesystem.h>

namespace
{

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"mm-filter"};

    TCLAP::ValueArg<std::string> argInput{"i",        "input", "Input .mm file", true, "input.mm",
                                          "input.mm", cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o", "output", "Output .mm file to write to", true, "out.mm", "out.mm", cmd};

    TCLAP::ValueArg<std::string> arg_plugins{
        "l",   "load-plugins", "One or more (comma separated) *.so files to load as plugins",
        false, "foobar.so",    "foobar.so",
        cmd};

    TCLAP::ValueArg<std::string> argPipeline{
        "p",
        "pipeline",
        "YAML file with the mp2p_icp_filters pipeline to load. It must "
        "contain a `filters:` section."
        "See the app README for examples:\n"
        "https://github.com/MOLAorg/mp2p_icp/tree/master/apps/mm-filter",
        false,
        "pipeline.yaml",
        "pipeline.yaml",
        cmd};

    TCLAP::ValueArg<std::string> argRename{
        "",
        "rename-layer",
        "Alternative operation: instead of applying a pipeline, just renames a "
        "layer from NAME to NEW_NAME.",
        false,
        "\"NAME|NEW_NAME\"",
        "\"NAME|NEW_NAME\"",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "",
        "INFO", cmd};
};

void run_mm_filter(Cli& cli)
{
    ASSERTMSG_(
        cli.argPipeline.isSet() || cli.argRename.isSet(),
        "It is mandatory to set at least one of these CLI arguments (run with "
        "--help) for further details: --pipeline or --rename");

    // Load plugins:
    if (cli.arg_plugins.isSet())
    {
        std::string errMsg;
        const auto  plugins = cli.arg_plugins.getValue();
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg)) throw std::runtime_error(errMsg);
    }

    const auto& filInput = cli.argInput.getValue();

    if (cli.argPipeline.isSet()) ASSERT_FILE_EXISTS_(cli.argPipeline.getValue());

    std::cout << "[mm-filter] Reading input map from: '" << filInput << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-filter] Done read map:" << mm.contents_summary() << std::endl;
    ASSERT_(!mm.empty());

    // Load pipeline:
    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli.arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    }

    if (cli.argPipeline.isSet())
    {
        const auto pipeline =
            mp2p_icp_filters::filter_pipeline_from_yaml_file(cli.argPipeline.getValue(), logLevel);

        // Apply:
        std::cout << "[mm-filter] Applying filter pipeline..." << std::endl;

        mp2p_icp_filters::apply_filter_pipeline(pipeline, mm);
    }
    else
    {
        ASSERT_(cli.argRename.isSet());
        const auto               s = cli.argRename.getValue();
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
    const auto filOut = cli.argOutput.getValue();
    std::cout << "[mm-filter] Writing metric map to: '" << filOut << "'..." << std::endl;

    if (!mm.save_to_file(filOut))
        THROW_EXCEPTION_FMT("Error writing to target file '%s'", filOut.c_str());
}
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

        run_mm_filter(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
