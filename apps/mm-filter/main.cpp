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

// CLI flags:
static TCLAP::CmdLine cmd("mm-filter");

static TCLAP::ValueArg<std::string> argInput(
    "i", "input", "Input .mm file", true, "input.mm", "input.mm", cmd);

static TCLAP::ValueArg<std::string> argOutput(
    "o", "output", "Output .mm file to write to", true, "out.mm", "out.mm",
    cmd);

static TCLAP::ValueArg<std::string> argPipeline(
    "p", "pipeline",
    "YAML file with the mp2p_icp_filters pipeline to load. It must "
    "contain a `filters:` section."
    "See the app README for examples:\n"
    "https://github.com/MOLAorg/mp2p_icp/tree/master/apps/mm-filter",
    false, "pipeline.yaml", "pipeline.yaml", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
    false, "", "INFO", cmd);

void run_mm_filter()
{
    const auto& filInput = argInput.getValue();

    ASSERT_FILE_EXISTS_(argPipeline.getValue());

    std::cout << "[mm-filter] Reading input map from: '" << filInput << "'..."
              << std::endl;

    mp2p_icp::metric_map_t mm;
    mm.load_from_file(filInput);

    std::cout << "[mm-filter] Done read map:" << mm.contents_summary()
              << std::endl;
    ASSERT_(!mm.empty());

    // Load pipeline:
    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(arg_verbosity_level.getValue());
    }

    const auto pipeline = mp2p_icp_filters::filter_pipeline_from_yaml_file(
        argPipeline.getValue(), logLevel);

    // Apply:
    std::cout << "[mm-filter] Applying filter pipeline..." << std::endl;

    mp2p_icp_filters::apply_filter_pipeline(pipeline, mm);

    std::cout << "[mm-filter] Done. Output map: " << mm.contents_summary()
              << std::endl;

    // Save as mm file:
    const auto filOut = argOutput.getValue();
    std::cout << "[mm-filter] Writing metric map to: '" << filOut << "'..."
              << std::endl;

    if (!mm.save_to_file(filOut))
        THROW_EXCEPTION_FMT(
            "Error writing to target file '%s'", filOut.c_str());
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        run_mm_filter();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
