/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   sm2mm/main.cpp
 * @brief  CLI tool to parse a SimpleMap (from SLAM) to metric maps (mm) via a
 *         configurable pipeline
 * @author Jose Luis Blanco Claraco
 * @date   Dec 15, 2023
 */

#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/filesystem.h>

// CLI flags:
static TCLAP::CmdLine cmd("sm2mm");

static TCLAP::ValueArg<std::string> argInput(
    "i", "input", "Input .simplemap file", true, "map.simplemap", "map.simplemap", cmd);

static TCLAP::ValueArg<std::string> argOutput(
    "o", "output", "Output .mm file to write to", true, "out.mm", "out.mm", cmd);

static TCLAP::ValueArg<std::string> argPlugins(
    "l", "load-plugins",
    "One or more (comma separated) *.so files to load as plugins, e.g. "
    "defining new CMetricMap classes",
    false, "foobar.so", "foobar.so", cmd);

static TCLAP::ValueArg<std::string> argPipeline(
    "p", "pipeline",
    "YAML file with the mp2p_icp_filters pipeline to load. It can optionally "
    "contain a `filters:`, a `generators:`, and a `final_filters:` sections. "
    "If this argument is not provided, the default generator will be used and "
    "no filtering will be applied, which might be ok in some cases. "
    "See the app README for examples:\n"
    "https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm",
    false, "pipeline.yaml", "pipeline.yaml", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "", "INFO",
    cmd);

static TCLAP::ValueArg<std::string> arg_lazy_load_base_dir(
    "", "externals-dir",
    "Lazy-load base directory for datasets with externally-stored observations", false,
    "dataset_Images", "<ExternalsDirectory>", cmd);

static TCLAP::SwitchArg argNoProgressBar(
    "", "no-progress-bar",
    "Disables the progress bar. Useful for cleaner output when using DEBUG "
    "verbosity level.",
    cmd);

static TCLAP::ValueArg<size_t> argIndexFrom(
    "", "from-index",
    "If provided, the simplemap keyframes until this index will be discarded "
    "and it will start at this point.",
    false, 0, "0", cmd);

static TCLAP::ValueArg<size_t> argIndexTo(
    "", "to-index",
    "If provided, the simplemap keyframes will be processed up to this index "
    "only.",
    false, 0, "0", cmd);

void run_sm_to_mm()
{
    const auto& filSM = argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[sm2mm] Reading simplemap from: '" << filSM << "'..." << std::endl;

    // TODO: progress bar
    sm.loadFromFile(filSM);

    std::cout << "[sm2mm] Done read simplemap with " << sm.size() << " keyframes." << std::endl;
    ASSERT_(!sm.empty());

    // Load pipeline from YAML file:
    mrpt::containers::yaml yamlData;  // default: empty

    if (argPipeline.isSet())
    {
        const auto filYaml = argPipeline.getValue();
        ASSERT_FILE_EXISTS_(filYaml);
        yamlData = mrpt::containers::yaml::FromFile(filYaml);
    }

    mp2p_icp::metric_map_t mm;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(arg_verbosity_level.getValue());
    }

    if (arg_lazy_load_base_dir.isSet())
        mrpt::io::setLazyLoadPathBase(arg_lazy_load_base_dir.getValue());

    mp2p_icp_filters::sm2mm_options_t opts;
    opts.showProgressBar = !argNoProgressBar.isSet();
    opts.verbosity       = logLevel;

    if (argIndexFrom.isSet()) opts.start_index = argIndexFrom.getValue();
    if (argIndexTo.isSet()) opts.end_index = argIndexTo.getValue();

    // Create the map:
    mp2p_icp_filters::simplemap_to_metricmap(sm, mm, yamlData, opts);

    std::cout << "[sm2mm] Final map: " << mm.contents_summary() << std::endl;

    // Save as mm file:
    const auto filOut = argOutput.getValue();
    std::cout << "[sm2mm] Writing metric map to: '" << filOut << "'..." << std::endl;

    if (!mm.save_to_file(filOut))
        THROW_EXCEPTION_FMT("Error writing to target file '%s'", filOut.c_str());
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        run_sm_to_mm();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
