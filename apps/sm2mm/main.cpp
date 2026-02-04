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
 * @file   sm2mm/main.cpp
 * @brief  CLI tool to parse a SimpleMap (from SLAM) to metric maps (mm) via a
 *         configurable pipeline
 * @author Jose Luis Blanco Claraco
 * @date   Dec 15, 2023
 */

#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#if MRPT_VERSION >= 0x020f07
#include <mrpt/io/compression_options.h>
#endif

// CLI flags:
struct CLI
{
    TCLAP::CmdLine cmd{"sm2mm"};

    TCLAP::ValueArg<std::string> argInput{
        "i", "input", "Input .simplemap file", true, "map.simplemap", "map.simplemap", cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o", "output", "Output .mm file to write to", true, "out.mm", "out.mm", cmd};

    TCLAP::ValueArg<std::string> argPlugins{
        "l",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes",
        false,
        "foobar.so",
        "foobar.so",
        cmd};

    TCLAP::ValueArg<std::string> argPipeline{
        "p",
        "pipeline",
        "YAML file with the mp2p_icp_filters pipeline to load. It can optionally "
        "contain a `filters:`, a `generators:`, and a `final_filters:` sections. "
        "If this argument is not provided, the default generator will be used and "
        "no filtering will be applied, which might be ok in some cases. "
        "See the app README for examples:\n"
        "https://github.com/MOLAorg/mp2p_icp/tree/develop/apps/sm2mm",
        false,
        "pipeline.yaml",
        "pipeline.yaml",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "",
        "INFO", cmd};

#if MRPT_VERSION >= 0x020f07
    TCLAP::ValueArg<std::string> arg_compression_method{
        "",
        "compression-method",
        "Compression method to use in the output metric map .mm file. "
        "Options: CompressionType::None|CompressionType::Gzip|CompressionType::Zstd. (Default: "
        "CompressionType::Zstd)",
        false,
        "CompressionType::Zstd",
        "METHOD",
        cmd};
#endif

    TCLAP::ValueArg<std::string> arg_lazy_load_base_dir{
        "",
        "externals-dir",
        "Lazy-load base directory for datasets with externally-stored observations",
        false,
        "dataset_Images",
        "<ExternalsDirectory>",
        cmd};

    TCLAP::SwitchArg argNoProgressBar{
        "", "no-progress-bar",
        "Disables the progress bar. Useful for cleaner output when using DEBUG "
        "verbosity level.",
        cmd};

    TCLAP::SwitchArg argProfiler{"", "profiler", "Enables profiler.", cmd};

    TCLAP::SwitchArg argDontThrowOnMissingExternals{
        "", "permit-missing-externals",
        "If set, missing external files will generate a warning instead of an exception stopping "
        "the processing.",
        cmd};

    TCLAP::ValueArg<size_t> argIndexFrom{
        "",
        "from-index",
        "If provided, the simplemap keyframes until this index will be discarded "
        "and it will start at this point.",
        false,
        0,
        "0",
        cmd};

    TCLAP::ValueArg<size_t> argIndexTo{
        "",
        "to-index",
        "If provided, the simplemap keyframes will be processed up to this index "
        "only.",
        false,
        0,
        "0",
        cmd};

    TCLAP::ValueArg<size_t> argDecimateNth{
        "", "decimate-nth", "Only process every N-th frame", false, 1, "N", cmd};

    TCLAP::ValueArg<size_t> argDecimateMax{
        "", "decimate-max", "Try to evenly pick at most N frames", false, 0, "N", cmd};
};

void run_sm_to_mm(CLI& cli)
{
    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[sm2mm] Reading simplemap from: '" << filSM << "'..." << std::endl;
    const double sm_t0 = mrpt::Clock::nowDouble();

    sm.loadFromFile(filSM);

    const double sm_t1 = mrpt::Clock::nowDouble();
    std::cout << "[sm2mm] Done read simplemap with " << sm.size() << " keyframes in "
              << (sm_t1 - sm_t0) << " sec.\n";

    ASSERT_(!sm.empty());

    // Load pipeline from YAML file:
    mrpt::containers::yaml yamlData;  // default: empty

    if (cli.argPipeline.isSet())
    {
        const auto filYaml = cli.argPipeline.getValue();
        ASSERT_FILE_EXISTS_(filYaml);
        yamlData = mrpt::containers::yaml::FromFile(filYaml);
    }

    mp2p_icp::metric_map_t mm;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli.arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    }

    if (cli.arg_lazy_load_base_dir.isSet())
    {
        mrpt::io::setLazyLoadPathBase(cli.arg_lazy_load_base_dir.getValue());
    }
    else
    {
        // Try to set a base dir according to the input file, appending "_Images" to the base name:
        const auto lazyBaseDir = mrpt::system::fileNameChangeExtension(filSM, "") + "_Images";
        if (mrpt::system::directoryExists(lazyBaseDir))
        {
            mrpt::io::setLazyLoadPathBase(lazyBaseDir);
            std::cout << "[sm2mm] Using lazy-load base dir: " << lazyBaseDir << std::endl;
        }
    }

    mp2p_icp_filters::sm2mm_options_t opts;
    opts.showProgressBar = !cli.argNoProgressBar.isSet();
    opts.verbosity       = logLevel;

    if (cli.argIndexFrom.isSet())
    {
        opts.start_index = cli.argIndexFrom.getValue();
    }
    if (cli.argIndexTo.isSet())
    {
        opts.end_index = cli.argIndexTo.getValue();
    }
    if (cli.argDecimateNth.isSet())
    {
        opts.decimate_every_nth_frame = cli.argDecimateNth.getValue();
    }
    if (cli.argDecimateMax.isSet())
    {
        opts.decimate_maximum_frame_count = cli.argDecimateMax.getValue();
    }

    std::optional<mrpt::system::CTimeLogger> profiler;
    if (cli.argProfiler.isSet())
    {
        profiler.emplace();
        opts.profiler = *profiler;
    }

    if (cli.argDontThrowOnMissingExternals.isSet())
    {
        opts.throw_on_missing_external_files = false;
    }

    // Create the map:
    mp2p_icp_filters::simplemap_to_metricmap(sm, mm, yamlData, opts);

    std::cout << "[sm2mm] Final map: " << mm.contents_summary() << std::endl;

    // Save as mm file:
    const auto filOut = cli.argOutput.getValue();
    std::cout << "[sm2mm] Writing metric map to: '" << filOut << "'..." << std::endl;

#if MRPT_VERSION >= 0x020f07
    mrpt::io::CompressionOptions compOpts{mrpt::io::CompressionType::Zstd, 3 /*default level*/};
    if (cli.arg_compression_method.isSet())
    {
        using ct      = mrpt::typemeta::TEnumType<mrpt::io::CompressionType>;
        compOpts.type = ct::name2value(cli.arg_compression_method.getValue());
    }
#endif

#if MRPT_VERSION >= 0x020f07
    if (!mm.save_to_file(filOut, compOpts))
#else
    if (!mm.save_to_file(filOut))
#endif
    {
        THROW_EXCEPTION_FMT("Error writing to target file '%s'", filOut.c_str());
    }
}

int main(int argc, char** argv)
{
    try
    {
        CLI cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        run_sm_to_mm(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
