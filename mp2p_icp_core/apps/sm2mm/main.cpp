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

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <optional>

#if MRPT_VERSION >= 0x020f07
#include <mrpt/io/compression_options.h>
#endif

// CLI flags:
struct CLI_
{
    CLI::App cmd{"sm2mm"};

    std::string argInput;
    std::string argOutput;
    std::string argPlugins;
    std::string argPipeline;
    std::string arg_verbosity_level;

#if MRPT_VERSION >= 0x020f07
    std::string arg_compression_method;
#endif

    std::string arg_lazy_load_base_dir;

    bool argNoProgressBar               = false;
    bool argProfiler                    = false;
    bool argDontThrowOnMissingExternals = false;

    std::optional<size_t> argIndexFrom;
    std::optional<size_t> argIndexTo;
    std::optional<size_t> argDecimateNth;
    std::optional<size_t> argDecimateMax;

    std::string argGeoRef;

    CLI_()
    {
        cmd.add_option("-i,--input", argInput, "Input .simplemap file")->required();
        cmd.add_option("-o,--output", argOutput, "Output .mm file to write to")->required();
        cmd.add_option(
            "-l,--load-plugins", argPlugins,
            "One or more (comma separated) *.so files to load as plugins, e.g. "
            "defining new CMetricMap classes");
        cmd.add_option(
            "-p,--pipeline", argPipeline,
            "YAML file with the mp2p_icp_filters pipeline to load. It can optionally "
            "contain a `filters:`, a `generators:`, and a `final_filters:` sections. "
            "If this argument is not provided, the default generator will be used and "
            "no filtering will be applied, which might be ok in some cases. "
            "See the app README for examples:\n"
            "https://github.com/MOLAorg/mp2p_icp/tree/develop/mp2p_icp_core/apps/sm2mm");
        cmd.add_option(
            "-v,--verbosity", arg_verbosity_level,
            "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");
#if MRPT_VERSION >= 0x020f07
        cmd.add_option(
            "--compression-method", arg_compression_method,
            "Compression method to use in the output metric map .mm file. "
            "Options: CompressionType::None|CompressionType::Gzip|CompressionType::Zstd. "
            "(Default: CompressionType::Zstd)");
#endif
        cmd.add_option(
            "--externals-dir", arg_lazy_load_base_dir,
            "Lazy-load base directory for datasets with externally-stored observations");
        cmd.add_flag(
            "--no-progress-bar", argNoProgressBar,
            "Disables the progress bar. Useful for cleaner output when using DEBUG "
            "verbosity level.");
        cmd.add_flag("--profiler", argProfiler, "Enables profiler.");
        cmd.add_flag(
            "--permit-missing-externals", argDontThrowOnMissingExternals,
            "If set, missing external files will generate a warning instead of an exception "
            "stopping the processing.");
        cmd.add_option(
            "--from-index", argIndexFrom,
            "If provided, the simplemap keyframes until this index will be discarded "
            "and it will start at this point.");
        cmd.add_option(
            "--to-index", argIndexTo,
            "If provided, the simplemap keyframes will be processed up to this index "
            "only.");
        cmd.add_option("--decimate-nth", argDecimateNth, "Only process every N-th frame");
        cmd.add_option("--decimate-max", argDecimateMax, "Try to evenly pick at most N frames");
        cmd.add_option(
            "-g,--georef", argGeoRef,
            "Optional geo-referencing file (binary `*.georef` or YAML `*.yaml`/`*.yml`) "
            "providing the SE(3) transformation T_enu_to_map and the geodetic reference "
            "coordinates. When supplied, robot poses are expressed in the ENU "
            "(East-North-Up) frame before being passed to the generators, and the output "
            ".mm file is tagged with the georeferencing metadata (T_enu_to_map set to "
            "identity, since the map is already in ENU coordinates).");
    }
};

namespace
{
bool is_binary_georef(const std::string& fil)
{
    return mrpt::system::extractFileExtension(fil) == "georef";
}
}  // namespace

void run_sm_to_mm(CLI_& cli)
{
    const auto& filSM = cli.argInput;

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

    if (!cli.argPipeline.empty())
    {
        const auto filYaml = cli.argPipeline;
        ASSERT_FILE_EXISTS_(filYaml);
        yamlData = mrpt::containers::yaml::FromFile(filYaml);
    }

    mp2p_icp::metric_map_t mm;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (!cli.arg_verbosity_level.empty())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level);
    }

    if (!cli.arg_lazy_load_base_dir.empty())
    {
        mrpt::io::setLazyLoadPathBase(cli.arg_lazy_load_base_dir);
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
    opts.showProgressBar = !cli.argNoProgressBar;
    opts.verbosity       = logLevel;

    if (cli.argIndexFrom.has_value())
    {
        opts.start_index = cli.argIndexFrom.value();
    }
    if (cli.argIndexTo.has_value())
    {
        opts.end_index = cli.argIndexTo.value();
    }
    if (cli.argDecimateNth.has_value())
    {
        opts.decimate_every_nth_frame = cli.argDecimateNth.value();
    }
    if (cli.argDecimateMax.has_value())
    {
        opts.decimate_maximum_frame_count = cli.argDecimateMax.value();
    }

    std::optional<mrpt::system::CTimeLogger> profiler;
    if (cli.argProfiler)
    {
        profiler.emplace();
        opts.profiler = *profiler;
    }

    if (cli.argDontThrowOnMissingExternals)
    {
        opts.throw_on_missing_external_files = false;
    }

    // Load georeferencing data if provided:
    if (!cli.argGeoRef.empty())
    {
        const auto filGeoRef = cli.argGeoRef;
        std::cout << "[sm2mm] Loading georeferencing from: '" << filGeoRef << "'..." << std::endl;

        std::optional<mp2p_icp::metric_map_t::Georeferencing> g;

        if (is_binary_georef(filGeoRef))
        {
            mrpt::io::CFileGZInputStream f(filGeoRef);
            auto                         arch = mrpt::serialization::archiveFrom(f);
            arch >> g;
        }
        else
        {
            const auto yamlGeoRef = mrpt::containers::yaml::FromFile(filGeoRef);
            g                     = mp2p_icp::FromYAML(yamlGeoRef);
        }

        if (!g.has_value())
        {
            THROW_EXCEPTION_FMT(
                "Georeferencing file '%s' was loaded but contained no data.", filGeoRef.c_str());
        }

        opts.georeferencing = *g;
        std::cout << "[sm2mm] Georeferencing loaded OK." << std::endl;
    }

    // Create the map:
    mp2p_icp_filters::simplemap_to_metricmap(sm, mm, yamlData, opts);

    std::cout << "[sm2mm] Final map: " << mm.contents_summary() << std::endl;

    // Save as mm file:
    const auto filOut = cli.argOutput;
    std::cout << "[sm2mm] Writing metric map to: '" << filOut << "'..." << std::endl;

#if MRPT_VERSION >= 0x020f07
    mrpt::io::CompressionOptions compOpts{mrpt::io::CompressionType::Zstd, 3 /*default level*/};
    if (!cli.arg_compression_method.empty())
    {
        using ct      = mrpt::typemeta::TEnumType<mrpt::io::CompressionType>;
        compOpts.type = ct::name2value(cli.arg_compression_method);
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
    CLI_ cli;

    CLI11_PARSE(cli.cmd, argc, argv);

    try
    {
        run_sm_to_mm(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
