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
 * @file   rawlog-filter/main.cpp
 * @brief  CLI tool to apply filter pipelines to datasets in rawlog files
 * @author Jose Luis Blanco Claraco
 * @date   Oct 21, 2024
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <optional>
#include <stdexcept>

#if MRPT_VERSION >= 0x020f07
#include <mrpt/io/CCompressedOutputStream.h>
#else
#include <mrpt/io/CFileGZOutputStream.h>
#endif

// CLI flags:
struct Cli
{
    CLI::App cmd{"rawlog-filter"};

    std::string argInput;
    std::string argOutput;
    std::string argPipeline;

    std::optional<size_t> arg_from;
    std::optional<size_t> arg_to;

    std::string arg_lazy_load_base_dir;
    std::string arg_verbosity_level;
    std::string arg_plugins;

    Cli()
    {
        cmd.add_option("-i,--input", argInput, "Input .rawlog file")->required();
        cmd.add_option("-o,--output", argOutput, "Output .rawlo file to write to")->required();
        cmd.add_option(
               "-p,--pipeline", argPipeline,
               "YAML file with the mp2p_icp_filters pipeline to load. It must "
               "contain a `filters:` section."
               "See the app README for examples:\n"
               "https://github.com/MOLAorg/mp2p_icp/tree/develop/mp2p_icp_core/apps/rawlog-filter")
            ->required();
        cmd.add_option("--from", arg_from, "First rawlog index to process");
        cmd.add_option("--to", arg_to, "Last rawlog index to process");
        cmd.add_option(
            "--externals-dir", arg_lazy_load_base_dir,
            "Lazy-load base directory for datasets with externally-stored "
            "observations");
        cmd.add_option(
            "-v,--verbosity", arg_verbosity_level,
            "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");
        cmd.add_option(
            "-l,--load-plugins", arg_plugins,
            "One or more (comma separated) *.so files to load as plugins");
    }
};

void run_mm_filter(Cli& cli)
{
    using namespace std::string_literals;

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

    ASSERT_FILE_EXISTS_(cli.argInput);
    ASSERT_FILE_EXISTS_(cli.argPipeline);

    const auto& filInput = cli.argInput;

    std::cout << "[rawlog-filter] Reading input rawlog from: '" << filInput << "'..." << std::endl;

    mrpt::obs::CRawlog dataset;

    bool readOk = dataset.loadFromRawLogFile(filInput);
    ASSERT_(readOk);

    std::cout << "[rawlog-filter] Done read dataset (" << dataset.size() << " entries)"
              << std::endl;

    // Load pipeline:
    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (!cli.arg_verbosity_level.empty())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level);
    }

    const auto yamlData = mrpt::containers::yaml::FromFile(cli.argPipeline);

    // Generators:
    mp2p_icp_filters::GeneratorSet generators;
    if (yamlData.has("generators"))
    {
        generators = mp2p_icp_filters::generators_from_yaml(yamlData["generators"], logLevel);
    }
    else
    {
        std::cout << "[rawlog-filter] Warning: no generators defined in the "
                     "pipeline, using default generator."
                  << std::endl;

        auto defaultGen = mp2p_icp_filters::Generator::Create();
        defaultGen->setMinLoggingLevel(logLevel);
        defaultGen->initialize({});
        generators.push_back(defaultGen);
    }

    // Filters:
    mp2p_icp_filters::FilterPipeline filters;
    if (yamlData.has("filters"))
    {
        filters = mp2p_icp_filters::filter_pipeline_from_yaml(yamlData["filters"], logLevel);
    }
    else
    {
        std::cout << "[sm2mm] Warning: no filters defined in the pipeline." << std::endl;
    }

    // Parameters for twist, and possibly other user-provided variables.
    mp2p_icp::ParameterSource ps;
    mp2p_icp::AttachToParameterSource(generators, ps);
    mp2p_icp::AttachToParameterSource(filters, ps);

    // Default values for twist variables:
    ps.updateVariables({{"vx", .0}, {"vy", .0}, {"vz", .0}, {"wx", .0}, {"wy", .0}, {"wz", .0}});
    ps.updateVariables(
        {{"robot_x", .0},
         {"robot_y", .0},
         {"robot_z", .0},
         {"robot_yaw", .0},
         {"robot_pitch", .0},
         {"robot_roll", .0}});

    // ps.updateVariables(options.customVariables);

    ps.realize();

    // progress bar:
    std::cout << "\n";  // Needed for the VT100 codes below.

    const double tStart = mrpt::Clock::nowDouble();

    size_t nKFs = dataset.size();
    if (cli.arg_to.has_value())
    {
        mrpt::keep_min(nKFs, cli.arg_to.value() + 1);
    }

    size_t curKF = 0;
    if (cli.arg_from.has_value())
    {
        mrpt::keep_max(curKF, cli.arg_from.value());
    }

    // Create output Rawlog file:
    const auto filOut = cli.argOutput;
    std::cout << "[rawlog-filter] Creating output rawlog file: '" << filOut << "'..." << std::endl;

#if MRPT_VERSION >= 0x020f07
    mrpt::io::CCompressedOutputStream fo(
        filOut, mrpt::io::OpenMode::TRUNCATE, {mrpt::io::CompressionType::Zstd});
#else
    mrpt::io::CFileGZOutputStream fo(filOut);
#endif

    auto outArch = mrpt::serialization::archiveFrom(fo);

    if (!cli.arg_lazy_load_base_dir.empty())
    {
        mrpt::io::setLazyLoadPathBase(cli.arg_lazy_load_base_dir);
    }

    for (; curKF < nKFs; curKF++)
    {
        auto obs = dataset.getAsObservation(curKF);
        ASSERTMSG_(obs, "Dataset is expected to have CObservation objects only!");

        obs->load();

        mp2p_icp::metric_map_t mm;

        bool handled = mp2p_icp_filters::apply_generators(generators, *obs, mm);

        if (!handled)
        {
            continue;
        }

        // process it:
        mp2p_icp_filters::apply_filter_pipeline(filters, mm);
        obs->unload();

        // Create output:
        mrpt::obs::CSensoryFrame sf;
        // Input:
        sf.insert(obs);
        // Output:
        for (const auto& [name, layer] : mm.layers)
        {
            if (!layer)
            {
                continue;
            }
            if (auto ptsMap = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer); ptsMap)
            {
                auto obsPts         = mrpt::obs::CObservationPointCloud::Create();
                obsPts->timestamp   = obs->timestamp;
                obsPts->sensorLabel = "out_"s + name;
                obsPts->pointcloud  = ptsMap;
                sf.insert(obsPts);
            }
        }
        outArch << sf;  // save to disk

        // progress bar:
        {
            const size_t N  = nKFs;
            const double pc = static_cast<double>(curKF) / static_cast<double>(N);

            const double tNow      = mrpt::Clock::nowDouble();
            const double ETA       = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
            const double totalTime = ETA + (tNow - tStart);

            std::cout << "\033[A\33[2KT\r"  // VT100 codes: cursor up and clear line
                      << mrpt::system::progress(pc, 30)
                      << mrpt::format(
                             " %6zu/%6zu (%.02f%%) ETA=%s / T=%s\n", curKF, N, 100 * pc,
                             mrpt::system::formatTimeInterval(ETA).c_str(),
                             mrpt::system::formatTimeInterval(totalTime).c_str());
            std::cout.flush();
        }
    }  // end for each KF.
}

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
