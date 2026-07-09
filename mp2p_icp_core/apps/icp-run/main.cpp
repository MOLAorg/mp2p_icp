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
 * @file   mp2p_icp_run/main.cpp
 * @brief  CLI tool to execute mp2p_icp runs against point cloud in files.
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15 , 2021
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/load_xyz_file.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/CImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <stdexcept>

// CLI flags:
static CLI::App cmd{"icp-run"};

static std::string argInputLocal;
static std::string argInputGlobal;
static std::string argYamlConfigFile;
static std::string argYamlConfigFileGenerators;
static std::string argYamlConfigFileFiltersLocal;
static std::string argYamlConfigFileFiltersGlobal;
static std::string argCfgNameFiltersGlobal = "filters";
static std::string argCfgNameFiltersLocal  = "filters";
static std::string argInitialGuess         = "[0 0 0 0 0 0]";
static bool        argGenerateDebugFiles   = false;
static bool        argProfile              = false;
static std::string arg_plugins;

static void defineCliArgs()
{
    cmd.add_option(
           "--input-local", argInputLocal,
           "Local input point cloud/map."
           "It is interpreted as a rawlog entry if using the "
           "format `<RAWLOG_FILE.rawlog>:<N>` to select the N-th entry in the "
           "rawlog; otherwise, if the file extension is `.mm` it is loaded as a "
           "serialized metric_map_t object; if it is a `.icplog` file, the local map "
           "from that icp log is taken as input; in any other case, the file is "
           "assumed to be a 3D pointcloud stored as a Nx3 ASCII matrix file.")
        ->required();

    cmd.add_option(
           "--input-global", argInputGlobal,
           "Global input point cloud/map. Same format than input-local. ")
        ->required();

    cmd.add_option(
           "-c,--config", argYamlConfigFile,
           "YAML config file describing the ICP pipeline. See docs:\n"
           " https://docs.mola-slam.org/latest/"
           "module-mp2p-icp.html#yaml-pipeline-definition-files")
        ->required();

    cmd.add_option(
        "--config-generators", argYamlConfigFileGenerators,
        "YAML config file describing the Generators. Can be also defined via an "
        "entry `generators` in the main `--config` yaml file. "
        "Can be used when processing a rawlog as input; if not present, a default "
        "Generator object will be used.");

    cmd.add_option(
        "--config-filters-local", argYamlConfigFileFiltersLocal,
        "YAML config file describing a filtering pipeline for local map."
        "If not provided, and the main --config yaml file contains a "
        "`filters` entry (can be overriden with --entry-name-filters-local), it "
        "will be used instead.");

    cmd.add_option(
        "--config-filters-global", argYamlConfigFileFiltersGlobal,
        "YAML config file describing a filtering pipeline for global map."
        "If not provided, and the main --config yaml file contains a"
        "`filters` entry (can be overriden with --entry-name-filters-global), it "
        "will be used instead.");

    cmd.add_option(
        "--entry-name-filters-global", argCfgNameFiltersGlobal,
        "Overrides the map name in the YAML configuration file for global map "
        "filter.");

    cmd.add_option(
        "--entry-name-filters-local", argCfgNameFiltersLocal,
        "Overrides the map name in the YAML configuration file for local map "
        "filter.");

    cmd.add_option(
        "--guess", argInitialGuess,
        "SE(3) transformation of local wrt global, to use as initial guess for the "
        "ICP algorithm. "
        "Format: \"[x y z yaw_deg pitch_deg roll_deg]\"");

    cmd.add_flag(
        "-d,--generate-debug-log", argGenerateDebugFiles,
        "Enforces generation of the .icplog debug log files for posterior "
        "visualization with icp-log-viewer, overriding the "
        "`generateDebugFiles` value in the configuration YAML file.");

    cmd.add_flag("--profiler", argProfile, "Enables the ICP profiler.");

    cmd.add_option(
        "-l,--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins");
}

// To avoid reading the same .rawlog file twice:
static std::map<std::string, mrpt::obs::CRawlog::Ptr> rawlogsCache;

static mrpt::obs::CRawlog::Ptr load_rawlog(const std::string& filename)
{
    ASSERT_FILE_EXISTS_(filename);

    // enable loading externally-stored lazy load objects:
    mrpt::img::CImage::setImagesPathBase(mrpt::obs::CRawlog::detectImagesDirectory(filename));

    auto& r = rawlogsCache[filename];
    if (r)
    {
        return r;
    }
    r = mrpt::obs::CRawlog::Create();

    std::cout << "Loading rawlog file `" << filename << "`..." << std::endl;

    bool rawlogReadOk = r->loadFromRawLogFile(filename, true);
    ASSERT_(rawlogReadOk);

    std::cout << "Done, " << r->size() << " entries." << std::endl;

    return r;
}

static mp2p_icp_filters::GeneratorSet generators;

static mp2p_icp::metric_map_t::Ptr pc_from_rawlog(const mrpt::obs::CRawlog& r, const size_t index)
{
    ASSERT_LT_(index, r.size());

    if (generators.empty())
    {
        std::cout << "[warning] Using default mp2p_icp_filters::Generator since no "
                     "YAML file was given describing custom generators.\n";

        auto defaultGen = mp2p_icp_filters::Generator::Create();
        defaultGen->initialize({});
        generators.push_back(defaultGen);
    }

    auto pc = mp2p_icp::metric_map_t::Create();

    auto o = r.getAsGeneric(index);
    ASSERT_(o);

    if (auto sf = std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(o); sf)
    {
        // Sensory-frame format:
        mp2p_icp_filters::apply_generators(generators, *sf, *pc);
    }
    else if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(o); obs)
    {
        mp2p_icp_filters::apply_generators(generators, *obs, *pc);
    }
    else
    {
        auto e = r.getAsGeneric(index);
        THROW_EXCEPTION_FMT(
            "Rawlog index %u is neither CSensoryFrame or CObservation. Found "
            "class name: '%s'",
            static_cast<unsigned int>(index), e->GetRuntimeClass()->className);
    }

    return pc;
}

static mp2p_icp::metric_map_t::Ptr load_input_pc(const std::string& filename, bool local)
{
    // rawlog?
    if (auto extPos = filename.find(".rawlog:"); extPos != std::string::npos)
    {
        const auto sepPos      = extPos + 7;
        const auto fil         = filename.substr(0, sepPos);
        const auto rawlogIndex = std::stoi(filename.substr(sepPos + 1));

        const auto r = load_rawlog(fil);

        return pc_from_rawlog(*r, rawlogIndex);
    }

    // serialized metric_map_t object?
    if (auto extPos = filename.find(".mm"); extPos != std::string::npos)
    {
        auto r = mp2p_icp::metric_map_t::Create();

        bool readOk = r->load_from_file(filename);
        ASSERT_(readOk);

        return r;
    }

    // icplog?
    if (auto extPos = filename.find(".icplog"); extPos != std::string::npos)
    {
        mp2p_icp::LogRecord lr;
        bool                icplogFileReadOK = lr.load_from_file(filename);
        ASSERT_(icplogFileReadOK);

        auto r = mp2p_icp::metric_map_t::Create();
        *r     = local ? *lr.pcLocal : *lr.pcGlobal;
        return r;
    }

    // Otherwise: assume it's an ASCII point cloud file:
    mrpt::maps::CSimplePointsMap::Ptr points = mp2p_icp::load_xyz_file(filename);

    auto pc                                          = mp2p_icp::metric_map_t::Create();
    pc->layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = points;

    return pc;
}

void runIcp()
{
    // Load plugins first: custom map/observation classes (e.g. from a plugin
    // .so) must be registered before any .mm/.rawlog file gets deserialized.
    if (!arg_plugins.empty())
    {
        std::string errMsg;
        const auto& plugins = arg_plugins;
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
        {
            throw std::runtime_error(errMsg);
        }
    }

    const auto cfg = mrpt::containers::yaml::FromFile(argYamlConfigFile);

    // ------------------------------
    // Generators set
    // ------------------------------
    if (!argYamlConfigFileGenerators.empty())
    {
        const auto& f = argYamlConfigFileGenerators;

        generators = mp2p_icp_filters::generators_from_yaml_file(f);

        std::cout << "Created " << generators.size() << " generators from: " << f << std::endl;
    }
    else if (cfg.has("generators"))
    {
        generators = mp2p_icp_filters::generators_from_yaml(cfg["generators"]);
    }

    // ------------------------------
    // Original input point clouds
    // ------------------------------
    auto pcLocal  = load_input_pc(argInputLocal, true);
    auto pcGlobal = load_input_pc(argInputGlobal, false);

    std::cout << "Input point cloud #1: " << pcLocal->contents_summary() << std::endl;
    std::cout << "Input point cloud #2: " << pcGlobal->contents_summary() << std::endl;

    // ------------------------------
    // Build ICP pipeline:
    // ------------------------------
    auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    if (argGenerateDebugFiles)
    {
        icpParams.generateDebugFiles = true;
    }

    const auto initialGuess = mrpt::math::TPose3D::FromString(argInitialGuess);

    // -----------------------------------------
    // Apply filtering pipeline, if defined
    // -----------------------------------------
    {
        mp2p_icp_filters::FilterPipeline filtersLocal;
        if (!argYamlConfigFileFiltersLocal.empty())
        {
            filtersLocal =
                mp2p_icp_filters::filter_pipeline_from_yaml_file(argYamlConfigFileFiltersLocal);
        }
        else if (cfg.has(argCfgNameFiltersLocal))
        {
            filtersLocal = mp2p_icp_filters::filter_pipeline_from_yaml(cfg[argCfgNameFiltersLocal]);
        }
        if (!filtersLocal.empty())
        {
            mp2p_icp_filters::apply_filter_pipeline(filtersLocal, *pcLocal);
            std::cout << "Filtered local map: " << pcLocal->contents_summary() << std::endl;
        }
    }
    {
        mp2p_icp_filters::FilterPipeline filtersGlobal;
        if (!argYamlConfigFileFiltersGlobal.empty())
        {
            filtersGlobal =
                mp2p_icp_filters::filter_pipeline_from_yaml_file(argYamlConfigFileFiltersGlobal);
        }
        else if (cfg.has(argCfgNameFiltersGlobal))
        {
            filtersGlobal =
                mp2p_icp_filters::filter_pipeline_from_yaml(cfg[argCfgNameFiltersGlobal]);
        }
        if (!filtersGlobal.empty())
        {
            mp2p_icp_filters::apply_filter_pipeline(filtersGlobal, *pcGlobal);
            std::cout << "Filtered global map: " << pcGlobal->contents_summary() << std::endl;
        }
    }

    if (argProfile)
    {
        icp->profiler().enable(true);
    }

    const double t_ini = mrpt::Clock::nowDouble();

    mp2p_icp::Results icpResults;
    icp->align(*pcLocal, *pcGlobal, initialGuess, icpParams, icpResults);

    const double t_end = mrpt::Clock::nowDouble();

    std::cout << "ICP result:\n";
    icpResults.print(std::cout);

    std::cout << "- time to solve: " << mrpt::system::formatTimeInterval(t_end - t_ini) << "\n";
}

int main(int argc, char** argv)
{
    defineCliArgs();

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        runIcp();
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
