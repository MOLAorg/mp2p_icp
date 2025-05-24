/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   rawlog-filter/main.cpp
 * @brief  CLI tool to apply filter pipelines to datasets in rawlog files
 * @author Jose Luis Blanco Claraco
 * @date   Oct 21, 2024
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/progress.h>

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"rawlog-filter"};

    TCLAP::ValueArg<std::string> argInput{
        "i", "input", "Input .rawlog file", true, "dataset.rawlog", "dataset.rawlog", cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o", "output", "Output .rawlo file to write to", true, "out.rawlog", "out.rawlog", cmd};

    TCLAP::ValueArg<std::string> argPipeline{
        "p",
        "pipeline",
        "YAML file with the mp2p_icp_filters pipeline to load. It must "
        "contain a `filters:` section."
        "See the app README for examples:\n"
        "https://github.com/MOLAorg/mp2p_icp/tree/develop/apps/rawlog-filter",
        true,
        "pipeline.yaml",
        "pipeline.yaml",
        cmd};

    TCLAP::ValueArg<size_t> arg_from{
        "", "from", "First rawlog index to process", false, 0, "Rawlog index", cmd};

    TCLAP::ValueArg<size_t> arg_to{
        "", "to", "Last rawlog index to process", false, 0, "Rawlog index", cmd};

    TCLAP::ValueArg<std::string> arg_lazy_load_base_dir{
        "",
        "externals-dir",
        "Lazy-load base directory for datasets with externally-stored "
        "observations",
        false,
        "dataset_Images",
        "<ExternalsDirectory>",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "",
        "INFO", cmd};
};

void run_mm_filter(Cli& cli)
{
    using namespace std::string_literals;

    ASSERT_FILE_EXISTS_(cli.argInput.getValue());
    ASSERT_FILE_EXISTS_(cli.argPipeline.getValue());

    const auto& filInput = cli.argInput.getValue();

    std::cout << "[rawlog-filter] Reading input rawlog from: '" << filInput << "'..." << std::endl;

    mrpt::obs::CRawlog dataset;

    bool readOk = dataset.loadFromRawLogFile(filInput);
    ASSERT_(readOk);

    std::cout << "[rawlog-filter] Done read dataset (" << dataset.size() << " entries)"
              << std::endl;

    // Load pipeline:
    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli.arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    }

    const auto yamlData = mrpt::containers::yaml::FromFile(cli.argPipeline.getValue());

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
    if (cli.arg_to.isSet()) mrpt::keep_min(nKFs, cli.arg_to.getValue() + 1);

    size_t curKF = 0;
    if (cli.arg_from.isSet()) mrpt::keep_max(curKF, cli.arg_from.getValue());

    // Create output Rawlog file:
    const auto filOut = cli.argOutput.getValue();
    std::cout << "[rawlog-filter] Creating output rawlog file: '" << filOut << "'..." << std::endl;

    mrpt::io::CFileGZOutputStream fo(filOut);
    auto                          outArch = mrpt::serialization::archiveFrom(fo);

    if (cli.arg_lazy_load_base_dir.isSet())
        mrpt::io::setLazyLoadPathBase(cli.arg_lazy_load_base_dir.getValue());

    for (; curKF < nKFs; curKF++)
    {
        auto obs = dataset.getAsObservation(curKF);
        ASSERTMSG_(obs, "Dataset is expected to have CObservation objects only!");

        obs->load();

        mp2p_icp::metric_map_t mm;

        bool handled = mp2p_icp_filters::apply_generators(generators, *obs, mm);

        if (!handled) continue;

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
            if (!layer) continue;
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
            const double pc = (1.0 * curKF) / N;

            const double tNow      = mrpt::Clock::nowDouble();
            const double ETA       = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
            const double totalTime = ETA + (tNow - tStart);

            std::cout << "\033[A\33[2KT\r"  // VT100 codes: cursor up and clear
                                            // line
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
