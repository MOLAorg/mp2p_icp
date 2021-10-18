/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/CImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <fstream>

// CLI flags:
static TCLAP::CmdLine cmd("mp2p-icp-run");

static TCLAP::ValueArg<std::string> argInput1(
    "1", "input1",
    "Input point cloud #1. It is interpreted as a rawlog entry if using the "
    "format `<RAWLOG_FILE.rawlog>:<N>` to select the N-th entry in the "
    "rawlog, or otherwise the filename is assumed to be a 3D pointcloud stored "
    "as a Nx3 ASCII matrix file.",
    true, "pointcloud1.txt", "pointcloud1.txt", cmd);

static TCLAP::ValueArg<std::string> argInput2(
    "2", "input2", "Input point cloud #2. Same format than --input1. ", true,
    "pointcloud2.txt", "pointcloud2.txt", cmd);

static TCLAP::ValueArg<std::string> argYamlConfigFile(
    "c", "config", "YAML config file describing the ICP pipeline", true,
    "icp-config.yaml", "icp-config.yaml", cmd);

static TCLAP::ValueArg<std::string> argYamlConfigFileGenerators(
    "", "config-generators",
    "YAML config file describing the Generators. "
    "Can be used when processing a rawlog as input; if not present, a default "
    "Generator object will be used.",
    false, "generators-config.yaml", "generators-config.yaml", cmd);

static TCLAP::ValueArg<std::string> argYamlConfigFileFilters1(
    "", "config-filters-pc1",
    "YAML config file describing a filtering pipeline for point cloud #1. ",
    false, "filters-config.yaml", "filters-config.yaml", cmd);

static TCLAP::ValueArg<std::string> argYamlConfigFileFilters2(
    "", "config-filters-pc2",
    "YAML config file describing a filtering pipeline for point cloud #2. ",
    false, "filters-config.yaml", "filters-config.yaml", cmd);

static TCLAP::ValueArg<std::string> argInitialGuess(
    "", "guess",
    "SE(3) transformation to use as initial guess for the ICP algorithm. "
    "Format: \"[x y z yaw_deg pitch_deg roll_deg]\"",
    false, "[0 0 0 0 0 0]", "[0 0 0 0 0 0]", cmd);

// To avoid reading the same .rawlog file twice:
static std::map<std::string, mrpt::obs::CRawlog::Ptr> rawlogsCache;

static mrpt::obs::CRawlog::Ptr load_rawlog(const std::string& filename)
{
    ASSERT_FILE_EXISTS_(filename);

    // enable loading externally-stored lazy load objects:
    mrpt::img::CImage::setImagesPathBase(
        mrpt::obs::CRawlog::detectImagesDirectory(filename));

    auto& r = rawlogsCache[filename];
    if (r) return r;
    r = mrpt::obs::CRawlog::Create();

    std::cout << "Loading rawlog file `" << filename << "`..." << std::endl;

    bool rawlogReadOk = r->loadFromRawLogFile(filename, true);
    ASSERT_(rawlogReadOk);

    std::cout << "Done, " << r->size() << " entries." << std::endl;

    return r;
}

static mp2p_icp_filters::GeneratorSet generators;

static mp2p_icp::metric_map_t::Ptr pc_from_rawlog(
    const mrpt::obs::CRawlog& r, const size_t index)
{
    ASSERT_LT_(index, r.size());

    if (generators.empty())
    {
        std::cout
            << "[warning] Using default mp2p_icp_filters::Generator since no "
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
    else if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(o);
             obs)
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

static mp2p_icp::metric_map_t::Ptr load_input_pc(const std::string& filename)
{
    if (auto extPos = filename.find(".rawlog:"); extPos != std::string::npos)
    {
        const auto sepPos      = extPos + 7;
        const auto fil         = filename.substr(0, sepPos);
        const auto rawlogIndex = std::stod(filename.substr(sepPos + 1));

        const auto r = load_rawlog(fil);

        return pc_from_rawlog(*r, rawlogIndex);
    }

    // Assume it's an ASCII point cloud file:
    mrpt::maps::CSimplePointsMap::Ptr points =
        mp2p_icp::load_xyz_file(filename);

    auto pc = mp2p_icp::metric_map_t::Create();
    pc->layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = points;

    return pc;
}

void runIcp()
{
    // ------------------------------
    // Generators set
    // ------------------------------
    if (argYamlConfigFileGenerators.isSet())
    {
        const auto& f = argYamlConfigFileGenerators.getValue();

        generators = mp2p_icp_filters::generators_from_yaml_file(f);

        std::cout << "Created " << generators.size()
                  << " generators from: " << f << std::endl;
    }

    // ------------------------------
    // Original input point clouds
    // ------------------------------
    auto pc1 = load_input_pc(argInput1.getValue());
    auto pc2 = load_input_pc(argInput2.getValue());

    std::cout << "Input point cloud #1: " << pc1->contents_summary()
              << std::endl;
    std::cout << "Input point cloud #2: " << pc2->contents_summary()
              << std::endl;

    // ------------------------------
    // Build ICP pipeline:
    // ------------------------------
    const auto cfg =
        mrpt::containers::yaml::FromFile(argYamlConfigFile.getValue());

    const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    const auto initialGuess =
        mrpt::math::TPose3D::FromString(argInitialGuess.getValue());

    // -----------------------------------------
    // Apply filtering pipeline, if defined
    // -----------------------------------------
    {
        mp2p_icp_filters::FilterPipeline filters1;
        if (argYamlConfigFileFilters1.isSet())
        {
            filters1 = mp2p_icp_filters::filter_pipeline_from_yaml_file(
                argYamlConfigFileFilters1.getValue());
        }
        else if (cfg.has("filters_pc1"))
        {
            filters1 =
                mp2p_icp_filters::filter_pipeline_from_yaml(cfg["filters_pc1"]);
        }
        if (!filters1.empty())
        {
            mp2p_icp_filters::apply_filter_pipeline(filters1, *pc1);
            std::cout << "Filtered point cloud #1: " << pc1->contents_summary()
                      << std::endl;
        }
    }
    {
        mp2p_icp_filters::FilterPipeline filters2;
        if (argYamlConfigFileFilters2.isSet())
        {
            filters2 = mp2p_icp_filters::filter_pipeline_from_yaml_file(
                argYamlConfigFileFilters2.getValue());
        }
        else if (cfg.has("filters_pc2"))
        {
            filters2 =
                mp2p_icp_filters::filter_pipeline_from_yaml(cfg["filters_pc2"]);
        }
        if (!filters2.empty())
        {
            mp2p_icp_filters::apply_filter_pipeline(filters2, *pc2);
            std::cout << "Filtered point cloud #2: " << pc2->contents_summary()
                      << std::endl;
        }
    }

    const double t_ini = mrpt::Clock::nowDouble();

    mp2p_icp::Results icpResults;
    icp->align(*pc1, *pc2, initialGuess, icpParams, icpResults);

    const double t_end = mrpt::Clock::nowDouble();

    std::cout
        << "ICP result:\n"
           " optimalPose_1_to_2: "
        << icpResults.optimal_tf.mean
        << "\n"
           " quality: "
        << 100 * icpResults.quality
        << " %\n"
           " iterations: "
        << icpResults.nIterations
        << "\n"
           " terminationReason: "
        << mrpt::typemeta::TEnumType<mp2p_icp::IterTermReason>::value2name(
               icpResults.terminationReason)
        << "\n"
           " time to solve: "
        << mrpt::system::formatTimeInterval(t_end - t_ini) << "\n"
        << " finalPairings: " << icpResults.finalPairings.contents_summary()
        << "\n";
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        runIcp();
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
    return 0;
}
