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
#include <mp2p_icp/pointcloud.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
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

    auto& r = rawlogsCache[filename];
    if (r) return r;
    r = mrpt::obs::CRawlog::Create();

    std::cout << "Loading rawlog file `" << filename << "`..." << std::endl;

    bool rawlogReadOk = r->loadFromRawLogFile(filename, true);
    ASSERT_(rawlogReadOk);

    std::cout << "Done, " << r->size() << " entries." << std::endl;

    return r;
}

static mrpt::maps::CSimplePointsMap::Ptr pc_from_rawlog(
    const mrpt::obs::CRawlog& r, const size_t index)
{
    ASSERT_LT_(index, r.size());

    auto m = mrpt::maps::CSimplePointsMap::Create();

    if (auto sf = r.getAsObservations(index); sf)
    {
        // Sensory-frame format:
        sf->insertObservationsInto(m.get());
        return m;
    }
    else if (auto obs = r.getAsObservation(index); obs)
    {
        // Observation format:
        obs->insertObservationInto(m.get());
        return m;
    }
    else
    {
        auto e = r.getAsGeneric(index);
        THROW_EXCEPTION_FMT(
            "Rawlog index %u is neither CSensoryFrame or CObservation. Found "
            "class name: '%s'",
            static_cast<unsigned int>(index), e->GetRuntimeClass()->className);
    }
}

static mrpt::maps::CSimplePointsMap::Ptr load_input_pc(
    const std::string& filename)
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
    return mp2p_icp::load_xyz_file(filename);
}

void runIcp()
{
    const auto points1 = load_input_pc(argInput1.getValue());
    const auto points2 = load_input_pc(argInput2.getValue());

    mp2p_icp::pointcloud_t pc1, pc2;
    pc1.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] = points1;
    pc2.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] = points2;

    std::cout << "Point cloud #1 size: " << pc1.size() << std::endl;
    std::cout << "Point cloud #2 size: " << pc2.size() << std::endl;

    const auto cfg =
        mrpt::containers::yaml::FromFile(argYamlConfigFile.getValue());

    const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    const auto initialGuess =
        mrpt::math::TPose3D::FromString(argInitialGuess.getValue());

    const double t_ini = mrpt::Clock::nowDouble();

    mp2p_icp::Results icpResults;
    icp->align(pc1, pc2, initialGuess, icpParams, icpResults);

    const double t_end = mrpt::Clock::nowDouble();

    std::cout
        << "ICP result:\n"
           " optimalPose_1_to_2: "
        << icpResults.optimal_tf.mean
        << "\n"
           " quality: "
        << icpResults.quality
        << "\n"
           " iterations: "
        << icpResults.nIterations
        << "\n"
           " terminationReason: "
        << mrpt::typemeta::TEnumType<mp2p_icp::IterTermReason>::value2name(
               icpResults.terminationReason)
        << "\n"
           " time to solve: "
        << mrpt::system::formatTimeInterval(t_end - t_ini) << "\n";
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
