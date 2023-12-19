/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   sm2mm.cpp
 * @brief  simplemap-to-metricmap utility function
 * @author Jose Luis Blanco Claraco
 * @date   Dec 18, 2023
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/core/Clock.h>
#include <mrpt/system/progress.h>
#include <mrpt/version.h>

#include <iostream>

void mp2p_icp_filters::simplemap_to_metricmap(
    const mrpt::maps::CSimpleMap& sm, mp2p_icp::metric_map_t& mm,
    const mrpt::containers::yaml& yamlData, const sm2mm_options_t& options)
{
    mm.clear();

    // Generators:
    mp2p_icp_filters::GeneratorSet generators;
    if (yamlData.has("generators"))
    {
        generators = mp2p_icp_filters::generators_from_yaml(
            yamlData["generators"], options.verbosity);
    }
    else
    {
        std::cout << "[sm2mm] Warning: no generators defined in the pipeline, "
                     "using default generator."
                  << std::endl;

        auto defaultGen = mp2p_icp_filters::Generator::Create();
        defaultGen->setMinLoggingLevel(options.verbosity);
        defaultGen->initialize({});
        generators.push_back(defaultGen);
    }

    // Filters:
    mp2p_icp_filters::FilterPipeline filters;
    if (yamlData.has("filters"))
    {
        filters = mp2p_icp_filters::filter_pipeline_from_yaml(
            yamlData["filters"], options.verbosity);
    }
    else
    {
        std::cout << "[sm2mm] Warning: no filters defined in the pipeline."
                  << std::endl;
    }

    // Final, overall filters for the whole metric map:
    mp2p_icp_filters::FilterPipeline finalFilters;
    if (yamlData.has("final_filters"))
    {
        finalFilters = mp2p_icp_filters::filter_pipeline_from_yaml(
            yamlData["final_filters"], options.verbosity);
    }

    // sm2mm core code:

    // Parameters for twist, and possibly other user-provided variables.
    mp2p_icp::ParameterSource ps;
    mp2p_icp::AttachToParameterSource(generators, ps);
    mp2p_icp::AttachToParameterSource(filters, ps);

    // Default values for twist variables:
    ps.updateVariables(
        {{"vx", .0},
         {"vy", .0},
         {"vz", .0},
         {"wx", .0},
         {"wy", .0},
         {"wz", .0}});
    ps.updateVariables({{"robot_x", .0}, {"robot_y", .0}, {"robot_z", .0}});
    ps.updateVariables(options.customVariables);
    ps.realize();

    // progress bar:
    if (options.showProgressBar)
        std::cout << "\n";  // Needed for the VT100 codes below.

    const double tStart = mrpt::Clock::nowDouble();

    size_t nKFs = sm.size();
    if (options.end_index.has_value())
        mrpt::keep_min(nKFs, *options.end_index + 1);

    size_t curKF = 0;
    if (options.start_index.has_value())
        mrpt::keep_max(curKF, *options.start_index);

    for (; curKF < nKFs; curKF++)
    {
#if MRPT_VERSION >= 0x020b05
        const auto& [pose, sf, twist] = sm.get(curKF);
        if (twist.has_value())
        {
            ps.updateVariables(
                {{"vx", twist->vx},
                 {"vy", twist->vy},
                 {"vz", twist->vz},
                 {"wx", twist->wx},
                 {"wy", twist->wy},
                 {"wz", twist->wz}});
        }
#else
        const auto& [pose, sf] = sm.get(curKF);
#endif
        ASSERT_(pose);
        ASSERT_(sf);
        const mrpt::poses::CPose3D robotPose = pose->getMeanVal();

        // Update pose variables:
        ps.updateVariables(
            {{"robot_x", robotPose.x()},
             {"robot_y", robotPose.y()},
             {"robot_z", robotPose.z()}});
        ps.realize();

        for (const auto& obs : *sf)
        {
            ASSERT_(obs);
            obs->load();

            mp2p_icp_filters::apply_generators(generators, *obs, mm, robotPose);
            mp2p_icp_filters::apply_filter_pipeline(filters, mm);

            obs->unload();
        }

        // progress bar:
        if (options.showProgressBar)
        {
            const size_t N  = nKFs;
            const double pc = (1.0 * curKF) / N;

            const double tNow = mrpt::Clock::nowDouble();
            const double ETA  = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
            const double totalTime = ETA + (tNow - tStart);

            std::cout
                << "\033[A\33[2KT\r"  // VT100 codes: cursor up and clear
                                      // line
                << mrpt::system::progress(pc, 30)
                << mrpt::format(
                       " %6zu/%6zu (%.02f%%) ETA=%s / T=%s\n", curKF, N,
                       100 * pc, mrpt::system::formatTimeInterval(ETA).c_str(),
                       mrpt::system::formatTimeInterval(totalTime).c_str());
            std::cout.flush();
        }
    }  // end for each KF.

    // Final optional filtering:
    if (!finalFilters.empty())
    {
        std::cout << "Applying 'final_filters'..." << std::endl;

        mp2p_icp_filters::apply_filter_pipeline(finalFilters, mm);

        std::cout << "Done with 'final_filters'." << std::endl;
    }
}
