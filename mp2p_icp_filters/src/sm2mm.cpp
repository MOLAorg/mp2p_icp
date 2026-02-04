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
 * @file   sm2mm.cpp
 * @brief  simplemap-to-metricmap utility function
 * @author Jose Luis Blanco Claraco
 * @date   Dec 18, 2023
 */

#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/system/progress.h>

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
#include <mp2p_icp/update_velocity_buffer_from_obs.h>
#endif

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
std::string first_n_lines(const std::string& input, std::size_t n)
{
    if (n == 0)
    {
        return {};
    }

    std::size_t pos   = 0;
    std::size_t lines = 0;

    while (lines < n)
    {
        pos = input.find('\n', pos);
        if (pos == std::string::npos)
        {
            // Fewer than n lines: return entire string
            return input;
        }
        ++pos;  // move past '\n'
        ++lines;
    }

    return input.substr(0, pos);
}
}  // namespace

void mp2p_icp_filters::simplemap_to_metricmap(
    const mrpt::maps::CSimpleMap& sm, mp2p_icp::metric_map_t& mm,
    const mrpt::containers::yaml& yamlData, const sm2mm_options_t& options)
{
    mm.clear();

    // Generators:
    mp2p_icp_filters::GeneratorSet generators;
    if (yamlData.has("generators"))
    {
        generators =
            mp2p_icp_filters::generators_from_yaml(yamlData["generators"], options.verbosity);
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
        filters =
            mp2p_icp_filters::filter_pipeline_from_yaml(yamlData["filters"], options.verbosity);
    }
    else
    {
        std::cout << "[sm2mm] Warning: no filters defined in the pipeline." << std::endl;
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
    mp2p_icp::AttachToParameterSource(filters, ps);
    mp2p_icp::AttachToParameterSource(finalFilters, ps);
    mp2p_icp::AttachToParameterSource(generators, ps);

    // Default values for twist variables:
    ps.updateVariables({{"vx", .0}, {"vy", .0}, {"vz", .0}, {"wx", .0}, {"wy", .0}, {"wz", .0}});
    ps.updateVariables(
        {{"robot_x", .0},
         {"robot_y", .0},
         {"robot_z", .0},
         {"robot_yaw", .0},
         {"robot_pitch", .0},
         {"robot_roll", .0}});
    ps.updateVariables(options.customVariables);
    ps.realize();

    // progress bar:
    if (options.showProgressBar)
    {
        std::cout << "\n";  // Needed for the VT100 codes below.
    }

    const double tStart = mrpt::Clock::nowDouble();

    size_t nKFs = sm.size();
    if (options.end_index.has_value())
    {
        mrpt::keep_min(nKFs, *options.end_index + 1);
    }

    size_t curKF = 0;
    if (options.start_index.has_value())
    {
        mrpt::keep_max(curKF, *options.start_index);
    }

    if (options.decimate_every_nth_frame.has_value() &&
        options.decimate_maximum_frame_count.has_value())
    {
        throw std::invalid_argument(
            "sm2mm: 'decimate_every_nth_frame' and 'decimate_maximum_frame_count' "
            "cannot be set simultaneously.");
    }

    // Determine the decimation stride (step size)
    size_t stride = 1;

    if (options.decimate_every_nth_frame.has_value())
    {
        stride = std::max<size_t>(1, *options.decimate_every_nth_frame);
    }
    else if (options.decimate_maximum_frame_count.has_value())
    {
        const size_t maxCount = *options.decimate_maximum_frame_count;
        if (maxCount == 0)
        {
            return;  // No frames to process
        }

        const size_t totalInRange = (nKFs > curKF) ? (nKFs - curKF) : 0;

        if (totalInRange > maxCount)
        {
            // To spread 'maxCount' frames evenly across 'totalInRange',
            // we calculate the stride as ceil(total / maxCount).
            stride = static_cast<size_t>(
                std::ceil(static_cast<double>(totalInRange) / static_cast<double>(maxCount)));
        }
    }

    size_t       processedKFs = 0;
    const size_t startIdx     = curKF;

    for (; curKF < nKFs; curKF++)
    {
        // Apply decimation logic
        if ((curKF - startIdx) % stride != 0)
        {
            continue;
        }

        // Extra safety check for the frame count limit
        if (options.decimate_maximum_frame_count.has_value() &&
            processedKFs >= *options.decimate_maximum_frame_count)
        {
            break;
        }

        // Get KF data:
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
        ASSERT_(pose);
        ASSERT_(sf);
        const mrpt::poses::CPose3D robotPose = pose->getMeanVal();

        // Update pose variables:
        ps.updateVariables(
            {{"robot_x", robotPose.x()},
             {"robot_y", robotPose.y()},
             {"robot_z", robotPose.z()},
             {"robot_yaw", robotPose.yaw()},
             {"robot_pitch", robotPose.pitch()},
             {"robot_roll", robotPose.roll()}});
        ps.realize();

        // First, search for velocity buffer data:
        for (const auto& obs : *sf)
        {
            ASSERT_(obs);
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
            mp2p_icp::update_velocity_buffer_from_obs(ps.localVelocityBuffer, obs);
#endif
        }

        try
        {
            // Next, do the actual sensor data processing:
            for (const auto& obs : *sf)
            {
                ASSERT_(obs);
                obs->load();

                bool handled = mp2p_icp_filters::apply_generators(generators, *obs, mm, robotPose);

                if (!handled)
                {
                    obs->unload();
                    continue;
                }

                // process it:
                mp2p_icp_filters::apply_filter_pipeline(filters, mm, options.profiler);
                obs->unload();
            }
        }
        catch (const std::exception& e)
        {
            // If the exception msg contains "Assert file existence failed", it's due to missing
            // external files.
            const std::string errMsg = e.what();
            if (errMsg.find("Assert file existence failed") != std::string::npos &&
                !options.throw_on_missing_external_files)
            {
                std::cerr << "[sm2mm] Keyframe #" << curKF
                          << ": skipping observation due to missing external files: "
                          << first_n_lines(errMsg, 3) << "\n";
                continue;
            }
            throw;  // Rethrow other exceptions
        }

#if 0
        // sanity checks:
        for (const auto& [name, map] : mm.layers)
        {
            const auto* pc = mp2p_icp::MapToPointsMap(*map);
            if (!pc) continue;  // not a point map
            const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(*pc);
            ASSERTMSG_(
                sanityPassed,
                mrpt::format(
                    "Sanity check did not pass for layer: '%s'", name.c_str()));
        }
#endif

        // progress bar:
        if (options.showProgressBar)
        {
            const size_t N  = nKFs;
            const double pc = static_cast<double>(curKF) / static_cast<double>(N);

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

        processedKFs++;
    }  // end for each KF.

    // Final optional filtering:
    if (!finalFilters.empty())
    {
        if (options.verbosity <= mrpt::system::LVL_INFO)
        {
            std::cout << "Applying 'final_filters'..." << std::endl;
        }

        mp2p_icp_filters::apply_filter_pipeline(finalFilters, mm, options.profiler);

        if (options.verbosity <= mrpt::system::LVL_INFO)
        {
            std::cout << "Done with 'final_filters'." << std::endl;
        }
    }
}
