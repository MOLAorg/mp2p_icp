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
 * @file   sm-cli-partition.cpp
 * @brief  Splits a simplemap into submaps of keyframes that observe the same place.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/yaml_variables.h>
#include <mp2p_icp_filters/simplemap_partition.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

#include "sm-cli.h"

namespace
{
int printCommandsPartition(bool showErrorMsg);

// Keyframe points with their submap as the "submap" field, one point per 0.2 m
// voxel (the first one seen)
void writeColoredMap(
    const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline,
    const mp2p_icp_filters::SimplemapPartition& part, const std::string& file)
{
    constexpr double voxel = 0.2;
    auto             pc    = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("submap");
    std::unordered_set<uint64_t> seen;
    for (size_t k = 0; k < sm.size(); k++)
    {
        mp2p_icp_filters::sm2mm_options_t opts;
        opts.verbosity   = mrpt::system::LVL_WARN;
        opts.start_index = k;
        opts.end_index   = k;
        mp2p_icp::metric_map_t mm;
        mp2p_icp_filters::simplemap_to_metricmap(sm, mm, pipeline, opts);
        const auto submap = static_cast<float>(part.keyframe_submap[k]);
        for (const auto& [name, layer] : mm.layers)
        {
            const auto* src = mp2p_icp::MapToPointsMap(*layer);
            if (!src)
            {
                continue;
            }
            const auto& xs = src->getPointsBufferRef_x();
            const auto& ys = src->getPointsBufferRef_y();
            const auto& zs = src->getPointsBufferRef_z();
            for (size_t i = 0; i < src->size(); i++)
            {
                const auto ix  = static_cast<int64_t>(std::floor(xs[i] / voxel)) & 0x1FFFFF;
                const auto iy  = static_cast<int64_t>(std::floor(ys[i] / voxel)) & 0x1FFFFF;
                const auto iz  = static_cast<int64_t>(std::floor(zs[i] / voxel)) & 0x1FFFFF;
                const auto key = (static_cast<uint64_t>(ix) << 42) |
                                 (static_cast<uint64_t>(iy) << 21) | static_cast<uint64_t>(iz);
                if (seen.insert(key).second)
                {
                    pc->insertPointFast(xs[i], ys[i], zs[i]);
                    pc->insertPointField_float("submap", submap);
                }
            }
        }
    }
    pc->mark_as_modified();
    mp2p_icp::metric_map_t out;
    out.layers["submaps"] = pc;
    ASSERTMSG_(out.save_to_file(file), "Error writing --output-colored-map: " + file);
    std::cout << "Map of submaps (" << pc->size() << " points) written to: '" << file << "'\n";
}

}  // namespace

int commandPartition()
{
    const auto& lstCmds = cli->argCmd;
    if (cli->argHelp)
    {
        return printCommandsPartition(false);
    }
    if (lstCmds.size() != 3)
    {
        return printCommandsPartition(true);
    }

    const std::string inFile  = lstCmds.at(1);
    const std::string outFile = lstCmds.at(2);

    // Checked before the map is loaded: no output may overwrite the input or another output
    {
        const std::string& mapOut = cli->arg_output_colored_map;
        ASSERTMSG_(
            !same_file(outFile, inFile), "The partition file must not be the input simple-map");
        ASSERTMSG_(
            mapOut.empty() || (!same_file(mapOut, inFile) && !same_file(mapOut, outFile)),
            "--output-colored-map must not be the input simple-map nor the partition file");
    }

    mrpt::maps::CSimpleMap sm = read_input_sm_from_cli(inFile);
    ASSERT_(!sm.empty());

    // Same lazy-load base directory convention as sm2mm:
    if (const auto lazyBaseDir = mrpt::system::fileNameChangeExtension(inFile, "") + "_Images";
        mrpt::system::directoryExists(lazyBaseDir))
    {
        mrpt::io::setLazyLoadPathBase(lazyBaseDir);
        std::cout << "Using lazy-load base dir: " << lazyBaseDir << std::endl;
    }

    mp2p_icp_filters::SimplemapPartitionParams p;
    p.sample_distance         = cli->arg_sample_distance;
    p.sample_angle_deg        = cli->arg_sample_angle_deg;
    p.decimation_voxel        = cli->arg_overlap_decimation;
    p.signature.voxel_size    = cli->arg_overlap_voxel;
    p.signature.max_range     = cli->arg_overlap_max_range;
    p.partition.max_ncut      = cli->arg_max_ncut;
    p.partition.min_part_size = cli->arg_min_submap_nodes;
    if (!cli->arg_pipeline.empty())
    {
        ASSERT_FILE_EXISTS_(cli->arg_pipeline);
        p.pipeline = mp2p_icp::load_yaml_file(cli->arg_pipeline);
    }

    mp2p_icp_filters::SimplemapPartitionStats stats;
    const auto part = mp2p_icp_filters::partition_simplemap(sm, p, &stats);

    const auto groups = part.submap_keyframes();
    std::cout << mrpt::format(
        "Keyframes: %zu, nodes: %zu, node edges: %zu, submaps: %zu\n", part.keyframe_count(),
        part.node_keyframe.size(), part.node_edges.size(), groups.size());
    std::cout << mrpt::format(
        "Time: nodes %.2f s, other keyframes %.2f s, partition %.2f s (%zu node voxels)\n",
        stats.nodes_seconds, stats.keyframes_seconds, stats.partition_seconds,
        stats.signature_voxels);
    if (stats.unconverged_eigen_solves > 0)
    {
        std::cout << mrpt::format(
            "Warning: %zu Fiedler vector solves did not converge; their cuts were taken along "
            "the best vector found.\n",
            stats.unconverged_eigen_solves);
    }
    for (size_t s = 0; s < groups.size(); s++)
    {
        const auto& g = groups[s];
        // Visits: maximal runs of consecutive keyframes
        size_t visits = 0;
        for (size_t i = 0; i < g.size(); i++)
        {
            visits += (i == 0 || g[i] != g[i - 1] + 1) ? 1 : 0;
        }
        std::cout << mrpt::format(
            "Submap %zu: %zu keyframes in %zu visits, first %zu, last %zu\n", s, g.size(), visits,
            g.empty() ? 0UL : g.front(), g.empty() ? 0UL : g.back());
    }

    part.save_to_file(outFile);
    std::cout << "Partition written to: '" << outFile << "'\n";

    if (!cli->arg_output_colored_map.empty())
    {
        const auto pipeline =
            p.pipeline.empty() ? mp2p_icp_filters::decimated_keyframes_pipeline(0.2) : p.pipeline;
        writeColoredMap(sm, pipeline, part, cli->arg_output_colored_map);
    }
    return 0;
}

namespace
{
int printCommandsPartition(bool showErrorMsg)
{
    if (showErrorMsg)
    {
        setConsoleErrorColor();
        std::cerr << "Error: missing or unknown subcommand.\n";
        setConsoleNormalColor();
    }

    fprintf(
        stderr,
        R"XXX(Usage:

    sm-cli partition <input.simplemap> <output_partition.yaml>
        [--pipeline <sm2mm-pipeline.yaml>]  # keyframes to points (default: built-in)
        [--sample-distance 2.0]             # distance between graph nodes [m]
        [--sample-angle-deg 30]             # rotation between graph nodes [deg]
        [--overlap-voxel 0.5]               # voxel size of the shared content [m]
        [--overlap-max-range 0]             # ignore points farther than this [m] (0: all)
        [--overlap-decimation 0.1]          # decimation of each keyframe first [m]
        [--max-ncut 0.2]                    # split only below this normalized cut
        [--min-submap-nodes 5]              # fewest nodes on each side of a split
        [--output-colored-map <out.mm>]     # points with a 'submap' field

Splits the keyframes into submaps that observe the same place, whenever they
were visited: keyframes are sampled (graph nodes), each one is turned into
voxels in the map frame, tagged with the side of the surface seen from its
sensor (sensorPose; the mean of several) and with whether they are above
or below it, and two nodes are as similar as the voxels they share (Jaccard). The
graph is split by recursive normalized cuts. Every keyframe joins the submap
of its most similar node. Poses are used as they are: no registration.

The output is read by `sm-cli level-walls --by-submap`.

)XXX");

    return showErrorMsg ? 1 : 0;
}
}  // namespace
