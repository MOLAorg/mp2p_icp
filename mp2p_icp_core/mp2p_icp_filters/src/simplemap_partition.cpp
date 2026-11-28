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
 * @file   simplemap_partition.cpp
 * @brief  Splits a simple-map into submaps of keyframes that observe the same place.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/simplemap_partition.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

namespace mp2p_icp_filters
{
namespace
{
constexpr const char* FORMAT_NAME = "mp2p_icp_simplemap_partition";

// Observations that hold range data: their sensor is the origin of their points
bool isRangeObservation(const mrpt::obs::CObservation& obs)
{
    return IS_CLASS(obs, mrpt::obs::CObservationPointCloud) ||
           IS_CLASS(obs, mrpt::obs::CObservation3DRangeScan) ||
           IS_CLASS(obs, mrpt::obs::CObservationRotatingScan) ||
           IS_CLASS(obs, mrpt::obs::CObservationVelodyneScan) ||
           IS_CLASS(obs, mrpt::obs::CObservation2DRangeScan);
}

// All point layers of `mm`, merged
mrpt::maps::CSimplePointsMap mergedPoints(const mp2p_icp::metric_map_t& mm)
{
    mrpt::maps::CSimplePointsMap pts;
    size_t                       total = 0;
    for (const auto& [name, layer] : mm.layers)
    {
        if (const auto* pc = mp2p_icp::MapToPointsMap(*layer); pc)
        {
            total += pc->size();
        }
    }
    pts.reserve(total);
    for (const auto& [name, layer] : mm.layers)
    {
        if (const auto* pc = mp2p_icp::MapToPointsMap(*layer); pc)
        {
            const auto& xs = pc->getPointsBufferRef_x();
            const auto& ys = pc->getPointsBufferRef_y();
            const auto& zs = pc->getPointsBufferRef_z();
            for (size_t i = 0; i < pc->size(); i++)
            {
                pts.insertPointFast(xs[i], ys[i], zs[i]);
            }
        }
    }
    pts.mark_as_modified();
    return pts;
}

mrpt::maps::CSimplePointsMap pointsOf(
    const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, size_t k)
{
    sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;
    opts.start_index     = k;
    opts.end_index       = k;
    mp2p_icp::metric_map_t mm;
    simplemap_to_metricmap(sm, mm, pipeline, opts);
    return mergedPoints(mm);
}

// Signature of keyframe k, with the sensor as the origin of its points. The
// whole keyframe is converted at once, as the pipeline expects. A cloud stored
// with an identity sensorPose is in the vehicle frame (e.g. a de-skewed copy of
// a scan) and tells nothing about where the sensor is: the origin is the mean
// of the range sensors with an offset, or the vehicle frame if there are none.
KeyframeSignature signatureOf(
    const mrpt::maps::CSimpleMap& sm, const mrpt::containers::yaml& pipeline, size_t k,
    const KeyframeSignatureParams& params)
{
    const auto&          kf    = sm.get(k);
    const auto           robot = kf.pose->getMeanVal();
    mrpt::math::TPoint3D sensorSum(0, 0, 0);
    size_t               sensorCount = 0;
    for (const auto& obs : *kf.sf)
    {
        if (obs && isRangeObservation(*obs))
        {
            const auto sensor = obs->getSensorPose();
            if (sensor.translation().norm() > 1e-6)
            {
                sensorSum += (robot + sensor).translation();
                sensorCount++;
            }
        }
    }
    const auto origin = sensorCount == 0 ? robot.translation() : sensorSum * (1.0 / sensorCount);
    return keyframe_signature(pointsOf(sm, pipeline, k), origin, params);
}

double rotationAngle(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
    const auto   R = a.getRotationMatrix().asEigen().transpose() * b.getRotationMatrix().asEigen();
    const double c = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
    return std::acos(c);
}

// The `maxCount` most similar links (ties: lower node), sorted by node
void keepBest(std::vector<std::pair<uint32_t, double>>& links, size_t maxCount)
{
    if (links.size() > maxCount)
    {
        std::sort(
            links.begin(), links.end(),
            [](const auto& a, const auto& b)
            { return a.second > b.second || (a.second == b.second && a.first < b.first); });
        links.resize(maxCount);
    }
    std::sort(links.begin(), links.end());
}

// For each index, the closest index in the sequence for which `has` is true
// (the earlier one on ties), or nullopt if there is none. Linear time.
std::vector<std::optional<size_t>> closestInSequence(const std::vector<bool>& has)
{
    const size_t                       n = has.size();
    std::vector<std::optional<size_t>> before(n);
    std::vector<std::optional<size_t>> after(n);
    for (size_t k = 0; k < n; k++)
    {
        before[k] = has[k] ? std::optional<size_t>(k) : (k > 0 ? before[k - 1] : std::nullopt);
    }
    for (size_t k = n; k-- > 0;)
    {
        after[k] = has[k] ? std::optional<size_t>(k) : (k + 1 < n ? after[k + 1] : std::nullopt);
    }
    std::vector<std::optional<size_t>> out(n);
    for (size_t k = 0; k < n; k++)
    {
        if (before[k] && (!after[k] || k - *before[k] <= *after[k] - k))
        {
            out[k] = before[k];
        }
        else
        {
            out[k] = after[k];
        }
    }
    return out;
}

}  // namespace

mrpt::containers::yaml decimated_keyframes_pipeline(double voxel)
{
    ASSERT_GT_(voxel, 0.0);
    return mrpt::containers::yaml::FromText(mrpt::format(
        R"XXX(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: 'raw'
filters:
  - class_name: mp2p_icp_filters::FilterDecimateVoxels
    params:
      input_pointcloud_layer: 'raw'
      output_pointcloud_layer: 'decimated'
      voxel_filter_resolution: %f
      decimate_method: DecimateMethod::FirstPoint
  - class_name: mp2p_icp_filters::FilterDeleteLayer
    params:
      pointcloud_layer_to_remove: ['raw']
)XXX",
        voxel));
}

uint64_t simplemap_fingerprint(const mrpt::maps::CSimpleMap& sm)
{
    // FNV-1a over the keyframe count and the first stamp of each keyframe
    uint64_t   h   = 1469598103934665603ULL;
    const auto mix = [&h](uint64_t v)
    {
        for (int b = 0; b < 8; b++)
        {
            h ^= (v >> (8 * b)) & 0xFF;
            h *= 1099511628211ULL;
        }
    };
    mix(sm.size());
    for (const auto& [pose, sf, twist] : sm)
    {
        int64_t ticks = 0;
        if (sf)
        {
            for (const auto& obs : *sf)
            {
                if (obs && obs->timestamp != mrpt::Clock::time_point())
                {
                    ticks = obs->timestamp.time_since_epoch().count();
                    break;
                }
            }
        }
        mix(static_cast<uint64_t>(ticks));
    }
    return h;
}

std::size_t SimplemapPartition::submap_count() const
{
    std::size_t n = 0;
    for (const auto s : keyframe_submap)
    {
        n = std::max<std::size_t>(n, static_cast<std::size_t>(s) + 1);
    }
    for (const auto s : node_submap)
    {
        n = std::max<std::size_t>(n, static_cast<std::size_t>(s) + 1);
    }
    return n;
}

std::vector<std::vector<size_t>> SimplemapPartition::submap_keyframes() const
{
    std::vector<std::vector<size_t>> out(submap_count());
    for (size_t k = 0; k < keyframe_submap.size(); k++)
    {
        out[keyframe_submap[k]].push_back(k);
    }
    return out;
}

void SimplemapPartition::save_to_file(const std::string& file) const
{
    std::ofstream f(file);
    ASSERTMSG_(f.is_open(), "Cannot write partition file: " + file);
    f << "# Keyframe submaps of a simple-map (sm-cli partition)\n";
    f << "format: " << FORMAT_NAME << "\n";
    f << "version: 1\n";
    f << "keyframe_count: " << keyframe_count() << "\n";
    f << "map_fingerprint: '"
      << mrpt::format("%016llx", static_cast<unsigned long long>(map_fingerprint)) << "'\n";
    f << "submap_count: " << submap_count() << "\n";
    f << "# [keyframe, submap, voxels] per node\n";
    f << "nodes:\n";
    for (size_t n = 0; n < node_keyframe.size(); n++)
    {
        f << "  - [" << node_keyframe[n] << ", " << node_submap[n] << ", " << node_voxels[n]
          << "]\n";
    }
    f << "# [node_i, node_j, similarity]\n";
    f << "node_edges:\n";
    for (const auto& e : node_edges)
    {
        f << mrpt::format("  - [%u, %u, %.5f]\n", e.i, e.j, e.w);
    }
    f << "# [submap, node, similarity, node, similarity, ...] per keyframe\n";
    f << "keyframes:\n";
    for (size_t k = 0; k < keyframe_submap.size(); k++)
    {
        f << "  - [" << keyframe_submap[k];
        for (const auto& [node, w] : keyframe_links[k])
        {
            f << mrpt::format(", %u, %.5f", node, w);
        }
        f << "]\n";
    }
    f.close();
    ASSERTMSG_(!f.fail(), "Error writing partition file: " + file);
}

SimplemapPartition SimplemapPartition::load_from_file(const std::string& file)
{
    MRPT_START
    ASSERT_FILE_EXISTS_(file);
    const auto y = mrpt::containers::yaml::FromFile(file);
    ASSERTMSG_(
        y.has("format") && y["format"].as<std::string>() == FORMAT_NAME,
        "Not a simple-map partition file: " + file);
    ASSERTMSG_(y["version"].as<int>() == 1, "Unsupported partition file version: " + file);

    SimplemapPartition p;
    if (y.has("map_fingerprint"))
    {
        p.map_fingerprint = std::stoull(y["map_fingerprint"].as<std::string>(), nullptr, 16);
    }
    if (y.has("nodes") && y["nodes"].isSequence())
    {
        for (const auto& row : y["nodes"].asSequence())
        {
            const auto& r = row.asSequence();
            ASSERT_EQUAL_(r.size(), 3UL);
            p.node_keyframe.push_back(r[0].as<uint32_t>());
            p.node_submap.push_back(r[1].as<uint32_t>());
            p.node_voxels.push_back(r[2].as<uint32_t>());
        }
    }
    if (y.has("node_edges") && y["node_edges"].isSequence())
    {
        for (const auto& row : y["node_edges"].asSequence())
        {
            const auto& r = row.asSequence();
            ASSERT_EQUAL_(r.size(), 3UL);
            p.node_edges.push_back({r[0].as<uint32_t>(), r[1].as<uint32_t>(), r[2].as<double>()});
        }
    }
    ASSERTMSG_(y.has("keyframes") && y["keyframes"].isSequence(), "Missing 'keyframes': " + file);
    for (const auto& row : y["keyframes"].asSequence())
    {
        const auto& r = row.asSequence();
        ASSERT_(!r.empty() && r.size() % 2 == 1);
        p.keyframe_submap.push_back(r[0].as<uint32_t>());
        auto& links = p.keyframe_links.emplace_back();
        for (size_t i = 1; i < r.size(); i += 2)
        {
            links.emplace_back(r[i].as<uint32_t>(), r[i + 1].as<double>());
        }
    }
    ASSERT_EQUAL_(p.keyframe_count(), y["keyframe_count"].as<size_t>());
    const auto nSubmaps = y["submap_count"].as<size_t>();
    for (const auto sm : p.node_submap)
    {
        ASSERTMSG_(sm < nSubmaps, "Partition file: node submap out of range");
    }
    for (const auto sm : p.keyframe_submap)
    {
        ASSERTMSG_(sm < nSubmaps, "Partition file: keyframe submap out of range");
    }

    const size_t nNodes = p.node_keyframe.size();
    for (const auto kf : p.node_keyframe)
    {
        ASSERTMSG_(kf < p.keyframe_count(), "Partition file: node keyframe out of range");
    }
    for (const auto& e : p.node_edges)
    {
        ASSERTMSG_(e.i < nNodes && e.j < nNodes, "Partition file: edge node out of range");
    }
    for (const auto& links : p.keyframe_links)
    {
        for (const auto& [node, w] : links)
        {
            ASSERTMSG_(node < nNodes, "Partition file: keyframe link to an unknown node");
        }
    }
    return p;
    MRPT_END
}

SimplemapPartition partition_simplemap(
    const mrpt::maps::CSimpleMap& sm, const SimplemapPartitionParams& params,
    SimplemapPartitionStats* stats)
{
    MRPT_START
    const size_t N = sm.size();
    ASSERT_GT_(N, 0UL);
    ASSERT_GT_(params.max_keyframe_links, 0UL);

    std::vector<mrpt::poses::CPose3D> poses;
    poses.reserve(N);
    for (const auto& [pose, sf, twist] : sm)
    {
        ASSERT_(pose);
        poses.push_back(pose->getMeanVal());
    }

    // Nodes: sampled by distance and rotation from the previous node
    std::vector<int64_t> nodeOfKf(N, -1);
    SimplemapPartition   out;
    {
        size_t last = 0;
        for (size_t k = 0; k < N; k++)
        {
            const bool isNode =
                k == 0 ||
                (poses[k].translation() - poses[last].translation()).norm() >=
                    params.sample_distance ||
                rotationAngle(poses[last], poses[k]) >= mrpt::DEG2RAD(params.sample_angle_deg);
            if (isNode)
            {
                nodeOfKf[k] = static_cast<int64_t>(out.node_keyframe.size());
                out.node_keyframe.push_back(static_cast<uint32_t>(k));
                last = k;
            }
        }
    }

    ASSERTMSG_(
        !params.pipeline.empty() || params.decimation_voxel > 0,
        "partition_simplemap(): decimation_voxel must be > 0 with the default pipeline");
    const auto pipeline = params.pipeline.empty()
                              ? decimated_keyframes_pipeline(params.decimation_voxel)
                              : params.pipeline;
    out.map_fingerprint = simplemap_fingerprint(sm);

    const auto parallelFor = [](size_t n, size_t grain, const std::function<void(size_t)>& f)
    {
#if defined(MP2P_HAS_TBB)
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, n, grain),
            [&](const tbb::blocked_range<size_t>& r)
            {
                for (size_t i = r.begin(); i < r.end(); i++)
                {
                    f(i);
                }
            });
#else
        (void)grain;
        for (size_t i = 0; i < n; i++)
        {
            f(i);
        }
#endif
    };

    // Nodes first: signatures, then the graph in node order
    const double                   t0     = mrpt::Clock::nowDouble();
    const size_t                   nNodes = out.node_keyframe.size();
    std::vector<KeyframeSignature> nodeSigs(nNodes);
    parallelFor(
        nNodes, 1,
        [&](size_t n)
        { nodeSigs[n] = signatureOf(sm, pipeline, out.node_keyframe[n], params.signature); });

    KeyframeOverlapGraph graph(params.graph);
    size_t               voxels = 0;
    for (auto& sig : nodeSigs)
    {
        graph.add_node(sig);
        voxels += sig.size();
        out.node_voxels.push_back(static_cast<uint32_t>(sig.size()));
        KeyframeSignature().swap(sig);
    }
    out.node_edges  = graph.edges();
    const double t1 = mrpt::Clock::nowDouble();

    // Then every other keyframe: its signature is only queried, not kept
    out.keyframe_links.resize(N);
    for (const auto& e : out.node_edges)
    {
        out.keyframe_links[out.node_keyframe[e.i]].emplace_back(e.j, e.w);
        out.keyframe_links[out.node_keyframe[e.j]].emplace_back(e.i, e.w);
    }
    parallelFor(
        N, 1,
        [&](size_t k)
        {
            auto& links = out.keyframe_links[k];
            if (nodeOfKf[k] >= 0)
            {
                // Its own node always stays: keep one fewer of the others
                keepBest(links, params.max_keyframe_links - 1);
                links.emplace_back(static_cast<uint32_t>(nodeOfKf[k]), 1.0);
                std::sort(links.begin(), links.end());
                return;
            }
            links = graph.query(signatureOf(sm, pipeline, k, params.signature));
            keepBest(links, params.max_keyframe_links);
        });
    const double t2 = mrpt::Clock::nowDouble();

    GraphPartitionStats cutStats;
    out.node_submap = spectral_partition(nNodes, out.node_edges, params.partition, &cutStats);
    const double t3 = mrpt::Clock::nowDouble();

    // Keyframe submaps: that of the most similar node
    out.keyframe_submap.assign(N, 0);
    std::vector<bool> assigned(N, false);
    for (size_t k = 0; k < N; k++)
    {
        const auto& links = out.keyframe_links[k];
        if (links.empty())
        {
            continue;
        }
        const auto best = std::max_element(
            links.begin(), links.end(),
            [](const auto& a, const auto& b)
            { return a.second < b.second || (a.second == b.second && a.first > b.first); });
        out.keyframe_submap[k] = out.node_submap[best->first];
        assigned[k]            = true;
    }
    const auto closest = closestInSequence(assigned);
    for (size_t k = 0; k < N; k++)
    {
        if (!assigned[k] && closest[k])
        {
            out.keyframe_submap[k] = out.keyframe_submap[*closest[k]];
        }
    }

    if (stats)
    {
        stats->nodes_seconds            = t1 - t0;
        stats->keyframes_seconds        = t2 - t1;
        stats->partition_seconds        = t3 - t2;
        stats->signature_voxels         = voxels;
        stats->unconverged_eigen_solves = cutStats.unconverged;
    }
    return out;
    MRPT_END
}

std::vector<mrpt::math::TVector3D> keyframe_ups_from_submaps(
    const SimplemapPartition& partition, const std::vector<std::optional<SubmapUp>>& submapUp,
    const UpFieldParams& params)
{
    MRPT_START
    ASSERT_GE_(submapUp.size(), partition.submap_count());
    ASSERT_GT_(params.step_sigma_deg, 0.0);
    const size_t nNodes = partition.node_keyframe.size();
    const size_t N      = partition.keyframe_count();

    // Nodes per submap, so each estimate counts once:
    std::vector<size_t> nodesInSubmap(submapUp.size(), 0);
    for (const auto s : partition.node_submap)
    {
        nodesInSubmap[s]++;
    }
    const double floor2 = mrpt::square(mrpt::DEG2RAD(params.min_sigma_deg));
    const auto   info   = [&](const SubmapUp& e, size_t count) -> Eigen::Matrix2d
    {
        const Eigen::Matrix2d cov = e.cov.asEigen() + floor2 * Eigen::Matrix2d::Identity();
        return cov.inverse() / static_cast<double>(count);
    };
    const auto horiz = [](const SubmapUp& e)
    {
        const auto u = e.up * (1.0 / e.up.norm());
        return Eigen::Vector2d(u.x, u.y);
    };

    // Per node: the information of its measurements and its information vector
    const auto                   M = static_cast<Eigen::Index>(2 * nNodes);
    std::vector<Eigen::Matrix2d> nodeInfo(nNodes, Eigen::Matrix2d::Zero());
    std::vector<Eigen::Vector2d> nodeVec(nNodes, Eigen::Vector2d::Zero());
    Eigen::Matrix2d              sumInfo = Eigen::Matrix2d::Zero();
    Eigen::Vector2d              sumVec  = Eigen::Vector2d::Zero();
    for (size_t n = 0; n < nNodes; n++)
    {
        const auto& est = submapUp[partition.node_submap[n]];
        if (!est)
        {
            continue;
        }
        nodeInfo[n] = info(*est, nodesInSubmap[partition.node_submap[n]]);
        nodeVec[n]  = nodeInfo[n] * horiz(*est);
        sumInfo += nodeInfo[n];
        sumVec += nodeVec[n];
    }
    ASSERTMSG_(sumInfo.trace() > 0, "keyframe_ups_from_submaps(): no submap has an up vector");
    const Eigen::Vector2d mean = sumInfo.ldlt().solve(sumVec);

    // (D + L / sigma_step^2) u = D target, plus a weak pull of every node to
    // the mean, so a part of the graph without estimates is determined:
    const double                        stepSigma = mrpt::DEG2RAD(params.step_sigma_deg);
    const double                        stiff     = 1.0 / (stepSigma * stepSigma);
    const double                        weak = 1e-9 * sumInfo.trace() / static_cast<double>(nNodes);
    std::vector<Eigen::Triplet<double>> trip;
    trip.reserve(4 * nNodes + 8 * partition.node_edges.size());
    Eigen::VectorXd b = Eigen::VectorXd::Zero(M);
    for (size_t n = 0; n < nNodes; n++)
    {
        const auto i = static_cast<Eigen::Index>(2 * n);
        for (int r = 0; r < 2; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                trip.emplace_back(i + r, i + c, nodeInfo[n](r, c) + (r == c ? weak : 0.0));
            }
        }
        b.segment<2>(i) = nodeVec[n] + weak * mean;
    }
    // Edge stiffness: the content both nodes see (from their similarity and
    // sizes), relative to that of a typical node, since a correction that
    // differs between them draws all of it twice.
    ASSERT_EQUAL_(partition.node_voxels.size(), nNodes);
    std::vector<uint32_t> sizes = partition.node_voxels;
    std::sort(sizes.begin(), sizes.end());
    const double refVoxels = std::max<double>(1.0, sizes.empty() ? 1.0 : sizes[sizes.size() / 2]);
    for (const auto& e : partition.node_edges)
    {
        const double sizeSum =
            static_cast<double>(partition.node_voxels[e.i]) + partition.node_voxels[e.j];
        const double shared = e.w * sizeSum / (1.0 + e.w);
        const double w      = stiff * shared / refVoxels;
        for (int r = 0; r < 2; r++)
        {
            const auto i = static_cast<Eigen::Index>(2 * e.i + r);
            const auto j = static_cast<Eigen::Index>(2 * e.j + r);
            trip.emplace_back(i, i, w);
            trip.emplace_back(j, j, w);
            trip.emplace_back(i, j, -w);
            trip.emplace_back(j, i, -w);
        }
    }
    // A weak link between consecutive nodes, so a node without content edges
    // (an empty scan) follows its neighbors in the sequence:
    const double weakStiff = 0.01 * stiff;
    for (size_t n = 0; n + 1 < nNodes; n++)
    {
        for (int r = 0; r < 2; r++)
        {
            const auto i = static_cast<Eigen::Index>(2 * n + r);
            const auto j = static_cast<Eigen::Index>(2 * (n + 1) + r);
            trip.emplace_back(i, i, weakStiff);
            trip.emplace_back(j, j, weakStiff);
            trip.emplace_back(i, j, -weakStiff);
            trip.emplace_back(j, i, -weakStiff);
        }
    }
    Eigen::SparseMatrix<double> H(M, M);
    H.setFromTriplets(trip.begin(), trip.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H);
    ASSERTMSG_(solver.info() == Eigen::Success, "keyframe_ups_from_submaps(): the solve failed");
    const Eigen::VectorXd uh = solver.solve(b);
    ASSERTMSG_(uh.allFinite(), "keyframe_ups_from_submaps(): the solve failed");

    const auto nodeUp = [&](size_t n)
    {
        const double ux = uh[static_cast<Eigen::Index>(2 * n)];
        const double uy = uh[static_cast<Eigen::Index>(2 * n + 1)];
        return mrpt::math::TVector3D(ux, uy, std::sqrt(std::max(0.0, 1.0 - ux * ux - uy * uy)));
    };

    // Keyframes: the mean of their linked nodes
    std::vector<std::optional<mrpt::math::TVector3D>> kfUp(N);
    for (size_t k = 0; k < N; k++)
    {
        mrpt::math::TVector3D acc(0, 0, 0);
        double                wsum = 0;
        for (const auto& [node, w] : partition.keyframe_links[k])
        {
            acc += nodeUp(node) * w;
            wsum += w;
        }
        if (wsum > 0)
        {
            kfUp[k] = acc * (1.0 / acc.norm());
        }
    }
    std::vector<bool> has(N);
    for (size_t k = 0; k < N; k++)
    {
        has[k] = kfUp[k].has_value();
    }
    const auto                         closest = closestInSequence(has);
    std::vector<mrpt::math::TVector3D> out(N);
    for (size_t k = 0; k < N; k++)
    {
        out[k] = closest[k] ? *kfUp[*closest[k]] : nodeUp(0);
    }
    return out;
    MRPT_END
}

std::vector<WeightedEdge> keyframe_place_links(
    const SimplemapPartition& partition, const std::vector<mrpt::math::TPoint3D>& kfPositions,
    double maxLength, double weight)
{
    MRPT_START
    const size_t N = partition.keyframe_count();
    ASSERT_EQUAL_(kfPositions.size(), N);

    std::map<std::pair<uint32_t, uint32_t>, double> best;
    for (size_t k = 0; k < N; k++)
    {
        for (const auto& [node, w] : partition.keyframe_links[k])
        {
            const uint32_t kn = partition.node_keyframe[node];
            const auto     a  = static_cast<uint32_t>(std::min<size_t>(k, kn));
            const auto     b  = static_cast<uint32_t>(std::max<size_t>(k, kn));
            if (b - a < 2 || (kfPositions[a] - kfPositions[b]).norm() > maxLength)
            {
                continue;
            }
            auto& v = best[{a, b}];
            v       = std::max(v, weight * w);
        }
    }
    std::vector<WeightedEdge> out;
    out.reserve(best.size());
    for (const auto& [ij, w] : best)
    {
        out.push_back({ij.first, ij.second, w});
    }
    return out;
    MRPT_END
}

}  // namespace mp2p_icp_filters
