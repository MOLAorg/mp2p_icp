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
 * @file   test-mp2p_level_by_place.cpp
 * @brief  Unit tests for simple-map partitions and leveling by place
 */

#include <mp2p_icp_filters/estimate_up_from_normals.h>
#include <mp2p_icp_filters/level_by_place.h>
#include <mp2p_icp_filters/simplemap_partition.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <set>

using namespace mp2p_icp_filters;

namespace
{
using Pts = std::vector<mrpt::math::TPoint3D>;

// Surfaces of an axis-aligned box room, every 10 cm
Pts room(double x0, double x1, double y0, double y1, double z0, double z1)
{
    constexpr double step = 0.1;
    Pts              pts;
    for (double x = x0; x <= x1 + 1e-9; x += step)
    {
        for (double y = y0; y <= y1 + 1e-9; y += step)
        {
            pts.emplace_back(x, y, z0);
            pts.emplace_back(x, y, z1);
        }
    }
    for (double z = z0 + step; z < z1 - 1e-9; z += step)
    {
        for (double x = x0; x <= x1 + 1e-9; x += step)
        {
            pts.emplace_back(x, y0, z);
            pts.emplace_back(x, y1, z);
        }
        for (double y = y0; y <= y1 + 1e-9; y += step)
        {
            pts.emplace_back(x0, y, z);
            pts.emplace_back(x1, y, z);
        }
    }
    return pts;
}

// A keyframe seeing `world` from `pose`
void addKeyframe(
    mrpt::maps::CSimpleMap& sm, const Pts& world, const mrpt::poses::CPose3D& pose, double t)
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    for (const auto& w : world)
    {
        const auto l = pose.inverseComposePoint(w);
        pc->insertPoint(static_cast<float>(l.x), static_cast<float>(l.y), static_cast<float>(l.z));
    }
    auto obs         = mrpt::obs::CObservationPointCloud::Create();
    obs->timestamp   = mrpt::Clock::fromDouble(t);
    obs->sensorLabel = "lidar";
    obs->pointcloud  = pc;
    auto sf          = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);
    auto pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
    pdf->mean = pose;
    sm.insert(pdf, sf);
}

// Two rooms side by side with a thin common wall (both faces in one voxel),
// visited A, B, A: the two visits of A must share a submap, B another one.
void test_partition_two_rooms(bool oriented)
{
    const Pts roomA = room(0.0, 8.0, 0.0, 6.0, 0.0, 3.0);
    const Pts roomB = room(8.1, 16.0, 0.0, 6.0, 0.0, 3.0);

    mrpt::maps::CSimpleMap sm;
    double                 t = 0;
    for (int k = 0; k < 15; k++, t += 1)
    {
        addKeyframe(sm, roomA, mrpt::poses::CPose3D(1.0 + 0.4 * k, 3.0, 1.2, 0, 0, 0), t);
    }
    for (int k = 0; k < 15; k++, t += 1)
    {
        addKeyframe(sm, roomB, mrpt::poses::CPose3D(9.0 + 0.4 * k, 3.0, 1.2, 0, 0, 0), t);
    }
    for (int k = 0; k < 15; k++, t += 1)
    {
        addKeyframe(sm, roomA, mrpt::poses::CPose3D(6.8 - 0.4 * k, 2.5, 1.2, 0, 0, 0), t);
    }

    SimplemapPartitionParams p;
    p.sample_distance         = 1.0;
    p.signature.oriented      = oriented;
    p.partition.min_part_size = 3;
    const auto part           = partition_simplemap(sm, p);
    ASSERT_EQUAL_(part.keyframe_count(), sm.size());

    // Largest similarity between a node of A and a node of B:
    double crossAB = 0;
    for (const auto& e : part.node_edges)
    {
        const bool aInB = part.node_keyframe[e.i] >= 15 && part.node_keyframe[e.i] < 30;
        const bool bInB = part.node_keyframe[e.j] >= 15 && part.node_keyframe[e.j] < 30;
        if (aInB != bInB)
        {
            crossAB = std::max(crossAB, e.w);
        }
    }
    std::cout << "two rooms, oriented=" << oriented << ": " << part.submap_count()
              << " submaps, largest A-B similarity " << crossAB << "\n";

    if (!oriented)
    {
        // The wall voxels are seen from both rooms: some similarity is left
        ASSERT_GT_(crossAB, 0.0);
        return;
    }
    // Different sides of the wall are different content:
    ASSERT_EQUAL_(crossAB, 0.0);
    ASSERT_EQUAL_(part.submap_count(), 2UL);
    for (size_t k = 0; k < sm.size(); k++)
    {
        const bool inB = k >= 15 && k < 30;
        ASSERT_EQUAL_(part.keyframe_submap[k], inB ? 1U : 0U);
    }

    // File round trip:
    const auto file = mrpt::system::getTempFileName() + ".yaml";
    part.save_to_file(file);
    const auto back = SimplemapPartition::load_from_file(file);
    ASSERT_(back.keyframe_submap == part.keyframe_submap);
    ASSERT_(back.node_keyframe == part.node_keyframe);
    ASSERT_(back.node_submap == part.node_submap);
    ASSERT_(back.node_voxels == part.node_voxels);
    ASSERT_EQUAL_(back.node_edges.size(), part.node_edges.size());
    ASSERT_EQUAL_(back.keyframe_links.size(), part.keyframe_links.size());
    for (size_t k = 0; k < back.keyframe_links.size(); k++)
    {
        ASSERT_EQUAL_(back.keyframe_links[k].size(), part.keyframe_links[k].size());
    }
}

// Two storeys, one above the other, a thin slab between them: the ceiling
// below and the floor above fall in the same voxels but face away.
void test_partition_two_floors()
{
    const Pts lower = room(0.0, 10.0, 0.0, 6.0, 0.0, 2.6);
    const Pts upper = room(0.0, 10.0, 0.0, 6.0, 2.7, 5.3);

    mrpt::maps::CSimpleMap sm;
    double                 t = 0;
    for (int k = 0; k < 20; k++, t += 1)
    {
        addKeyframe(sm, lower, mrpt::poses::CPose3D(1.0 + 0.4 * k, 3.0, 1.2, 0, 0, 0), t);
    }
    for (int k = 0; k < 20; k++, t += 1)
    {
        addKeyframe(sm, upper, mrpt::poses::CPose3D(9.0 - 0.4 * k, 3.0, 3.9, 0, 0, 0), t);
    }
    for (int k = 0; k < 20; k++, t += 1)
    {
        addKeyframe(sm, lower, mrpt::poses::CPose3D(1.2 + 0.4 * k, 2.5, 1.2, 0, 0, 0), t);
    }
    SimplemapPartitionParams p;
    p.sample_distance         = 1.0;
    p.partition.min_part_size = 3;
    const auto part           = partition_simplemap(sm, p);
    ASSERT_EQUAL_(part.submap_count(), 2UL);
    for (size_t k = 0; k < sm.size(); k++)
    {
        const bool upperFloor = k >= 20 && k < 40;
        ASSERT_EQUAL_(part.keyframe_submap[k], upperFloor ? 1U : 0U);
    }
}

mrpt::math::TVector3D upTilted(double pitchDeg, double rollDeg)
{
    const auto R = mrpt::poses::CPose3D::FromYawPitchRoll(
        0.0, mrpt::DEG2RAD(pitchDeg), mrpt::DEG2RAD(rollDeg));
    return R.rotateVector({0, 0, 1});
}

// A constant up: the positions rotate rigidly about the first one
void test_constant_up_is_rigid()
{
    std::vector<mrpt::poses::CPose3D> poses;
    for (int k = 0; k < 50; k++)
    {
        const double a = 0.2 * k;
        poses.emplace_back(10 * std::cos(a), 10 * std::sin(a), 0.05 * k, a, 0, 0);
    }
    const auto                         up = upTilted(1.5, -0.7);
    std::vector<mrpt::math::TVector3D> ups(poses.size(), up);
    const std::vector<WeightedEdge>    links = {{3, 40, 5.0}, {10, 30, 2.0}};
    const auto                         res   = level_poses_by_place(poses, ups, links);

    const auto C = rotation_aligning_up_to_z(up);
    for (size_t k = 0; k < poses.size(); k++)
    {
        const auto expected = poses[0].translation() +
                              C.rotateVector(poses[k].translation() - poses[0].translation());
        ASSERT_LT_((res.poses[k].translation() - expected).norm(), 1e-9);
        const auto Rexp = C.getRotationMatrix().asEigen() * poses[k].getRotationMatrix().asEigen();
        ASSERT_LT_((res.poses[k].getRotationMatrix().asEigen() - Rexp).norm(), 1e-9);
    }
    ASSERT_LT_(res.sequential_residual_max, 1e-9);
    ASSERT_LT_(res.link_residual_max, 1e-9);
}

// Out through place A into place B and back through A, each place with its
// own tilt. Leveled by place, the two visits of A keep their relative
// position; an integration in time order separates them.
void test_revisit_consistency()
{
    std::vector<mrpt::poses::CPose3D>  poses;
    std::vector<double>                times;
    std::vector<mrpt::math::TVector3D> ups;
    const auto                         upA   = upTilted(2.0, 0.5);
    const auto                         upB   = upTilted(-1.0, 1.5);
    const auto                         place = [&](double y) { return y < 10.0 ? upA : upB; };
    for (int k = 0; k <= 40; k++)  // out: y = 0..20
    {
        poses.emplace_back(0.0, 0.5 * k, 0.0, mrpt::DEG2RAD(90.0), 0, 0);
    }
    for (int k = 39; k >= 0; k--)  // back: y = 19.5..0, 0.2 m aside
    {
        poses.emplace_back(0.2, 0.5 * k, 0.0, mrpt::DEG2RAD(-90.0), 0, 0);
    }
    for (size_t k = 0; k < poses.size(); k++)
    {
        times.push_back(static_cast<double>(k));
        ups.push_back(place(poses[k].y()));
    }
    // Links between the two visits of each spot:
    std::vector<WeightedEdge> links;
    for (uint32_t k = 0; k < 40; k++)
    {
        links.push_back({k, static_cast<uint32_t>(poses.size() - 1 - k), 10.0});
    }
    const auto res = level_poses_by_place(poses, ups, links);

    // The split of each revisit pair: the change of its height difference
    double maxSplit = 0;
    for (const auto& l : links)
    {
        if (poses[l.i].y() >= 10.0)
        {
            continue;  // only place A
        }
        const double split =
            (res.poses[l.j].z() - res.poses[l.i].z()) - (poses[l.j].z() - poses[l.i].z());
        // A rigid rotation of a 0.2 m offset, at most:
        const double rigid =
            std::abs(rotation_aligning_up_to_z(upA)
                         .rotateVector(poses[l.j].translation() - poses[l.i].translation())
                         .z);
        maxSplit = std::max(maxSplit, std::abs(split) - rigid);
    }
    std::cout << "revisit split beyond the rigid part (by place): " << maxSplit * 1e3 << " mm\n";
    ASSERT_LT_(maxSplit, 1e-4);

    // Time windows of the same ups separate the visits:
    std::vector<UpSample> samples = {{20.0, upA}, {40.0, upB}, {60.0, upA}};
    const auto            byTime  = level_poses_from_up(times, poses, samples);
    double                maxTime = 0;
    for (const auto& l : links)
    {
        if (poses[l.i].y() < 10.0)
        {
            maxTime = std::max(
                maxTime,
                std::abs((byTime[l.j].z() - byTime[l.i].z()) - (poses[l.j].z() - poses[l.i].z())));
        }
    }
    std::cout << "revisit split (by time): " << maxTime * 1e3 << " mm\n";
    ASSERT_GT_(maxTime, 1e-3);

    // A dense path at the keyframe times moves with the keyframes:
    const auto dense = level_path_by_place(times, poses, times, poses, res.poses, ups);
    for (size_t k = 0; k < poses.size(); k++)
    {
        ASSERT_LT_((dense[k].translation() - res.poses[k].translation()).norm(), 1e-9);
    }
    // ...and between keyframes it stays between them:
    const std::vector<double>               tMid = {10.5};
    const std::vector<mrpt::poses::CPose3D> pMid = {mrpt::poses::CPose3D(0.0, 5.25, 0.0, 0, 0, 0)};
    const auto mid = level_path_by_place(tMid, pMid, times, poses, res.poses, ups);
    const auto avg = (res.poses[10].translation() + res.poses[11].translation()) * 0.5;
    ASSERT_LT_((mid[0].translation() - avg).norm(), 1e-9);
}

// An up estimate with the same sigma [deg] on both axes
SubmapUp est(const mrpt::math::TVector3D& up, double sigmaDeg)
{
    SubmapUp e;
    e.up        = up;
    e.cov(0, 0) = mrpt::square(mrpt::DEG2RAD(sigmaDeg));
    e.cov(1, 1) = e.cov(0, 0);
    return e;
}

void test_keyframe_ups()
{
    // A chain of 20 nodes (one keyframe each), two submaps of 10 nodes, plus
    // a keyframe without links. Neighbors share most of their content.
    SimplemapPartition p;
    for (uint32_t n = 0; n < 20; n++)
    {
        p.node_keyframe.push_back(n);
        p.node_submap.push_back(n < 10 ? 0 : 1);
        p.node_voxels.push_back(100);
        if (n + 1 < 20)
        {
            p.node_edges.push_back({n, n + 1, 0.6});
        }
    }
    p.keyframe_links.resize(21);
    for (uint32_t n = 0; n < 20; n++)
    {
        p.keyframe_links[n].emplace_back(n, 1.0);
    }
    p.keyframe_submap.assign(21, 1);
    const auto upA    = upTilted(1.0, 0.0);
    const auto upB    = upTilted(-1.0, 0.0);
    const auto tiltOf = [](const mrpt::math::TVector3D& u)
    { return mrpt::RAD2DEG(std::atan2(u.x, u.z)); };

    // Loose field: each submap keeps its own estimate
    UpFieldParams loose;
    loose.step_sigma_deg = 5.0;
    const auto a         = keyframe_ups_from_submaps(p, {est(upA, 0.05), est(upB, 0.05)}, loose);
    ASSERT_NEAR_(tiltOf(a[0]), 1.0, 0.01);
    ASSERT_NEAR_(tiltOf(a[19]), -1.0, 0.01);
    // The keyframe without links takes its neighbor's:
    ASSERT_LT_((a[20] - a[19]).norm(), 1e-12);

    // Stiff field, uncertain estimates: close to their mean everywhere
    UpFieldParams stiff;
    stiff.step_sigma_deg = 0.001;
    const auto b         = keyframe_ups_from_submaps(p, {est(upA, 0.5), est(upB, 0.5)}, stiff);
    ASSERT_LT_(std::abs(tiltOf(b[0])), 0.02);
    ASSERT_LT_(std::abs(tiltOf(b[19])), 0.02);

    // Default stiffness: a smooth transition, monotonic along the chain
    const auto c = keyframe_ups_from_submaps(p, {est(upA, 0.05), est(upB, 0.05)});
    for (size_t k = 0; k + 1 < 20; k++)
    {
        ASSERT_GE_(tiltOf(c[k]) + 1e-9, tiltOf(c[k + 1]));
    }
    ASSERT_GT_(tiltOf(c[0]), 0.0);
    ASSERT_LT_(tiltOf(c[19]), 0.0);

    // A submap without an estimate follows its neighbors:
    const auto d = keyframe_ups_from_submaps(p, {est(upA, 0.05), std::nullopt});
    ASSERT_NEAR_(tiltOf(d[19]), 1.0, 0.01);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_partition_two_rooms(true);
        test_partition_two_rooms(false);
        test_partition_two_floors();
        test_constant_up_is_rigid();
        test_revisit_consistency();
        test_keyframe_ups();
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
