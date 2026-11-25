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
 * @file   test-mp2p_plane_segmentation.cpp
 * @brief  Unit tests for segment_planes() and FilterPlaneSegmentation
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 *
 * Set MP2P_PLANE_SEGMENTATION_BENCHMARK=1 to also time a 1e7 point scene.
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterPlaneSegmentation.h>
#include <mp2p_icp_filters/plane_segmentation.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/geometry.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/CTicTac.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#if defined(MP2P_HAS_TBB)
#include <tbb/global_control.h>
#endif

namespace
{
using mp2p_icp_filters::PlaneSegmentationParams;
using mp2p_icp_filters::PlaneSegmentationResult;
using mp2p_icp_filters::segment_planes;
using mp2p_icp_filters::SegmentedPlane;
using mrpt::math::TPoint3D;
using mrpt::math::TVector3D;

/// Adds random points on surfaces and in volumes, with isotropic noise.
class SceneBuilder
{
   public:
    explicit SceneBuilder(unsigned int seed, mrpt::maps::CPointsMap::Ptr pc = {})
        : pc_(pc ? pc : mrpt::maps::CSimplePointsMap::Create())
    {
        rng_.randomize(seed);
    }

    /// Rectangle `o + s*a + t*b`, s,t in [0,1], with `density` points/m2.
    void rect(
        const TPoint3D& o, const TVector3D& a, const TVector3D& b, double density, double noise)
    {
        const double area = mrpt::math::crossProduct3D(a, b).norm();
        const auto   n    = static_cast<std::size_t>(area * density);
        for (std::size_t i = 0; i < n; i++)
        {
            const double s = rng_.drawUniform(0.0, 1.0);
            const double t = rng_.drawUniform(0.0, 1.0);
            add(o + a * s + b * t, noise);
        }
    }

    /// Vertical cylinder sector of radius R about (cx, cy), angles [a0, a1],
    /// z in [z0, z1].
    void cylinder(
        double cx, double cy, double R, double a0, double a1, double z0, double z1, double density,
        double noise)
    {
        const double area = R * (a1 - a0) * (z1 - z0);
        const auto   n    = static_cast<std::size_t>(area * density);
        for (std::size_t i = 0; i < n; i++)
        {
            const double ang = rng_.drawUniform(a0, a1);
            const double z   = rng_.drawUniform(z0, z1);
            add({cx + R * std::cos(ang), cy + R * std::sin(ang), z}, noise);
        }
    }

    /// Gaussian blob (vegetation-like clutter).
    void blob(const TPoint3D& c, double sigma, std::size_t n)
    {
        for (std::size_t i = 0; i < n; i++)
        {
            add(c, sigma);
        }
    }

    void add(const TPoint3D& p, double noise)
    {
        pc_->insertPointFast(
            static_cast<float>(p.x + rng_.drawGaussian1D(0, noise)),
            static_cast<float>(p.y + rng_.drawGaussian1D(0, noise)),
            static_cast<float>(p.z + rng_.drawGaussian1D(0, noise)));
    }

    mrpt::maps::CPointsMap::Ptr done()
    {
        pc_->mark_as_modified();
        return pc_;
    }

    mrpt::random::CRandomGenerator& rng() { return rng_; }

   private:
    mrpt::maps::CPointsMap::Ptr    pc_;
    mrpt::random::CRandomGenerator rng_;
};

constexpr double ROOM_X0 = -3.0;
constexpr double ROOM_X1 = 3.0;
constexpr double ROOM_Y0 = -2.0;
constexpr double ROOM_Y1 = 2.0;
constexpr double ROOM_Z0 = -1.0;
constexpr double ROOM_Z1 = 1.5;

void addRoom(SceneBuilder& sb, double density, double noise)
{
    const double dx = ROOM_X1 - ROOM_X0;
    const double dy = ROOM_Y1 - ROOM_Y0;
    const double dz = ROOM_Z1 - ROOM_Z0;
    sb.rect({ROOM_X0, ROOM_Y0, ROOM_Z0}, {dx, 0, 0}, {0, dy, 0}, density, noise);  // floor
    sb.rect({ROOM_X0, ROOM_Y0, ROOM_Z1}, {dx, 0, 0}, {0, dy, 0}, density, noise);  // ceiling
    sb.rect({ROOM_X0, ROOM_Y0, ROOM_Z0}, {dx, 0, 0}, {0, 0, dz}, density, noise);
    sb.rect({ROOM_X0, ROOM_Y1, ROOM_Z0}, {dx, 0, 0}, {0, 0, dz}, density, noise);
    sb.rect({ROOM_X0, ROOM_Y0, ROOM_Z0}, {0, dy, 0}, {0, 0, dz}, density, noise);
    sb.rect({ROOM_X1, ROOM_Y0, ROOM_Z0}, {0, dy, 0}, {0, 0, dz}, density, noise);
}

double angleDeg(const TVector3D& a, const TVector3D& b)
{
    const double c = std::abs(a.x * b.x + a.y * b.y + a.z * b.z) / (a.norm() * b.norm());
    return mrpt::RAD2DEG(std::acos(std::min(1.0, c)));
}

/// Index of the plane best matching (n, d) (sign-agnostic), or -1.
int findPlane(
    const PlaneSegmentationResult& r, const TVector3D& n, double d, double maxDeg, double maxOff)
{
    for (std::size_t k = 0; k < r.planes.size(); k++)
    {
        const auto&  pl  = r.planes[k];
        const double dot = pl.normal.x * n.x + pl.normal.y * n.y + pl.normal.z * n.z;
        const double off = dot > 0 ? pl.offset : -pl.offset;
        if (angleDeg(pl.normal, n) < maxDeg && std::abs(off - d) < maxOff)
        {
            return static_cast<int>(k);
        }
    }
    return -1;
}

void printPlanes(const PlaneSegmentationResult& r, std::size_t maxRows = 12)
{
    std::cout << "  " << r.num_input_points << " pts, " << r.num_voxels << " voxels, "
              << r.num_normals << " normals, " << r.num_regions << " regions, " << r.num_merged
              << " merged, " << r.planes.size() << " planes\n";
    for (std::size_t k = 0; k < r.planes.size() && k < maxRows; k++)
    {
        const auto& p = r.planes[k];
        std::cout << mrpt::format(
            "   #%zu n=(%+.4f %+.4f %+.4f) d=%+.4f pts=%zu area=%.2f rms=%.4f p95=%.4f "
            "obb=[%.2f %.2f]x[%.2f %.2f]\n",
            k, p.normal.x, p.normal.y, p.normal.z, p.offset, p.num_points(), p.area,
            p.thickness_rms, p.thickness_p95, p.u_min, p.u_max, p.v_min, p.v_max);
    }
}

// ---------------------------------------------------------------------------
void test_box_room()
{
    SceneBuilder sb(1U);
    addRoom(sb, 2500.0, 0.005);
    const auto pc = sb.done();

    const auto r = segment_planes(*pc);
    std::cout << "[test_box_room]\n";
    printPlanes(r);

    ASSERT_EQUAL_(r.planes.size(), 6UL);

    struct Face
    {
        TVector3D n;
        double    d;
        double    area;
    };
    const double dx      = ROOM_X1 - ROOM_X0;
    const double dy      = ROOM_Y1 - ROOM_Y0;
    const double dz      = ROOM_Z1 - ROOM_Z0;
    const Face   faces[] = {{{0, 0, 1}, ROOM_Z0, dx * dy}, {{0, 0, 1}, ROOM_Z1, dx * dy},
                            {{0, 1, 0}, ROOM_Y0, dx * dz}, {{0, 1, 0}, ROOM_Y1, dx * dz},
                            {{1, 0, 0}, ROOM_X0, dy * dz}, {{1, 0, 0}, ROOM_X1, dy * dz}};

    std::vector<int> used(r.planes.size(), 0);
    for (const auto& f : faces)
    {
        const int k = findPlane(r, f.n, f.d, 0.3, 0.005);
        ASSERTMSG_(k >= 0, "A face of the room was not found");
        ASSERT_EQUAL_(used[k], 0);
        used[k]         = 1;
        const auto&  pl = r.planes[k];
        const double rA = pl.area / f.area;
        ASSERT_GT_(rA, 0.85);
        ASSERT_LT_(rA, 1.10);
        ASSERT_LT_(pl.thickness_rms, 0.008);
        ASSERT_GT_(pl.thickness_p95, pl.thickness_rms);
        // The origin, inside the room, is on the positive side of every face:
        ASSERT_LT_(pl.offset, 0.0);
        // Right-handed in-plane basis:
        const auto c = mrpt::math::crossProduct3D(pl.axis_u, pl.axis_v);
        ASSERT_LT_((c - pl.normal).norm(), 1e-9);
        // Inliers are sorted and unique:
        ASSERT_(std::is_sorted(pl.inliers.begin(), pl.inliers.end()));
        ASSERT_(std::adjacent_find(pl.inliers.begin(), pl.inliers.end()) == pl.inliers.end());
        // Covariance: positive diagonal, and the offset sigma matches sigma/sqrt(N)
        ASSERT_GT_(pl.covariance(3, 3), 0.0);
        ASSERT_LT_(
            std::abs(
                pl.tangent_variances[2] -
                pl.thickness_rms * pl.thickness_rms / static_cast<double>(pl.num_points())),
            1e-15);
    }
    // No point in two planes, and almost all points in one:
    std::vector<uint8_t> seen(pc->size(), 0);
    std::size_t          total = 0;
    for (const auto& pl : r.planes)
    {
        for (const auto i : pl.inliers)
        {
            ASSERT_EQUAL_(seen[i], 0);
            seen[i] = 1;
            total++;
        }
    }
    ASSERT_GT_(static_cast<double>(total), 0.97 * static_cast<double>(pc->size()));
}

// ---------------------------------------------------------------------------
void test_noisy_tilted_ground()
{
    // A 30x30 m ground with 2 cm noise, tilted by 3 deg, plus vegetation-like
    // clutter above it.
    const double    tilt = mrpt::DEG2RAD(3.0);
    const TVector3D a    = {30.0 * std::cos(tilt), 0, 30.0 * std::sin(tilt)};
    const TVector3D b    = {0, 30.0, 0};
    const TPoint3D  o    = {-15.0 * std::cos(tilt), -15.0, -1.5 - 15.0 * std::sin(tilt)};

    SceneBuilder sb(2U);
    sb.rect(o, a, b, 400.0, 0.02);
    const std::size_t nGround = sb.done()->size();
    for (int i = 0; i < 20; i++)
    {
        const double x = sb.rng().drawUniform(-12.0, 12.0);
        const double y = sb.rng().drawUniform(-12.0, 12.0);
        sb.blob({x, y, -1.5 + x * std::tan(tilt) + 1.2}, 0.4, 1500);
    }
    const auto pc = sb.done();

    const auto r = segment_planes(*pc);
    std::cout << "[test_noisy_tilted_ground]\n";
    printPlanes(r, 4);

    ASSERT_GE_(r.planes.size(), 1UL);
    const auto&     g = r.planes.front();
    const TVector3D nTrue{-std::sin(tilt), 0, std::cos(tilt)};
    ASSERT_LT_(angleDeg(g.normal, nTrue), 0.1);
    // Normal points up, towards the origin which is above the ground:
    ASSERT_GT_(g.normal.z, 0.0);
    // Ground passes through (0,0,-1.5):
    ASSERT_LT_(std::abs(g.offset - (nTrue.z * -1.5)), 0.005);
    ASSERT_GT_(g.thickness_rms, 0.017);
    ASSERT_LT_(g.thickness_rms, 0.023);
    ASSERT_GT_(static_cast<double>(g.num_points()), 0.97 * static_cast<double>(nGround));
    ASSERT_LT_(static_cast<double>(g.num_points()), 1.01 * static_cast<double>(nGround));
    ASSERT_GT_(g.area, 0.9 * 900);
    // The blobs are not planes:
    for (std::size_t k = 1; k < r.planes.size(); k++)
    {
        ASSERT_LT_(r.planes[k].area, 5.0);
    }
}

// ---------------------------------------------------------------------------
void test_parallel_walls_stay_apart()
{
    // A floor, and two parallel walls 10 cm apart (e.g. a panel in front of
    // a wall). Several placements relative to the voxel grid, including the
    // worst case where each wall straddles a voxel boundary.
    for (const double x0 : {3.0, 3.025, 3.05, 3.075})
    {
        SceneBuilder sb(3U);
        sb.rect({-5.0, -5.0, 0.0}, {10.0, 0, 0}, {0, 10.0, 0}, 1000.0, 0.005);
        sb.rect({x0, -2.0, 0.2}, {0, 4.0, 0}, {0, 0, 2.5}, 2500.0, 0.005);
        sb.rect({x0 + 0.10, -2.0, 0.2}, {0, 4.0, 0}, {0, 0, 2.5}, 2500.0, 0.005);
        const auto pc = sb.done();

        const auto r = segment_planes(*pc);
        std::cout << "[test_parallel_walls_stay_apart] x0=" << x0 << "\n";
        printPlanes(r);

        const int kA = findPlane(r, {1, 0, 0}, x0, 0.5, 0.01);
        const int kB = findPlane(r, {1, 0, 0}, x0 + 0.10, 0.5, 0.01);
        ASSERTMSG_(kA >= 0 && kB >= 0, "Both walls must be found");
        ASSERT_NOT_EQUAL_(kA, kB);
        // No other plane parallel to them (e.g. a phantom one in between):
        std::size_t nParallel = 0;
        for (const auto& pl : r.planes)
        {
            if (angleDeg(pl.normal, {1, 0, 0}) < 10.0)
            {
                nParallel++;
            }
        }
        ASSERT_EQUAL_(nParallel, 2UL);
        for (const int k : {kA, kB})
        {
            const auto& pl = r.planes[k];
            ASSERT_LT_(pl.thickness_rms, 0.008);
            ASSERT_GT_(pl.area, 0.85 * 10.0);
            ASSERT_LT_(pl.area, 1.10 * 10.0);
        }
        ASSERT_GE_(findPlane(r, {0, 0, 1}, 0.0, 0.5, 0.01), 0);
    }
}

// ---------------------------------------------------------------------------
void test_curved_surface_is_split_or_rejected()
{
    for (const double R : {10.0, 20.0})
    {
        // An 8 m wide, 3 m high cylinder sector: no plane may cover most of it.
        const double halfAng = 4.0 / R;
        SceneBuilder sb(4U);
        sb.cylinder(-R, 0.0, R, -halfAng, halfAng, -1.0, 2.0, 2500.0, 0.003);
        const auto        pc    = sb.done();
        const std::size_t total = pc->size();

        const auto r = segment_planes(*pc);
        std::cout << "[test_curved_surface_is_split_or_rejected] R=" << R << "\n";
        printPlanes(r);

        for (const auto& pl : r.planes)
        {
            ASSERT_LT_(static_cast<double>(pl.num_points()), 0.6 * static_cast<double>(total));
            ASSERT_LE_(pl.thickness_rms, PlaneSegmentationParams().max_thickness);
        }
    }
}

// ---------------------------------------------------------------------------
/// Ray-casts a 16-ring LiDAR (+-15 deg, 0.2 deg azimuth) inside an axis
/// aligned box, from several poses.
mrpt::maps::CPointsMap::Ptr makeSparseLidarScans(
    const TPoint3D& boxMin, const TPoint3D& boxMax, const std::vector<TPoint3D>& poses,
    double rangeNoise, unsigned int seed)
{
    SceneBuilder sb(seed);
    for (const auto& s : poses)
    {
        for (int ring = 0; ring < 16; ring++)
        {
            const double el = mrpt::DEG2RAD(-15.0 + 2.0 * ring);
            for (int az = 0; az < 1800; az++)
            {
                const double    a = mrpt::DEG2RAD(0.2 * az);
                const TVector3D d{
                    std::cos(el) * std::cos(a), std::cos(el) * std::sin(a), std::sin(el)};
                double t = 1e9;
                for (int k = 0; k < 3; k++)
                {
                    const double dk = k == 0 ? d.x : (k == 1 ? d.y : d.z);
                    const double sk = k == 0 ? s.x : (k == 1 ? s.y : s.z);
                    const double lo = k == 0 ? boxMin.x : (k == 1 ? boxMin.y : boxMin.z);
                    const double hi = k == 0 ? boxMax.x : (k == 1 ? boxMax.y : boxMax.z);
                    if (std::abs(dk) < 1e-9)
                    {
                        continue;
                    }
                    const double tk = ((dk > 0 ? hi : lo) - sk) / dk;
                    t               = std::min(t, tk);
                }
                const double tn = t + sb.rng().drawGaussian1D(0, rangeNoise);
                sb.add({s.x + tn * d.x, s.y + tn * d.y, s.z + tn * d.z}, 0.0);
            }
        }
    }
    return sb.done();
}

void test_sparse_multiring_lidar()
{
    // A 20x14x4 m hall, scanned by a 16-ring LiDAR from 3 poses (ring spacing
    // on the floor: 0.6 to 1.8 m). The voxel size must follow the sparsity.
    const TPoint3D boxMin{-10.0, -7.0, -1.0};
    const TPoint3D boxMax{10.0, 7.0, 3.0};
    const auto     pc = makeSparseLidarScans(
            boxMin, boxMax, {{-3.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {3.0, -1.0, 0.0}}, 0.01, 5U);

    PlaneSegmentationParams p;
    p.voxel_size    = 0.20;
    p.normal_radius = 1.5;
    const auto r    = segment_planes(*pc, p);
    std::cout << "[test_sparse_multiring_lidar]\n";
    printPlanes(r);

    ASSERT_GE_(findPlane(r, {0, 0, 1}, boxMin.z, 0.5, 0.01), 0);
    ASSERT_GE_(findPlane(r, {1, 0, 0}, boxMin.x, 0.5, 0.01), 0);
    ASSERT_GE_(findPlane(r, {1, 0, 0}, boxMax.x, 0.5, 0.01), 0);
    ASSERT_GE_(findPlane(r, {0, 1, 0}, boxMin.y, 0.5, 0.01), 0);
    ASSERT_GE_(findPlane(r, {0, 1, 0}, boxMax.y, 0.5, 0.01), 0);
}

// ---------------------------------------------------------------------------
/// A 100x100 m block: tilted ground, buildings, trees.
mrpt::maps::CPointsMap::Ptr makeCityBlock(std::size_t nPoints, unsigned int seed)
{
    SceneBuilder sb(seed);
    auto&        rng = sb.rng();

    struct Building
    {
        double x, y, w, l, h;
    };
    std::vector<Building> bs;
    for (int i = 0; i < 12; i++)
    {
        const double x = -40.0 + 26.0 * (i % 4) + rng.drawUniform(-3.0, 3.0);
        const double y = -35.0 + 30.0 * (i / 4) + rng.drawUniform(-3.0, 3.0);
        bs.push_back(
            {x, y, rng.drawUniform(8.0, 14.0), rng.drawUniform(8.0, 14.0),
             rng.drawUniform(6.0, 15.0)});
    }
    // Surface area, to spread the points uniformly:
    double area = 100.0 * 100.0;
    for (const auto& b : bs)
    {
        area += 2 * (b.w + b.l) * b.h + b.w * b.l;
    }
    const double nSurf   = 0.9 * static_cast<double>(nPoints);
    const double density = nSurf / area;
    const double noise   = 0.02;
    const double slope   = std::tan(mrpt::DEG2RAD(1.0));

    sb.rect(
        {-50.0, -50.0, -50.0 * slope}, {100.0, 0, 100.0 * slope}, {0, 100.0, 0}, density, noise);
    for (const auto& b : bs)
    {
        const double z0 = b.x * slope;
        sb.rect({b.x, b.y, z0}, {b.w, 0, 0}, {0, 0, b.h}, density, noise);
        sb.rect({b.x, b.y + b.l, z0}, {b.w, 0, 0}, {0, 0, b.h}, density, noise);
        sb.rect({b.x, b.y, z0}, {0, b.l, 0}, {0, 0, b.h}, density, noise);
        sb.rect({b.x + b.w, b.y, z0}, {0, b.l, 0}, {0, 0, b.h}, density, noise);
        sb.rect({b.x, b.y, z0 + b.h}, {b.w, 0, 0}, {0, b.l, 0}, density, noise);
    }
    const std::size_t nTrees = 40;
    const std::size_t perTree =
        static_cast<std::size_t>(0.1 * static_cast<double>(nPoints)) / nTrees;
    for (std::size_t i = 0; i < nTrees; i++)
    {
        const double x = rng.drawUniform(-48.0, 48.0);
        const double y = rng.drawUniform(-48.0, 48.0);
        sb.blob({x, y, x * slope + 4.0}, 1.5, perTree);
    }
    return sb.done();
}

void time_city_block(std::size_t nPoints)
{
    const auto pc = makeCityBlock(nPoints, 6U);

    mrpt::system::CTicTac tic;
    const auto            r = segment_planes(*pc);
    const double          t = tic.Tac();

    std::size_t nInl = 0;
    for (const auto& pl : r.planes)
    {
        nInl += pl.num_points();
    }
    std::cout << mrpt::format(
        "[time_city_block] %zu points: %zu planes, %.1f%% of the points, %.3f s\n", pc->size(),
        r.planes.size(), 100.0 * static_cast<double>(nInl) / static_cast<double>(pc->size()), t);
    printPlanes(r, 5);

    // The ground is the largest plane:
    ASSERT_GE_(r.planes.size(), 12UL * 4UL);
    const TVector3D nG{-std::sin(mrpt::DEG2RAD(1.0)), 0, std::cos(mrpt::DEG2RAD(1.0))};
    ASSERT_LT_(angleDeg(r.planes.front().normal, nG), 0.1);
}

// ---------------------------------------------------------------------------
void expect_identical(const PlaneSegmentationResult& a, const PlaneSegmentationResult& b)
{
    ASSERT_EQUAL_(a.planes.size(), b.planes.size());
    ASSERT_EQUAL_(a.num_regions, b.num_regions);
    ASSERT_EQUAL_(a.num_merged, b.num_merged);
    for (std::size_t k = 0; k < a.planes.size(); k++)
    {
        const auto& pa = a.planes[k];
        const auto& pb = b.planes[k];
        ASSERT_(pa.inliers == pb.inliers);
        const double va[] = {pa.normal.x, pa.normal.y,      pa.normal.z,     pa.offset,
                             pa.area,     pa.thickness_rms, pa.thickness_p95};
        const double vb[] = {pb.normal.x, pb.normal.y,      pb.normal.z,     pb.offset,
                             pb.area,     pb.thickness_rms, pb.thickness_p95};
        ASSERTMSG_(std::memcmp(va, vb, sizeof(va)) == 0, "Plane parameters are not bit-identical");
    }
}

void test_determinism()
{
    // Room with clutter, so that seeds, merges and point competition happen.
    SceneBuilder sb(7U);
    addRoom(sb, 2500.0, 0.01);
    sb.blob({1.0, 0.5, 0.0}, 0.3, 20000);
    sb.rect({0.0, -1.0, -0.2}, {1.2, 0, 0}, {0, 0.8, 0}, 2500.0, 0.005);  // a table
    const auto pc = sb.done();

    const auto ref = segment_planes(*pc);
    std::cout << "[test_determinism] " << ref.planes.size() << " planes\n";
    expect_identical(ref, segment_planes(*pc));

#if defined(MP2P_HAS_TBB)
    for (const int nThreads : {1, 2, 3, 8})
    {
        tbb::global_control gc(tbb::global_control::max_allowed_parallelism, nThreads);
        expect_identical(ref, segment_planes(*pc));
    }
#endif
}

// ---------------------------------------------------------------------------
void test_filter()
{
    // A cloud with an extra field: built as a plain one, then copied.
    SceneBuilder sb(8U);
    addRoom(sb, 1500.0, 0.005);
    sb.blob({0.5, 0.0, 0.0}, 0.3, 5000);
    const auto src = sb.done();

    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float("intensity");
    pc->resize(src->size());
    auto* intensity = pc->getPointsBufferRef_float_field("intensity");
    ASSERT_EQUAL_(intensity->size(), src->size());
    for (std::size_t i = 0; i < src->size(); i++)
    {
        float x = 0;
        float y = 0;
        float z = 0;
        src->getPoint(i, x, y, z);
        pc->setPoint(i, x, y, z);
        (*intensity)[i] = static_cast<float>(i % 100);
    }
    pc->mark_as_modified();
    const std::size_t N = pc->size();

    // Output layers:
    {
        mp2p_icp::metric_map_t m;
        m.layers["raw"] = pc;
        mp2p_icp_filters::FilterPlaneSegmentation f;
        f.initialize(mrpt::containers::yaml::FromText(R"(
          input_pointcloud_layer: 'raw'
          output_layer_planes: 'planes'
          output_layer_others: 'others'
          plane_id_field: 'plane_id'
          voxel_size: 0.05*2
          min_area: 1.0
          viewpoint: [0.0, 0.0, 0.0]
        )"));
        f.filter(m);

        ASSERT_EQUAL_(m.planes.size(), 6UL);
        const auto planes = m.point_layer("planes");
        const auto others = m.point_layer("others");
        ASSERT_EQUAL_(planes->size() + others->size(), N);
        ASSERT_GT_(others->size(), 4000UL);  // the blob
        const auto* ids = planes->getPointsBufferRef_uint16_field("plane_id");
        ASSERT_(ids != nullptr);
        ASSERT_EQUAL_(ids->size(), planes->size());
        for (const auto id : *ids)
        {
            ASSERT_GE_(id, 1);
            ASSERT_LE_(id, 6);
        }
        // Fields are carried over:
        ASSERT_(planes->getPointsBufferRef_float_field("intensity") != nullptr);
        ASSERT_(!pc->hasPointField("plane_id"));
        // Plane sizes match the patches:
        std::vector<std::size_t> count(7, 0);
        for (const auto id : *ids)
        {
            count[id]++;
        }
        for (std::size_t k = 0; k < 6; k++)
        {
            ASSERT_EQUAL_(count[k + 1], static_cast<std::size_t>(m.planes[k].num_points));
            ASSERT_GT_(m.planes[k].area, 5.0);
        }
    }

    // In place:
    {
        mp2p_icp::metric_map_t m;
        m.layers["raw"] = pc;
        mp2p_icp_filters::FilterPlaneSegmentation f;
        f.initialize(mrpt::containers::yaml::FromText(R"(
          input_pointcloud_layer: 'raw'
          plane_id_field: 'plane_id'
          store_planes: false
        )"));
        f.filter(m);
        ASSERT_(m.planes.empty());
        const auto* ids = pc->getPointsBufferRef_uint16_field("plane_id");
        ASSERT_(ids != nullptr);
        ASSERT_EQUAL_(ids->size(), N);
        const auto nOnPlanes =
            std::count_if(ids->begin(), ids->end(), [](uint16_t v) { return v != 0; });
        ASSERT_GT_(static_cast<double>(nOnPlanes), 0.9 * static_cast<double>(N));
    }

    // In place, on a layer that cannot hold the field:
    {
        mp2p_icp::metric_map_t m;
        SceneBuilder           sbPlain(9U);
        addRoom(sbPlain, 500.0, 0.005);
        m.layers["raw"] = sbPlain.done();
        mp2p_icp_filters::FilterPlaneSegmentation f;
        f.initialize(mrpt::containers::yaml::FromText(R"(
          input_pointcloud_layer: 'raw'
          plane_id_field: 'plane_id'
        )"));
        bool threw = false;
        try
        {
            f.filter(m);
        }
        catch (const std::exception&)
        {
            threw = true;
        }
        ASSERT_(threw);
    }
}

void test_filter_rejects_bad_outputs_without_changes()
{
    // Bad output configurations must throw before the map is changed.
    SceneBuilder sb(10U);
    addRoom(sb, 500.0, 0.005);
    const auto pc = sb.done();

    const auto tryFilter = [&](const std::string& yamlText, mp2p_icp::metric_map_t& m)
    {
        mp2p_icp_filters::FilterPlaneSegmentation f;
        f.initialize(mrpt::containers::yaml::FromText(yamlText));
        try
        {
            f.filter(m);
        }
        catch (const std::exception&)
        {
            return true;
        }
        return false;
    };

    const auto makeMap = [&]()
    {
        mp2p_icp::metric_map_t m;
        m.layers["raw"]    = pc;
        m.layers["voxels"] = mrpt::maps::CSimplePointsMap::Create();
        m.planes.emplace_back();  // a previous patch that must survive
        return m;
    };

    {
        auto m = makeMap();
        ASSERT_(tryFilter(
            R"(
          input_pointcloud_layer: 'raw'
          output_layer_planes: 'split'
          output_layer_others: 'split'
        )",
            m));
        ASSERT_EQUAL_(m.planes.size(), 1UL);
        ASSERT_EQUAL_(m.layers.size(), 2UL);
    }
    {
        auto m = makeMap();
        ASSERT_(tryFilter(
            R"(
          input_pointcloud_layer: 'raw'
          output_layer_planes: 'raw'
        )",
            m));
        ASSERT_EQUAL_(m.planes.size(), 1UL);
    }
    {
        // An existing planes layer that cannot hold the plane id field:
        auto m = makeMap();
        ASSERT_(tryFilter(
            R"(
          input_pointcloud_layer: 'raw'
          output_layer_planes: 'voxels'
          output_layer_others: 'others'
          plane_id_field: 'plane_id'
        )",
            m));
        ASSERT_EQUAL_(m.planes.size(), 1UL);
        ASSERT_EQUAL_(m.layers.count("others"), 0UL);
        ASSERT_EQUAL_(m.point_layer("voxels")->size(), 0UL);
    }
    {
        // An existing output layer with a field of another type, and a field
        // that it lacks: it must not be changed.
        auto input = mrpt::maps::CGenericPointsMap::Create();
        input->registerField_float("intensity");
        input->registerField_uint8("label");
        input->resize(pc->size());
        for (std::size_t i = 0; i < pc->size(); i++)
        {
            float x = 0;
            float y = 0;
            float z = 0;
            pc->getPoint(i, x, y, z);
            input->setPoint(i, x, y, z);
        }
        input->mark_as_modified();

        auto existing = mrpt::maps::CGenericPointsMap::Create();
        existing->registerField_double("intensity");

        mp2p_icp::metric_map_t m;
        m.layers["raw"]    = input;
        m.layers["planes"] = existing;
        m.planes.emplace_back();
        ASSERT_(tryFilter(
            R"(
          input_pointcloud_layer: 'raw'
          output_layer_planes: 'planes'
          plane_id_field: 'plane_id'
        )",
            m));
        ASSERT_EQUAL_(m.planes.size(), 1UL);
        ASSERT_(!existing->hasPointField("label"));
        ASSERT_(!existing->hasPointField("plane_id"));
        ASSERT_(!input->hasPointField("plane_id"));
    }
}

void test_empty_and_tiny()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    ASSERT_(segment_planes(*pc).planes.empty());
    for (int i = 0; i < 10; i++)
    {
        pc->insertPointFast(static_cast<float>(i), 0, 0);
    }
    pc->insertPointFast(std::nanf(""), 0, 0);
    pc->mark_as_modified();
    const auto r = segment_planes(*pc);
    ASSERT_(r.planes.empty());
    ASSERT_EQUAL_(r.num_input_points, 10UL);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_empty_and_tiny();
        test_box_room();
        test_noisy_tilted_ground();
        test_parallel_walls_stay_apart();
        test_curved_surface_is_split_or_rejected();
        test_sparse_multiring_lidar();
        test_determinism();
        test_filter();
        test_filter_rejects_bad_outputs_without_changes();
        time_city_block(1'000'000);
        if (mrpt::get_env<bool>("MP2P_PLANE_SEGMENTATION_BENCHMARK", false))
        {
            time_city_block(10'000'000);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
