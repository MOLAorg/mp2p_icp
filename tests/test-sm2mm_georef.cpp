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
 * @file   test-sm2mm_georef.cpp
 * @brief  Unit tests for sm2mm() with ENU georeferencing
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 *
 * Design of the key test:
 * ========================
 * The simplemap contains one keyframe whose robot pose is the identity.
 * The observation is a flat horizontal grid of points at z=0 **in the local
 * (map) frame**.  However, the map frame itself is tilted with respect to ENU:
 * we choose a T_enu_to_map with a significant pitch and roll so that, when the
 * same points are seen in the ENU frame, they still lie at z≈0 in ENU (because
 * T_enu_to_map compensates the tilt).
 *
 * A FilterBoundingBox with a thin z-slab [-0.1, 0.1] is applied.
 * Without the ENU transform the tilt causes points to span a large z range and
 * most of them fall outside the slab → few/no points inside.
 * With the ENU transform all points are at z≈0 → all points land inside.
 *
 * Concretely:
 *  - Grid: N×N points in the XY plane, x,y ∈ [-(N-1)/2, (N-1)/2], z=0,
 *    expressed in the map (local) frame.
 *  - T_enu_to_map = CPose3D(0, 0, 0,  yaw=0, pitch=40°, roll=30°)
 *    (the map frame is pitched and rolled by these amounts w.r.t. ENU).
 *  - The effective robot pose in ENU is:
 *      p_enu = T_enu_to_map (+) robotPose   (robotPose = Identity here)
 *    so the generator receives the ENU pose, and the observation points
 *    (expressed in sensor frame = robot frame = map frame here) get
 *    transformed to ENU world coordinates, ending up at z≈0.
 *  - Without georef the generator uses robotPose = Identity, so points remain
 *    at their local-frame positions where z=0 but tilted in world → they
 *    appear with varying z after global insertion.
 *
 * Wait - the generator transforms points using robotPose and the sensor pose.
 * Points are stored in the sensor frame (z=0 plane). With robotPose = Identity
 * they are inserted as-is (z=0), so the BoundingBox filter would pass all of
 * them even WITHOUT georef, which would make the test trivial and incorrect.
 *
 * To make the test meaningful we therefore set robotPose ≠ Identity:
 *  - robotPose  = T_enu_to_map^{-1}  (the robot is at the tilted pose).
 *  - Points in sensor frame are a flat z=0 grid.
 *  - Without georef: inserted pose = robotPose (tilted) → the flat grid is
 *    tilted in world → points span a large z range → most fail the z-slab.
 *  - With georef: effective pose = T_enu_to_map (+) robotPose
 *                                = T_enu_to_map (+) T_enu_to_map^{-1}
 *                                = Identity
 *    → points inserted at Identity → flat grid at z=0 → all pass the z-slab.
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mp2p_icp_filters/Generator.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/topography/data_types.h>

#include <cmath>
#include <iostream>

using namespace mp2p_icp;
using namespace mp2p_icp_filters;

namespace
{

// ---------------------------------------------------------------------------
// Constants shared across tests
// ---------------------------------------------------------------------------

// Grid size: GRID_N × GRID_N points at unit spacing, centred at the origin.
constexpr int   GRID_N  = 9;  // odd → symmetric, centre at (0,0,0)
constexpr float SPACING = 1.0f;

// Significant tilt angles (radians) for T_enu_to_map.
// Both pitch and roll are large enough that the untilted test unambiguously
// fails the z-slab filter.
const double PITCH_DEG = 40.0;
const double ROLL_DEG  = 30.0;
const double PITCH_RAD = PITCH_DEG * M_PI / 180.0;
const double ROLL_RAD  = ROLL_DEG * M_PI / 180.0;

// BoundingBox z-slab half-thickness.  Points in the ENU frame lie at z=0;
// with numerical noise a generous slab is safe.
constexpr float Z_SLAB = 0.10f;

// XY extent of the grid (used for the BoundingBox XY bounds).
// Grid goes from -(GRID_N-1)/2*SPACING to +(GRID_N-1)/2*SPACING.
const float HALF_GRID = static_cast<float>((GRID_N - 1) / 2) * SPACING + 0.5f * SPACING;  // NOLINT

// ---------------------------------------------------------------------------
// Build a flat z=0 grid observation in the robot/sensor frame
// ---------------------------------------------------------------------------
mrpt::obs::CObservationPointCloud::Ptr buildFlatGridObs()
{
    auto obs = mrpt::obs::CObservationPointCloud::Create();
    auto pc  = mrpt::maps::CSimplePointsMap::Create();

    const float half = static_cast<float>((GRID_N - 1)) * SPACING * 0.5f;
    for (int i = 0; i < GRID_N; ++i)
    {
        for (int j = 0; j < GRID_N; ++j)
        {
            float x = -half + static_cast<float>(i) * SPACING;
            float y = -half + static_cast<float>(j) * SPACING;
            pc->insertPointFast(x, y, 0.0f);
        }
    }
    obs->pointcloud  = pc;
    obs->sensorLabel = "lidar";
    // sensorPose = Identity (sensor is at robot origin)
    return obs;
}

// ---------------------------------------------------------------------------
// Build a CSimpleMap with one keyframe
//   robotPose: the pose of the robot in the map frame
//   obs: the observation attached to that keyframe
// ---------------------------------------------------------------------------
mrpt::maps::CSimpleMap buildSimpleMap(
    const mrpt::poses::CPose3D& robotPose, mrpt::obs::CObservationPointCloud::Ptr obs)
{
    mrpt::maps::CSimpleMap sm;

    auto posePDF  = mrpt::poses::CPose3DPDFGaussian::Create();
    posePDF->mean = robotPose;

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);

    sm.insert(posePDF, sf);
    return sm;
}

// ---------------------------------------------------------------------------
// Build a YAML pipeline with a single BoundingBox filter:
//   - input layer  : "raw"
//   - inside layer : "inside"   (z in [-Z_SLAB, Z_SLAB], XY wide open)
//   - outside layer: "outside"
// ---------------------------------------------------------------------------
mrpt::containers::yaml buildBBoxPipeline()
{
    // We use a generator (default) + one BoundingBox filter.
    // The generator will create a "raw" layer from the observation.
    // The filter then splits it.
    const std::string yaml_text = std::string(R"(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: raw

filters:
  - class_name: mp2p_icp_filters::FilterBoundingBox
    params:
      input_pointcloud_layer:   raw
      inside_pointcloud_layer:  inside
      outside_pointcloud_layer: outside
      bounding_box_min: [-)") + std::to_string(HALF_GRID) +
                                  ", -" + std::to_string(HALF_GRID) + ", -" +
                                  std::to_string(Z_SLAB) + R"(]
      bounding_box_max: [)" + std::to_string(HALF_GRID) +
                                  ", " + std::to_string(HALF_GRID) + ", " + std::to_string(Z_SLAB) +
                                  R"(]
)";

    return mrpt::containers::yaml::FromText(yaml_text);
}

// ---------------------------------------------------------------------------
// Build a fake georeferencing struct.
//   T_enu_to_map: significant pitch + roll (map frame is tilted w.r.t. ENU).
//   geo_coord: arbitrary geodetic coordinates near Almería, Spain.
// ---------------------------------------------------------------------------
metric_map_t::Georeferencing buildGeoref()
{
    metric_map_t::Georeferencing g;

    // Fake geodetic reference point (Almería, Spain area)
    g.geo_coord.lat    = mrpt::DEG2RAD(36.8340);  // degrees N
    g.geo_coord.lon    = mrpt::DEG2RAD(-2.4637);  // degrees W
    g.geo_coord.height = 42.0;  // metres above ellipsoid

    // T_enu_to_map: the local map frame has pitch=PITCH_RAD, roll=ROLL_RAD
    // relative to ENU (yaw=0 for simplicity).
    g.T_enu_to_map.mean = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, PITCH_RAD, ROLL_RAD);
    // Zero covariance (no uncertainty on the reference).
    g.T_enu_to_map.cov.setZero();

    return g;
}

// ===========================================================================
// Test 1 - WITHOUT georeferencing
//
// Robot pose = T_enu_to_map^{-1} (tilted).  Points are a flat z=0 grid in
// sensor frame.  After insertion the grid is tilted in world coordinates,
// so the z values span a large range and most points fall outside the z-slab.
// ===========================================================================
void test_sm2mm_NoGeoref_TiltedPoseFails()
{
    std::cout << "Testing sm2mm WITHOUT georef (tilted pose → z-slab rejects most points)... ";

    const auto T_enu_to_map = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, PITCH_RAD, ROLL_RAD);
    // Robot pose is the *inverse* of T_enu_to_map (= T_map_to_enu)
    const mrpt::poses::CPose3D robotPose = mrpt::poses::CPose3D() - T_enu_to_map;

    auto sm = buildSimpleMap(robotPose, buildFlatGridObs());

    sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;
    // No georeferencing set → uses robotPose as-is.

    metric_map_t mm;
    simplemap_to_metricmap(sm, mm, buildBBoxPipeline(), opts);

    ASSERTMSG_(mm.layers.count("inside"), "Expected 'inside' layer in output map");
    ASSERTMSG_(mm.layers.count("outside"), "Expected 'outside' layer in output map");

    auto* inside_pc  = MapToPointsMap(*mm.layers.at("inside"));
    auto* outside_pc = MapToPointsMap(*mm.layers.at("outside"));
    ASSERTMSG_(inside_pc, "inside layer is not a point map");
    ASSERTMSG_(outside_pc, "outside layer is not a point map");

    const size_t total = inside_pc->size() + outside_pc->size();
    ASSERT_EQUAL_(total, static_cast<size_t>(GRID_N * GRID_N));

    // With the large tilt most points MUST be outside the z-slab.
    // We require that fewer than half passed (conservative threshold).
    const size_t n_inside = inside_pc->size();
    ASSERT_LT_(n_inside, static_cast<size_t>(GRID_N * GRID_N) / 2);

    std::cout << "Success ✅ (" << n_inside << "/" << (GRID_N * GRID_N)
              << " inside z-slab without georef)\n";
}

// ===========================================================================
// Test 2 - WITH georeferencing
//
// Same setup, but now opts.georeferencing = buildGeoref().
// Effective pose = T_enu_to_map (+) robotPose
//               = T_enu_to_map (+) T_enu_to_map^{-1}
//               = Identity
// Points in sensor frame (z=0) are inserted at Identity → remain at z=0 in
// world (ENU) → ALL points land inside the z-slab.
// ===========================================================================
void test_sm2mm_WithGeoref_ENUPosePasses()
{
    std::cout << "Testing sm2mm WITH georef (ENU pose → all points inside z-slab)... ";

    const auto T_enu_to_map = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, PITCH_RAD, ROLL_RAD);
    const mrpt::poses::CPose3D robotPose = mrpt::poses::CPose3D() - T_enu_to_map;

    auto sm = buildSimpleMap(robotPose, buildFlatGridObs());

    sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;
    opts.georeferencing  = buildGeoref();  // ← ENU transform enabled

    metric_map_t mm;
    simplemap_to_metricmap(sm, mm, buildBBoxPipeline(), opts);

    ASSERTMSG_(mm.layers.count("inside"), "Expected 'inside' layer in output map");
    ASSERTMSG_(mm.layers.count("outside"), "Expected 'outside' layer in output map");

    auto* inside_pc  = MapToPointsMap(*mm.layers.at("inside"));
    auto* outside_pc = MapToPointsMap(*mm.layers.at("outside"));
    ASSERTMSG_(inside_pc, "inside layer is not a point map");
    ASSERTMSG_(outside_pc, "outside layer is not a point map");

    const size_t total = inside_pc->size() + outside_pc->size();
    ASSERT_EQUAL_(total, static_cast<size_t>(GRID_N * GRID_N));

    // ALL points must be inside the z-slab (effective pose = Identity).
    ASSERT_EQUAL_(inside_pc->size(), static_cast<size_t>(GRID_N * GRID_N));
    ASSERT_EQUAL_(outside_pc->size(), 0u);

    std::cout << "Success ✅ (all " << inside_pc->size() << "/" << (GRID_N * GRID_N)
              << " inside z-slab with georef)\n";
}

// ===========================================================================
// Test 3 - Output map carries georeferencing metadata with identity T_enu_to_map
//
// Verifies that after simplemap_to_metricmap() with georeferencing:
//   1. mm.georeferencing is populated.
//   2. geo_coord matches the input verbatim.
//   3. T_enu_to_map.mean is the identity pose (within floating-point tolerance).
//   4. T_enu_to_map.cov is preserved from the input (zero in our case).
// ===========================================================================
void test_sm2mm_WithGeoref_OutputMetadata()
{
    std::cout << "Testing sm2mm WITH georef (output map metadata is correct)... ";

    const auto T_enu_to_map = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, PITCH_RAD, ROLL_RAD);
    const mrpt::poses::CPose3D robotPose = mrpt::poses::CPose3D() - T_enu_to_map;

    auto sm = buildSimpleMap(robotPose, buildFlatGridObs());

    const auto inputGeoref = buildGeoref();

    sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;
    opts.georeferencing  = inputGeoref;

    metric_map_t mm;
    simplemap_to_metricmap(sm, mm, buildBBoxPipeline(), opts);

    // 1. Georeferencing must be present in the output map.
    ASSERTMSG_(mm.georeferencing.has_value(), "Output map has no georeferencing metadata");

    const auto& outG = *mm.georeferencing;

    // 2. Geodetic reference must be copied verbatim.
    ASSERT_NEAR_(outG.geo_coord.lat, inputGeoref.geo_coord.lat, 1e-10);
    ASSERT_NEAR_(outG.geo_coord.lon, inputGeoref.geo_coord.lon, 1e-10);
    ASSERT_NEAR_(outG.geo_coord.height, inputGeoref.geo_coord.height, 1e-6);

    // 3. T_enu_to_map mean must be the identity (map is now in ENU).
    const auto&  T_out = outG.T_enu_to_map.mean;
    const double tol   = 1e-9;
    ASSERT_NEAR_(T_out.x(), 0.0, tol);
    ASSERT_NEAR_(T_out.y(), 0.0, tol);
    ASSERT_NEAR_(T_out.z(), 0.0, tol);
    ASSERT_NEAR_(T_out.yaw(), 0.0, tol);
    ASSERT_NEAR_(T_out.pitch(), 0.0, tol);
    ASSERT_NEAR_(T_out.roll(), 0.0, tol);

    // 4. Covariance must be preserved (zero in our test case).
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            ASSERT_NEAR_(outG.T_enu_to_map.cov(r, c), inputGeoref.T_enu_to_map.cov(r, c), 1e-15);
        }
    }

    std::cout << "Success ✅\n";
}

// ===========================================================================
// Test 4 - Without georeferencing, output map has NO georeferencing metadata
// ===========================================================================
void test_sm2mm_NoGeoref_NoOutputMetadata()
{
    std::cout << "Testing sm2mm WITHOUT georef (output map must have no georef metadata)... ";

    const auto robotPose = mrpt::poses::CPose3D();  // identity
    auto       sm        = buildSimpleMap(robotPose, buildFlatGridObs());

    sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;
    // opts.georeferencing intentionally not set

    metric_map_t mm;
    simplemap_to_metricmap(sm, mm, buildBBoxPipeline(), opts);

    ASSERTMSG_(
        !mm.georeferencing.has_value(),
        "Output map must NOT have georeferencing when none was supplied");

    std::cout << "Success ✅\n";
}

}  // namespace

// ---------------------------------------------------------------------------
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        std::vector<std::function<void()>> tests = {
            test_sm2mm_NoGeoref_TiltedPoseFails,
            test_sm2mm_WithGeoref_ENUPosePasses,
            test_sm2mm_WithGeoref_OutputMetadata,
            test_sm2mm_NoGeoref_NoOutputMetadata,
        };

        int failures = 0;
        for (const auto& test : tests)
        {
            try
            {
                test();
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error: ❌\n" << e.what() << std::endl;
                failures++;
            }
        }

        if (failures == 0)
        {
            std::cout << "\n✅ All tests passed!" << std::endl;
        }
        else
        {
            std::cout << "\n❌ " << failures << " test(s) failed!" << std::endl;
        }

        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error:\n" << e.what() << std::endl;
        return 1;
    }
}
