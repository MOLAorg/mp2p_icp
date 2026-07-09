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
 * @file   test-georef-yaml.cpp
 * @brief  Unit tests for mp2p_icp::ToYAML() / FromYAML() georeferencing
 *         serialization.
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 *
 * Coverage
 * --------
 *  1.  std::nullopt → ToYAML → FromYAML → std::nullopt          (null round-trip)
 *  2.  Identity pose + zero cov → round-trip                     (trivial pose)
 *  3.  Non-trivial pose (yaw/pitch/roll + translation) → round-trip
 *  4.  Non-trivial pose + non-zero covariance → round-trip       (full round-trip)
 *  5.  Geodetic coordinates are preserved with full precision     (lat/lon/alt)
 *  6.  Degree ↔ radian conversion is exact for known angles      (deg/rad fidelity)
 *  7.  ToYAML() emits the mandatory structural keys              (schema check)
 *  8.  ToYAML(nullopt) emits  defined=false, no geo_coord key   (null schema)
 *  9.  FromYAML() throws on missing "type" key                   (error path)
 * 10.  FromYAML() throws on wrong "type" value (bad magic)       (error path)
 * 11.  FromYAML() throws on missing "defined" key                (error path)
 * 12.  FromYAML() throws when the whole input is not a Map       (error path)
 * 13.  Covariance diagonal round-trip fidelity                   (cov diagonal)
 * 14.  Covariance off-diagonal round-trip fidelity               (cov full)
 * 15.  ToYAML → print as text → re-parse → FromYAML             (text serialization)
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/topography/data_types.h>

#include <cmath>
#include <iostream>
#include <sstream>

using namespace mp2p_icp;

namespace
{

// ---------------------------------------------------------------------------
// Tolerance constants
// ---------------------------------------------------------------------------
constexpr double TOL_POSE  = 1e-9;  // metres / radians
constexpr double TOL_DEG   = 1e-9;  // degrees round-tripped through deg↔rad
constexpr double TOL_COORD = 1e-12;  // geodetic decimal degrees / metres
constexpr double TOL_COV   = 1e-12;  // covariance matrix entries

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Build a Georeferencing with fully non-trivial values. */
metric_map_t::Georeferencing makeNonTrivialGeoref()
{
    metric_map_t::Georeferencing g;

    // Geodetic reference near Almería, Spain
    g.geo_coord.lat.decimal_value = 36.834012345678;
    g.geo_coord.lon.decimal_value = -2.463712345678;
    g.geo_coord.height            = 42.125;

    // Non-trivial SE(3) pose
    g.T_enu_to_map.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.5, -2.3, 0.7, mrpt::DEG2RAD(45.0), mrpt::DEG2RAD(10.0), mrpt::DEG2RAD(-20.0));

    // Non-zero, non-diagonal covariance
    g.T_enu_to_map.cov.setZero();
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            // Use a recognisable pattern: (r+1)*(c+1)*0.001, symmetric
            g.T_enu_to_map.cov(r, c) = static_cast<double>((r + 1) * (c + 1)) * 0.001;
        }
    }

    return g;
}

/** Assert two CPose3D objects are equal within TOL_POSE. */
void assertPosesEqual(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b, const char* ctx)
{
    ASSERT_NEAR_(a.x(), b.x(), TOL_POSE);
    ASSERT_NEAR_(a.y(), b.y(), TOL_POSE);
    ASSERT_NEAR_(a.z(), b.z(), TOL_POSE);
    ASSERT_NEAR_(a.yaw(), b.yaw(), TOL_POSE);
    ASSERT_NEAR_(a.pitch(), b.pitch(), TOL_POSE);
    ASSERT_NEAR_(a.roll(), b.roll(), TOL_POSE);
    (void)ctx;
}

/** Assert two 6×6 covariance matrices are equal within TOL_COV. */
void assertCovEqual(
    const mrpt::math::CMatrixDouble66& a, const mrpt::math::CMatrixDouble66& b, const char* ctx)
{
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            ASSERT_NEAR_(a(r, c), b(r, c), TOL_COV);
        }
    }
    (void)ctx;
}

// ===========================================================================
// Test 1 - nullopt round-trip
// ===========================================================================
void test_nullopt_roundtrip()
{
    std::cout << "Test 1 - nullopt → ToYAML → FromYAML → nullopt ... ";

    const std::optional<metric_map_t::Georeferencing> input = std::nullopt;
    const auto                                        yaml  = ToYAML(input);
    const auto                                        out   = FromYAML(yaml);

    ASSERTMSG_(!out.has_value(), "Expected nullopt back from a null georef round-trip");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 2 - identity pose + zero covariance round-trip
// ===========================================================================
void test_identity_pose_roundtrip()
{
    std::cout << "Test 2 - identity pose + zero cov round-trip ... ";

    metric_map_t::Georeferencing g;
    g.geo_coord.lat.decimal_value = 0.0;
    g.geo_coord.lon.decimal_value = 0.0;
    g.geo_coord.height            = 0.0;
    g.T_enu_to_map.mean           = mrpt::poses::CPose3D::Identity();
    g.T_enu_to_map.cov.setZero();

    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected a value back from identity round-trip");

    assertPosesEqual(g.T_enu_to_map.mean, out->T_enu_to_map.mean, "identity pose");
    assertCovEqual(g.T_enu_to_map.cov, out->T_enu_to_map.cov, "identity cov");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 3 - non-trivial pose, zero covariance round-trip
// ===========================================================================
void test_nontrivial_pose_roundtrip()
{
    std::cout << "Test 3 - non-trivial pose (no cov) round-trip ... ";

    metric_map_t::Georeferencing g;
    g.geo_coord.lat.decimal_value = 36.834;
    g.geo_coord.lon.decimal_value = -2.463;
    g.geo_coord.height            = 42.0;

    g.T_enu_to_map.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        3.14, -1.59, 2.65, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(-15.0), mrpt::DEG2RAD(5.0));
    g.T_enu_to_map.cov.setZero();

    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected a value from non-trivial pose round-trip");
    assertPosesEqual(g.T_enu_to_map.mean, out->T_enu_to_map.mean, "non-trivial pose");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 4 - full round-trip: non-trivial pose + non-zero covariance
// ===========================================================================
void test_full_roundtrip()
{
    std::cout << "Test 4 - full round-trip (pose + cov) ... ";

    const auto g    = makeNonTrivialGeoref();
    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected a value from full round-trip");
    assertPosesEqual(g.T_enu_to_map.mean, out->T_enu_to_map.mean, "full pose");
    assertCovEqual(g.T_enu_to_map.cov, out->T_enu_to_map.cov, "full cov");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 5 - geodetic coordinate precision
// ===========================================================================
void test_geodetic_precision()
{
    std::cout << "Test 5 - geodetic coordinate precision ... ";

    metric_map_t::Georeferencing g;
    // Use values with many significant digits
    g.geo_coord.lat.decimal_value = 36.834012345678901;
    g.geo_coord.lon.decimal_value = -2.463712345678901;
    g.geo_coord.height            = 12345.6789012345;
    g.T_enu_to_map.mean           = mrpt::poses::CPose3D::Identity();
    g.T_enu_to_map.cov.setZero();

    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected value from geodetic precision test");

    ASSERT_NEAR_(out->geo_coord.lat.decimal_value, g.geo_coord.lat.decimal_value, TOL_COORD);
    ASSERT_NEAR_(out->geo_coord.lon.decimal_value, g.geo_coord.lon.decimal_value, TOL_COORD);
    ASSERT_NEAR_(out->geo_coord.height, g.geo_coord.height, TOL_COORD);

    std::cout << "✅\n";
}

// ===========================================================================
// Test 6 - degree ↔ radian fidelity for known angles
//
// ToYAML stores angles in degrees; FromYAML converts back to radians.
// Verify that for angles that are exact in degrees (multiples of common
// fractions) the round-trip error is negligible.
// ===========================================================================
void test_degree_radian_fidelity()
{
    std::cout << "Test 6 - degree ↔ radian fidelity ... ";

    // Angles chosen to be exact in degrees but irrational in radians
    const double yaw_deg   = 90.0;
    const double pitch_deg = -45.0;
    const double roll_deg  = 30.0;

    metric_map_t::Georeferencing g;
    g.geo_coord.lat.decimal_value = 0.0;
    g.geo_coord.lon.decimal_value = 0.0;
    g.geo_coord.height            = 0.0;
    g.T_enu_to_map.mean           = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                  0, 0, 0, mrpt::DEG2RAD(yaw_deg), mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));
    g.T_enu_to_map.cov.setZero();

    const auto yaml = ToYAML(g);

    // Verify the stored YAML values are in degrees (human-readable check)
    const double stored_yaw_deg   = yaml["T_enu_to_map"]["mean"]["yaw_deg"].as<double>();
    const double stored_pitch_deg = yaml["T_enu_to_map"]["mean"]["pitch_deg"].as<double>();
    const double stored_roll_deg  = yaml["T_enu_to_map"]["mean"]["roll_deg"].as<double>();

    ASSERT_NEAR_(stored_yaw_deg, yaw_deg, TOL_DEG);
    ASSERT_NEAR_(stored_pitch_deg, pitch_deg, TOL_DEG);
    ASSERT_NEAR_(stored_roll_deg, roll_deg, TOL_DEG);

    // Round-trip: the recovered angles in radians must match the originals
    const auto out = FromYAML(yaml);
    ASSERTMSG_(out.has_value(), "Expected value from deg/rad fidelity test");

    ASSERT_NEAR_(out->T_enu_to_map.mean.yaw(), mrpt::DEG2RAD(yaw_deg), TOL_POSE);
    ASSERT_NEAR_(out->T_enu_to_map.mean.pitch(), mrpt::DEG2RAD(pitch_deg), TOL_POSE);
    ASSERT_NEAR_(out->T_enu_to_map.mean.roll(), mrpt::DEG2RAD(roll_deg), TOL_POSE);

    std::cout << "✅\n";
}

// ===========================================================================
// Test 7 - ToYAML emits the mandatory structural keys when defined
// ===========================================================================
void test_yaml_schema_defined()
{
    std::cout << "Test 7 - YAML schema keys present when defined=true ... ";

    const auto g    = makeNonTrivialGeoref();
    const auto yaml = ToYAML(g);

    ASSERTMSG_(yaml.isMap(), "Top-level must be a map");
    ASSERTMSG_(yaml.has("type"), "Missing key: type");
    ASSERTMSG_(yaml.has("defined"), "Missing key: defined");
    ASSERTMSG_(yaml.has("geo_coord"), "Missing key: geo_coord");
    ASSERTMSG_(yaml.has("T_enu_to_map"), "Missing key: T_enu_to_map");

    ASSERTMSG_(yaml["geo_coord"].has("lat"), "geo_coord missing lat");
    ASSERTMSG_(yaml["geo_coord"].has("lon"), "geo_coord missing lon");
    ASSERTMSG_(yaml["geo_coord"].has("altitude"), "geo_coord missing altitude");

    ASSERTMSG_(yaml["T_enu_to_map"].has("mean"), "T_enu_to_map missing mean");
    ASSERTMSG_(yaml["T_enu_to_map"].has("cov"), "T_enu_to_map missing cov");

    const auto& mean = yaml["T_enu_to_map"]["mean"];
    ASSERTMSG_(mean.has("x"), "mean missing x");
    ASSERTMSG_(mean.has("y"), "mean missing y");
    ASSERTMSG_(mean.has("z"), "mean missing z");
    ASSERTMSG_(mean.has("yaw_deg"), "mean missing yaw_deg");
    ASSERTMSG_(mean.has("pitch_deg"), "mean missing pitch_deg");
    ASSERTMSG_(mean.has("roll_deg"), "mean missing roll_deg");

    // type must be the magic string
    ASSERT_EQUAL_(yaml["type"].as<std::string>(), std::string("mp2p_icp::Georeferencing"));

    // defined must be true
    ASSERTMSG_(yaml["defined"].as<bool>(), "defined should be true");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 8 - ToYAML(nullopt) emits defined=false and no geo_coord
// ===========================================================================
void test_yaml_schema_undefined()
{
    std::cout << "Test 8 - YAML schema when defined=false (nullopt) ... ";

    const std::optional<metric_map_t::Georeferencing> input = std::nullopt;
    const auto                                        yaml  = ToYAML(input);

    ASSERTMSG_(yaml.isMap(), "Top-level must be a map");
    ASSERTMSG_(yaml.has("type"), "Missing key: type");
    ASSERTMSG_(yaml.has("defined"), "Missing key: defined");

    ASSERTMSG_(!yaml["defined"].as<bool>(), "defined should be false for nullopt");

    // geo_coord and T_enu_to_map must NOT be present for a null georef
    ASSERTMSG_(!yaml.has("geo_coord"), "geo_coord must be absent when defined=false");
    ASSERTMSG_(!yaml.has("T_enu_to_map"), "T_enu_to_map must be absent when defined=false");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 9 - FromYAML throws on missing "type" key
// ===========================================================================
void test_from_yaml_throws_missing_type()
{
    std::cout << "Test 9 - FromYAML throws on missing 'type' ... ";

    mrpt::containers::yaml yaml = mrpt::containers::yaml::Map();
    yaml["defined"]             = true;
    yaml["geo_coord"]           = mrpt::containers::yaml::Map();
    yaml["T_enu_to_map"]        = mrpt::containers::yaml::Map();

    bool threw = false;
    try
    {
        FromYAML(yaml);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERTMSG_(threw, "Expected an exception for missing 'type' key");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 10 - FromYAML throws on wrong magic string in "type"
// ===========================================================================
void test_from_yaml_throws_wrong_magic()
{
    std::cout << "Test 10 - FromYAML throws on wrong magic string ... ";

    // Build a structurally valid YAML but with the wrong type string
    const auto g    = makeNonTrivialGeoref();
    auto       yaml = ToYAML(g);
    yaml["type"]    = "some::wrong::Type";  // corrupt the magic string

    bool threw = false;
    try
    {
        FromYAML(yaml);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERTMSG_(threw, "Expected an exception for wrong magic string in 'type'");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 11 - FromYAML throws on missing "defined" key
// ===========================================================================
void test_from_yaml_throws_missing_defined()
{
    std::cout << "Test 11 - FromYAML throws on missing 'defined' ... ";

    mrpt::containers::yaml yaml = mrpt::containers::yaml::Map();

    yaml["type"] = "mp2p_icp::Georeferencing";
    // "defined" intentionally omitted

    bool threw = false;
    try
    {
        FromYAML(yaml);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERTMSG_(threw, "Expected an exception for missing 'defined' key");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 12 - FromYAML throws when the top-level node is not a Map
// ===========================================================================
void test_from_yaml_throws_not_a_map()
{
    std::cout << "Test 12 - FromYAML throws when input is not a map ... ";

    // A scalar node, not a map
    const auto yaml = mrpt::containers::yaml(42);

    bool threw = false;
    try
    {
        FromYAML(yaml);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERTMSG_(threw, "Expected an exception when top-level node is not a map");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 13 - covariance diagonal round-trip
// ===========================================================================
void test_cov_diagonal_roundtrip()
{
    std::cout << "Test 13 - covariance diagonal round-trip ... ";

    metric_map_t::Georeferencing g;
    g.geo_coord.lat.decimal_value = 36.0;
    g.geo_coord.lon.decimal_value = -2.0;
    g.geo_coord.height            = 0.0;
    g.T_enu_to_map.mean           = mrpt::poses::CPose3D::Identity();
    g.T_enu_to_map.cov.setZero();

    // Distinct diagonal values
    for (int i = 0; i < 6; ++i)
    {
        g.T_enu_to_map.cov(i, i) = static_cast<double>(i + 1) * 0.1;
    }

    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected value from diagonal cov test");

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_NEAR_(out->T_enu_to_map.cov(i, i), g.T_enu_to_map.cov(i, i), TOL_COV);
    }
    // Off-diagonal must remain zero
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            if (r != c)
            {
                ASSERT_NEAR_(out->T_enu_to_map.cov(r, c), 0.0, TOL_COV);
            }
        }
    }

    std::cout << "✅\n";
}

// ===========================================================================
// Test 14 - covariance full (off-diagonal) round-trip
// ===========================================================================
void test_cov_full_roundtrip()
{
    std::cout << "Test 14 - covariance full off-diagonal round-trip ... ";

    const auto g    = makeNonTrivialGeoref();  // has non-zero off-diagonal cov
    const auto yaml = ToYAML(g);
    const auto out  = FromYAML(yaml);

    ASSERTMSG_(out.has_value(), "Expected value from full cov test");
    assertCovEqual(g.T_enu_to_map.cov, out->T_enu_to_map.cov, "off-diagonal cov");

    std::cout << "✅\n";
}

// ===========================================================================
// Test 15 - ToYAML → serialise to text → re-parse → FromYAML
//
// This mirrors the actual on-disk workflow used by mola-sm-georeferencing and
// mm-georef: the YAML object is printed to a text stream and then re-loaded
// via mrpt::containers::yaml::FromText / FromStream.
// ===========================================================================
void test_text_serialization_roundtrip()
{
    std::cout << "Test 15 - text serialization round-trip (print → parse) ... ";

    const auto g    = makeNonTrivialGeoref();
    const auto yaml = ToYAML(g);

    // Print to string stream (as done when writing *.yaml files)
    std::ostringstream oss;
    yaml.printAsYAML(oss);
    const std::string yaml_text = oss.str();

    ASSERTMSG_(!yaml_text.empty(), "Serialized YAML text must not be empty");

    // Re-parse from the text
    std::istringstream     iss(yaml_text);
    mrpt::containers::yaml reparsed;
    reparsed.loadFromStream(iss);

    const auto out = FromYAML(reparsed);

    ASSERTMSG_(out.has_value(), "Expected value after text round-trip");

    assertPosesEqual(g.T_enu_to_map.mean, out->T_enu_to_map.mean, "text pose");
    assertCovEqual(g.T_enu_to_map.cov, out->T_enu_to_map.cov, "text cov");

    ASSERT_NEAR_(out->geo_coord.lat.decimal_value, g.geo_coord.lat.decimal_value, TOL_COORD);
    ASSERT_NEAR_(out->geo_coord.lon.decimal_value, g.geo_coord.lon.decimal_value, TOL_COORD);
    ASSERT_NEAR_(out->geo_coord.height, g.geo_coord.height, TOL_COORD);

    std::cout << "✅\n";
}

}  // namespace

// ---------------------------------------------------------------------------
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        std::vector<std::function<void()>> tests = {
            test_nullopt_roundtrip,
            test_identity_pose_roundtrip,
            test_nontrivial_pose_roundtrip,
            test_full_roundtrip,
            test_geodetic_precision,
            test_degree_radian_fidelity,
            test_yaml_schema_defined,
            test_yaml_schema_undefined,
            test_from_yaml_throws_missing_type,
            test_from_yaml_throws_wrong_magic,
            test_from_yaml_throws_missing_defined,
            test_from_yaml_throws_not_a_map,
            test_cov_diagonal_roundtrip,
            test_cov_full_roundtrip,
            test_text_serialization_roundtrip,
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
                std::cerr << "Error: ❌\n" << e.what() << "\n";
                failures++;
            }
        }

        if (failures == 0)
        {
            std::cout << "\n✅ All tests passed!\n";
        }
        else
        {
            std::cout << "\n❌ " << failures << " test(s) failed!\n";
        }

        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error:\n" << e.what() << "\n";
        return 1;
    }
}
