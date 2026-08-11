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
 * @file   test-mp2p_matcher_pt2pt_parameterizable.cpp
 * @brief  Unit tests for matcher with Parameterizable dynamic parameters
 * @author Jose Luis Blanco Claraco
 * @date   Dec 13, 2023
 */

#include <mp2p_icp/Matcher_Adaptive.h>
#include <mp2p_icp/Matcher_Point2Line.h>
#include <mp2p_icp/Matcher_Point2Plane.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>

#include <variant>

namespace
{
/** Reads back the live value of a declared dynamic parameter, identified by the
 *  original expression text it was loaded from. This works regardless of the
 *  visibility of the member variable holding it, and it fails if the parameter
 *  was not declared as a dynamic expression at all (i.e. if it was read with a
 *  static YAML load, which silently truncates formulas).
 */
double declaredParameterValue(const mp2p_icp::Parameterizable& obj, const std::string& expression)
{
    for (const auto& p : obj.declaredParameters())
    {
        if (p.expression != expression)
        {
            continue;
        }

        ASSERTMSG_(
            !p.is_constant,
            "Parameter '" + expression + "' was evaluated as a constant, not as a formula");

        if (std::holds_alternative<double*>(p.target))
        {
            return *std::get<double*>(p.target);
        }
        if (std::holds_alternative<float*>(p.target))
        {
            return static_cast<double>(*std::get<float*>(p.target));
        }
        THROW_EXCEPTION("Unexpected target type for a floating-point parameter");
    }

    THROW_EXCEPTION_FMT(
        "No dynamic parameter was declared from the expression '%s'. It was probably read with a "
        "static YAML load, which silently truncates formulas.",
        expression.c_str());
}

void test_points_distance_threshold()
{
    mp2p_icp::Matcher_Points_DistanceThreshold m;
    mrpt::containers::yaml                     p;
    p["threshold"]           = "MATCH_THRESHOLD*2.0";  // Define as an expr.
    p["thresholdAngularDeg"] = .0;

    m.initialize(p);

    mp2p_icp::ParameterSource globalParams;
    globalParams.attach(m);

    // v1:
    globalParams.updateVariable("MATCH_THRESHOLD", 1.5);
    globalParams.realize();

    ASSERT_NEAR_(m.threshold, 3.0, 1e-4);
    ASSERT_NEAR_(m.thresholdAngularDeg, .0, 1e-4);

    // v2:
    globalParams.updateVariable("MATCH_THRESHOLD", 0.5);
    globalParams.realize();

    ASSERT_NEAR_(m.threshold, 1.0, 1e-4);
    ASSERT_NEAR_(m.thresholdAngularDeg, .0, 1e-4);
}

void test_point2plane_distance_threshold()
{
    mp2p_icp::Matcher_Point2Plane m;
    mrpt::containers::yaml        p;
    p["distanceThreshold"] = "2.0*MATCH_THRESHOLD";

    m.initialize(p);

    mp2p_icp::ParameterSource globalParams;
    globalParams.attach(m);

    globalParams.updateVariable("MATCH_THRESHOLD", 1.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "2.0*MATCH_THRESHOLD"), 3.0, 1e-4);

    globalParams.updateVariable("MATCH_THRESHOLD", 0.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "2.0*MATCH_THRESHOLD"), 1.0, 1e-4);
}

void test_point2line_distance_threshold()
{
    mp2p_icp::Matcher_Point2Line m;
    mrpt::containers::yaml       p;
    p["distanceThreshold"]  = "2.0*MATCH_THRESHOLD";
    p["knn"]                = 4;
    p["lineEigenThreshold"] = 0.01;
    p["minimumLinePoints"]  = 4;

    m.initialize(p);

    mp2p_icp::ParameterSource globalParams;
    globalParams.attach(m);

    globalParams.updateVariable("MATCH_THRESHOLD", 1.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "2.0*MATCH_THRESHOLD"), 3.0, 1e-4);

    globalParams.updateVariable("MATCH_THRESHOLD", 0.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "2.0*MATCH_THRESHOLD"), 1.0, 1e-4);
}

void test_adaptive_distances()
{
    mp2p_icp::Matcher_Adaptive m;
    mrpt::containers::yaml     p;
    p["confidenceInterval"]        = 0.80;
    p["firstToSecondDistanceMax"]  = 1.2;
    p["enableDetectPlanes"]        = false;
    p["absoluteMaxSearchDistance"] = "10.0*MATCH_THRESHOLD";
    p["minimumCorrDist"]           = "0.5*MATCH_THRESHOLD";
    p["planeMinimumDistance"]      = "0.2*MATCH_THRESHOLD";

    m.initialize(p);

    mp2p_icp::ParameterSource globalParams;
    globalParams.attach(m);

    globalParams.updateVariable("MATCH_THRESHOLD", 1.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "10.0*MATCH_THRESHOLD"), 15.0, 1e-4);
    ASSERT_NEAR_(declaredParameterValue(m, "0.5*MATCH_THRESHOLD"), 0.75, 1e-4);
    ASSERT_NEAR_(declaredParameterValue(m, "0.2*MATCH_THRESHOLD"), 0.30, 1e-4);

    globalParams.updateVariable("MATCH_THRESHOLD", 0.5);
    globalParams.realize();
    ASSERT_NEAR_(declaredParameterValue(m, "10.0*MATCH_THRESHOLD"), 5.0, 1e-4);
    ASSERT_NEAR_(declaredParameterValue(m, "0.5*MATCH_THRESHOLD"), 0.25, 1e-4);
    ASSERT_NEAR_(declaredParameterValue(m, "0.2*MATCH_THRESHOLD"), 0.10, 1e-4);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_points_distance_threshold();
        test_point2plane_distance_threshold();
        test_point2line_distance_threshold();
        test_adaptive_distances();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
