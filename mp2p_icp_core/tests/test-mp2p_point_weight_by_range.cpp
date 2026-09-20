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
 * @file   test-mp2p_point_weight_by_range.cpp
 * @brief  Unit tests for per-point weighting of pairings by range
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2026
 */

#include <mp2p_icp/Matcher_Cov2Cov.h>
#include <mp2p_icp/PointWeightByRange.h>
#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>

namespace
{
void test_disabled_by_default()
{
    const mp2p_icp::PointWeightByRange w;
    ASSERT_(!w.enabled());
    for (const double r : {0.5, 5.0, 20.0, 100.0, 500.0})
    {
        ASSERT_NEAR_(w(r), 1.0, 1e-12);
    }
}

void test_knee_shape()
{
    mp2p_icp::PointWeightByRange w;
    w.alpha    = 1.0;
    w.refRange = 20.0;
    ASSERT_(w.enabled());

    // Everything at or below the reference range saturates at maxWeight.
    ASSERT_NEAR_(w(1.0), 1.0, 1e-12);
    ASSERT_NEAR_(w(20.0), 1.0, 1e-12);

    // Beyond it, the weight decays as the ratio of the ranges.
    ASSERT_NEAR_(w(40.0), 0.5, 1e-12);
    ASSERT_NEAR_(w(80.0), 0.25, 1e-12);

    // The exponent is the decay rate.
    w.alpha = 2.0;
    ASSERT_NEAR_(w(40.0), 0.25, 1e-12);
    ASSERT_NEAR_(w(80.0), 0.0625, 1e-12);
}

void test_clamping()
{
    mp2p_icp::PointWeightByRange w;
    w.alpha     = 2.0;
    w.refRange  = 20.0;
    w.minWeight = 0.05;

    // A far point is floored, never dropped outright.
    ASSERT_NEAR_(w(1000.0), 0.05, 1e-12);

    // A negative exponent up-weights the far field instead, and the ceiling
    // is what keeps that bounded.
    w.alpha     = -1.0;
    w.minWeight = 0.0;
    w.maxWeight = 3.0;
    ASSERT_NEAR_(w(40.0), 2.0, 1e-12);
    ASSERT_NEAR_(w(200.0), 3.0, 1e-12);  // clamped

    // Range zero must not divide by zero.
    ASSERT_(std::isfinite(w(0.0)));
}

void test_matcher_yaml_roundtrip()
{
    mp2p_icp::Matcher_Cov2Cov m;

    // Default: off, so the matcher behaves as every release before this.
    ASSERT_(!m.pointWeightByRange().enabled());

    mrpt::containers::yaml p = mrpt::containers::yaml::Map();
    p["threshold"]           = 0.5;
    p["pointWeightAlpha"]    = 2.0;
    p["pointWeightRefRange"] = 15.0;
    p["pointWeightMin"]      = 0.02;
    m.initialize(p);

    const auto w = m.pointWeightByRange();
    ASSERT_(w.enabled());
    ASSERT_NEAR_(w.refRange, 15.0, 1e-9);
    ASSERT_NEAR_(w(15.0), 1.0, 1e-9);
    ASSERT_NEAR_(w(30.0), 0.25, 1e-9);
    ASSERT_NEAR_(w.minWeight, 0.02, 1e-9);
}
}  // namespace

int main(int, char**)
{
    try
    {
        test_disabled_by_default();
        test_knee_shape();
        test_clamping();
        test_matcher_yaml_roundtrip();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
