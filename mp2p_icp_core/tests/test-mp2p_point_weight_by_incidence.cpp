/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/PointWeightByIncidence.h>
#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>

namespace
{
void test_disabled_by_default()
{
    const mp2p_icp::PointWeightByIncidence w;
    ASSERT_(!w.enabled());
    // Every incidence weighs the same while disabled.
    ASSERT_NEAR_(w(1.0), 1.0, 1e-12);
    ASSERT_NEAR_(w(0.01), 1.0, 1e-12);
}

void test_knee_shape()
{
    mp2p_icp::PointWeightByIncidence w;
    w.alpha     = 1.0;
    w.refCos    = 0.5;
    w.minWeight = 0.0;
    w.maxWeight = 1.0;
    ASSERT_(w.enabled());

    // Head-on is capped by the ceiling, grazing decays linearly in |cos|.
    ASSERT_NEAR_(w(1.0), 1.0, 1e-9);
    ASSERT_NEAR_(w(0.5), 1.0, 1e-9);
    ASSERT_NEAR_(w(0.25), 0.5, 1e-9);
    ASSERT_NEAR_(w(0.05), 0.1, 1e-9);

    // Monotone in the grazing direction.
    ASSERT_(w(0.4) > w(0.2));
    ASSERT_(w(0.2) > w(0.1));
}

void test_clamping()
{
    mp2p_icp::PointWeightByIncidence w;
    w.alpha     = 2.0;
    w.refCos    = 0.5;
    w.minWeight = 0.2;
    w.maxWeight = 1.5;

    // The floor is what keeps grazing returns in play at a steep exponent.
    ASSERT_NEAR_(w(0.001), 0.2, 1e-9);
    // And the ceiling bounds the head-on end.
    ASSERT_NEAR_(w(1.0), 1.5, 1e-9);
}

void test_degenerate_inputs()
{
    mp2p_icp::PointWeightByIncidence w;
    w.alpha     = 1.0;
    w.refCos    = 0.5;
    w.minWeight = 0.01;
    w.maxWeight = 1.0;

    // A cosine of zero must not produce a NaN or a division by zero.
    const double atZero = w(0.0);
    ASSERT_(std::isfinite(atZero));
    ASSERT_(atZero >= w.minWeight);

    // Neither must a nonsensical reference.
    w.refCos = 0.0;
    ASSERT_(std::isfinite(w(0.5)));
}
}  // namespace

int main(int, char**)
{
    try
    {
        test_disabled_by_default();
        test_knee_shape();
        test_clamping();
        test_degenerate_inputs();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
