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
 * @file   test-mp2p_cov_features.cpp
 * @brief  Unit tests for CovFeatureEvaluator on synthetic scenes
 */

#include <mp2p_icp/IcpCovFeatures.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
// Points on the inner faces of an axis-aligned box, with an optional opening
// of the two x-faces (a corridor along x).
mrpt::maps::CSimplePointsMap::Ptr box_scene(
    double lx, double ly, double lz, bool openEnds, double step, uint32_t seed)
{
    auto                           m = mrpt::maps::CSimplePointsMap::Create();
    mrpt::random::CRandomGenerator rng;
    rng.randomize(seed);
    auto jitter = [&]() { return rng.drawUniform(-0.4 * step, 0.4 * step); };

    for (double a = -lx; a <= lx; a += step)
    {
        for (double b = -ly; b <= ly; b += step)
        {
            m->insertPoint(a + jitter(), b + jitter(), -lz);
            m->insertPoint(a + jitter(), b + jitter(), lz);
        }
        for (double b = -lz; b <= lz; b += step)
        {
            m->insertPoint(a + jitter(), -ly, b + jitter());
            m->insertPoint(a + jitter(), ly, b + jitter());
        }
    }
    if (!openEnds)
    {
        for (double a = -ly; a <= ly; a += step)
        {
            for (double b = -lz; b <= lz; b += step)
            {
                m->insertPoint(-lx, a + jitter(), b + jitter());
                m->insertPoint(lx, a + jitter(), b + jitter());
            }
        }
    }
    // A pillar breaks the box's symmetries, so every DOF is constrained.
    for (double b = -lz; b <= lz; b += step)
    {
        for (double a = 0; a < 2 * M_PI; a += step / 0.3)
        {
            m->insertPoint(0.3 * lx + 0.3 * std::cos(a), 0.2 * ly + 0.3 * std::sin(a), b);
        }
    }
    return m;
}

// The same scene, expressed in the vehicle frame of `pose`.
mrpt::maps::CSimplePointsMap::Ptr to_vehicle(
    const mrpt::maps::CSimplePointsMap& world, const mrpt::poses::CPose3D& pose)
{
    auto        m  = mrpt::maps::CSimplePointsMap::Create();
    const auto& xs = world.getPointsBufferRef_x();
    const auto& ys = world.getPointsBufferRef_y();
    const auto& zs = world.getPointsBufferRef_z();
    for (size_t i = 0; i < xs.size(); i++)
    {
        double lx = 0;
        double ly = 0;
        double lz = 0;
        pose.inverseComposePoint(xs[i], ys[i], zs[i], lx, ly, lz);
        m->insertPoint(lx, ly, lz);
    }
    return m;
}

void test_at_truth_and_offset()
{
    const auto world     = box_scene(6.0, 4.0, 2.0, false, 0.1, 1);
    const auto truth     = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.5, -0.3, 0.1, 0.2, 0.0, 0.0);
    const auto scanWorld = box_scene(6.0, 4.0, 2.0, false, 0.1, 2);  // other samples
    const auto scan      = to_vehicle(*scanWorld, truth);

    mp2p_icp::CovFeatureEvaluator ev;
    ev.setReference({{world.get(), mrpt::poses::CPose3D::Identity()}});
    ev.setScan(*scan);
    ASSERT_(ev.referenceSize() > 1000);
    ASSERT_(ev.scanSize() > 1000);

    mp2p_icp::CovFeatureContext ctx;
    ctx.initialGuess = truth;
    const auto r     = ev.evaluate(truth, ctx);
    ASSERT_(r.ok);
    ASSERT_EQUAL_(r.features.size(), mp2p_icp::CovFeatureEvaluator::featureNames().size());

    // At the truth the analytic sigmas are millimetric and the common cost's
    // optimum is within a fraction of them.
    for (int i = 0; i < 3; i++)
    {
        ASSERT_LT_(std::sqrt(r.Sigma_a(i, i)), 0.01);
    }
    const auto& names = mp2p_icp::CovFeatureEvaluator::featureNames();
    auto        feat  = [&](const std::string& n)
    {
        const auto it = std::find(names.begin(), names.end(), n);
        ASSERT_(it != names.end());
        return static_cast<double>(r.features[static_cast<size_t>(it - names.begin())]);
    };
    const double fracAssoc = feat("B_frac_assoc");
    ASSERT_GT_(fracAssoc, 0.95);

    // Off by 5 cm along x: the whitened step back to the optimum is many
    // analytic sigmas long on at least one axis.
    const auto off = truth + mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.05, 0, 0, 0, 0, 0);
    const auto r2  = ev.evaluate(off, ctx);
    ASSERT_(r2.ok);
    double maxStep  = 0;
    double maxStep0 = 0;
    for (int i = 0; i < 6; i++)
    {
        const auto idx = static_cast<size_t>(
            std::find(names.begin(), names.end(), "C_step_a" + std::to_string(i)) - names.begin());
        maxStep  = std::max(maxStep, static_cast<double>(r2.features[idx]));
        maxStep0 = std::max(maxStep0, static_cast<double>(r.features[idx]));
    }
    ASSERT_GT_(maxStep, 1.0);  // log10(1 + |step|) > 1  <=>  |step| > 9 sigmas
    ASSERT_LT_(maxStep0, maxStep);

    // Deterministic.
    const auto r3 = ev.evaluate(truth, ctx);
    for (size_t i = 0; i < r.features.size(); i++)
    {
        const bool bothNaN = std::isnan(r.features[i]) && std::isnan(r3.features[i]);
        ASSERT_(bothNaN || r.features[i] == r3.features[i]);
    }
    std::cout << "[Test Passed] evaluator at truth and at an offset\n";
}

void test_corridor_degeneracy()
{
    const auto world     = box_scene(20.0, 1.5, 1.5, true, 0.1, 3);
    const auto scanWorld = box_scene(20.0, 1.5, 1.5, true, 0.1, 4);
    const auto truth     = mrpt::poses::CPose3D::Identity();
    const auto scan      = to_vehicle(*scanWorld, truth);

    mp2p_icp::CovFeatureEvaluator ev;
    ev.setReference({{world.get(), truth}});
    ev.setScan(*scan);
    const auto r = ev.evaluate(truth, {});
    ASSERT_(r.ok);

    // The pillar makes x observable, but far worse than y and z.
    ASSERT_GT_(r.Sigma_a(0, 0), 10 * r.Sigma_a(1, 1));
    ASSERT_GT_(r.Sigma_a(0, 0), 10 * r.Sigma_a(2, 2));
    std::cout << "[Test Passed] corridor degeneracy shows in the analytic covariance\n";
}

// A sliding window maintained with addReferenceCloud() and
// removeReferenceCloud() must give the features of setReference() on the
// same clouds.
void test_incremental_reference()
{
    const auto truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.5, -0.3, 0.1, 0.2, 0.0, 0.0);
    std::vector<mrpt::maps::CSimplePointsMap::Ptr> clouds;
    std::vector<mrpt::poses::CPose3D>              poses;
    for (uint32_t k = 0; k < 5; k++)
    {
        const auto p =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.2 * k, 0.05 * k, 0.0, 0.03 * k, 0.0, 0.0);
        clouds.push_back(to_vehicle(*box_scene(6.0, 4.0, 2.0, false, 0.1, 10 + k), p));
        poses.push_back(p);
    }
    const auto scan = to_vehicle(*box_scene(6.0, 4.0, 2.0, false, 0.1, 99), truth);

    mp2p_icp::CovFeatureEvaluator batch;
    batch.setReference(
        {{clouds[1].get(), poses[1]},
         {clouds[2].get(), poses[2]},
         {clouds[3].get(), poses[3]},
         {clouds[4].get(), poses[4]}});
    batch.setScan(*scan);

    mp2p_icp::CovFeatureEvaluator inc;
    for (size_t k = 0; k < 4; k++)
    {
        inc.addReferenceCloud(*clouds[k], poses[k]);
    }
    inc.finalizeReference();
    inc.addReferenceCloud(*clouds[4], poses[4]);
    inc.removeReferenceCloud(*clouds[0], poses[0]);
    inc.finalizeReference();
    inc.setScan(*scan);

    ASSERT_EQUAL_(inc.referenceSize(), batch.referenceSize());

    mp2p_icp::CovFeatureContext ctx;
    ctx.initialGuess = truth;
    const auto rb    = batch.evaluate(truth, ctx);
    const auto ri    = inc.evaluate(truth, ctx);
    ASSERT_(rb.ok && ri.ok);
    for (size_t i = 0; i < rb.features.size(); i++)
    {
        const double a = rb.features[i];
        const double b = ri.features[i];
        if (std::isnan(a) && std::isnan(b))
        {
            continue;
        }
        ASSERTMSG_(
            std::abs(a - b) <= 1e-4 * (1.0 + std::abs(a)),
            mrpt::format(
                "feature %s: batch %g, incremental %g",
                mp2p_icp::CovFeatureEvaluator::featureNames().at(i).c_str(), a, b));
    }
    std::cout << "[Test Passed] incremental reference matches the batch one\n";
}

}  // namespace

int main()
{
    try
    {
        test_at_truth_and_offset();
        test_corridor_degeneracy();
        test_incremental_reference();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
