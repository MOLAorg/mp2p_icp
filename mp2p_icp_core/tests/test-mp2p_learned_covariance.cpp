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
 * @file   test-mp2p_learned_covariance.cpp
 * @brief  Unit tests for the learned-covariance model file and estimator
 */

#include <mp2p_icp/IcpCovFeatures.h>
#include <mp2p_icp/LearnedCovariance.h>
#include <mp2p_icp/LearnedCovarianceEstimator.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <future>
#include <iostream>
#include <string>
#include <vector>

namespace
{
// A planar-walled room with a pillar, every DOF constrained.
mrpt::maps::CSimplePointsMap::Ptr room(uint32_t seed, const mrpt::poses::CPose3D& viewer)
{
    auto                           m = mrpt::maps::CSimplePointsMap::Create();
    mrpt::random::CRandomGenerator rng;
    rng.randomize(seed);
    const double step = 0.1;
    auto         add  = [&](double x, double y, double z)
    {
        double lx = 0;
        double ly = 0;
        double lz = 0;
        viewer.inverseComposePoint(
            x + rng.drawUniform(-0.04, 0.04), y + rng.drawUniform(-0.04, 0.04), z, lx, ly, lz);
        m->insertPoint(lx, ly, lz);
    };
    for (double a = -6; a <= 6; a += step)
    {
        for (double b = -4; b <= 4; b += step)
        {
            add(a, b, -2);
            add(a, b, 2);
        }
        for (double b = -2; b <= 2; b += step)
        {
            add(a, -4, b);
            add(a, 4, b);
        }
    }
    for (double a = -4; a <= 4; a += step)
    {
        for (double b = -2; b <= 2; b += step)
        {
            add(-6, a, b);
            add(6, a, b);
        }
    }
    for (double b = -2; b <= 2; b += step)
    {
        for (double a = 0; a < 2 * M_PI; a += 0.3)
        {
            add(1.8 + 0.3 * std::cos(a), 0.8 + 0.3 * std::sin(a), b);
        }
    }
    return m;
}

class Writer
{
   public:
    explicit Writer(const std::string& file) : os_(file, std::ios::binary) { ASSERT_(os_.good()); }

    template <typename T>
    void pod(T v)
    {
        os_.write(reinterpret_cast<const char*>(&v), sizeof(T));
    }
    void u32(size_t v) { pod<uint32_t>(static_cast<uint32_t>(v)); }
    void str(const std::string& s)
    {
        u32(s.size());
        os_.write(s.data(), static_cast<std::streamsize>(s.size()));
    }
    void floats(size_t n, float v)
    {
        for (size_t i = 0; i < n; i++)
        {
            pod<float>(v);
        }
    }
    void floats(const std::vector<float>& v)
    {
        for (const float x : v)
        {
            pod<float>(x);
        }
    }

   private:
    std::ofstream os_;
};

// A model whose hidden layers output zeros, so each member's head returns its
// biases: member m has L_m = scale_m * I and a failure logit of zero.
std::string write_model(
    const std::string& name, const std::vector<double>& scales, uint32_t featureVersion,
    const char* magic = "MP2PLCOV")
{
    const std::string file   = mrpt::system::getTempFileName() + name;
    const auto&       names  = mp2p_icp::CovFeatureEvaluator::featureNames();
    const size_t      nf     = 5;
    const size_t      nIn    = nf + 2;  // + two "mode" one-hots
    const size_t      hidden = 8;

    Writer w(file);
    for (int i = 0; i < 8; i++)
    {
        w.pod<char>(magic[i]);
    }
    w.u32(2);
    w.u32(featureVersion);
    w.str("unit-test model");
    w.u32(nf);
    for (size_t i = 0; i < nf; i++)
    {
        w.str(names.at(i));
        w.pod<double>(0.0);
        w.pod<double>(0.0);
        w.pod<double>(1.0);
    }
    w.u32(0);  // no missing-value indicators
    w.u32(1);
    w.str("mode");
    w.u32(2);
    w.str("s2m");
    w.str("s2s");
    w.pod<double>(8.0);
    w.pod<double>(1e-5);
    w.u32(scales.size());
    for (const double s : scales)
    {
        w.u32(3);
        w.pod<uint8_t>(0);  // linear, all zeros
        w.u32(hidden);
        w.u32(nIn);
        w.floats(hidden * nIn, 0.f);
        w.floats(hidden, 0.f);
        w.pod<uint8_t>(1);  // layernorm
        w.u32(hidden);
        w.floats(hidden, 1.f);
        w.floats(hidden, 0.f);
        w.pod<uint8_t>(0);  // head
        w.u32(22);
        w.u32(hidden);
        w.floats(22 * hidden, 0.f);
        w.floats(6, static_cast<float>(std::log(s)));
        w.floats(16, 0.f);
    }
    return file;
}

mp2p_icp::CovFeatureResult evaluate_room(mp2p_icp::CovFeatureEvaluator& ev)
{
    const auto truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.3, -0.2, 0.05, 0.1, 0.0, 0.0);
    ev.setReference({{room(1, mrpt::poses::CPose3D::Identity()).get(), {}}});
    ev.setScan(*room(2, truth));
    mp2p_icp::CovFeatureContext ctx;
    ctx.initialGuess = truth;
    const auto r     = ev.evaluate(truth, ctx);
    ASSERT_(r.ok);
    return r;
}

void test_model_file()
{
    mp2p_icp::LearnedCovariance lc;
    lc.load(write_model("_ok.bin", {2.0, 1.0}, mp2p_icp::CovFeatureEvaluator::kVersion));
    ASSERT_EQUAL_(lc.numMembers(), 2U);
    ASSERT_EQUAL_(lc.note(), std::string("unit-test model"));

    mp2p_icp::CovFeatureEvaluator ev;
    const auto                    r    = evaluate_room(ev);
    const auto                    pred = lc.predict(r, {{"mode", "s2m"}});

    // Members predict 4 and 1 times the analytic covariance; the ensemble, their
    // mean (up to the float32 rounding of the stored log-scales).
    const Eigen::Matrix<double, 6, 6> expected = 2.5 * r.Sigma_a;
    ASSERT_LT_((pred.Sigma - expected).norm(), 1e-6 * expected.norm());
    ASSERT_LT_(std::abs(pred.pFail - 0.5), 1e-12);

    bool threw = false;
    try
    {
        mp2p_icp::LearnedCovariance bad;
        bad.load(write_model("_magic.bin", {1.0}, 1, "NOTMODEL"));
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);

    threw = false;
    try
    {
        mp2p_icp::LearnedCovariance bad;
        bad.load(write_model("_ver.bin", {1.0}, mp2p_icp::CovFeatureEvaluator::kVersion + 1));
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);
    std::cout << "[Test Passed] learned-covariance model file\n";
}

// The estimator evaluates each request against the window as it was when the
// request was made, and gives the result of the synchronous path.
void test_estimator()
{
    mp2p_icp::LearnedCovarianceEstimator::Parameters p;
    p.model_file   = write_model("_est.bin", {2.0, 1.0}, mp2p_icp::CovFeatureEvaluator::kVersion);
    p.window_scans = 2;
    p.context      = {{"mode", "s2m"}};
    p.max_waiting  = 100;  // keep every request
    mp2p_icp::LearnedCovarianceEstimator est(p);

    const auto truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.3, -0.2, 0.05, 0.1, 0.0, 0.0);
    std::vector<mrpt::poses::CPose3D> poses;
    for (int k = 0; k < 3; k++)
    {
        poses.push_back(
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.1 * k, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
    std::vector<mrpt::maps::CPointsMap::Ptr> clouds;
    for (int k = 0; k < 3; k++)
    {
        clouds.push_back(room(10 + static_cast<uint32_t>(k), poses[static_cast<size_t>(k)]));
    }
    const auto scan = room(99, truth);

    mp2p_icp::CovFeatureContext ctx;
    ctx.initialGuess = truth;

    const auto f0 = est.submit({scan, truth, ctx});  // empty window
    for (size_t k = 0; k < 3; k++)
    {
        est.pushToWindow(clouds[k], poses[k]);
    }
    const auto f1 = est.submit({scan, truth, ctx});  // window: clouds 1, 2
    est.clearWindow();
    const auto f2 = est.submit({scan, truth, ctx});  // empty again

    ASSERT_(!f0.get().ok);
    ASSERT_(!f2.get().ok);
    const auto r1 = f1.get();
    ASSERT_(r1.ok);

    mp2p_icp::CovFeatureEvaluator ev;
    ev.setReference({{clouds[1].get(), poses[1]}, {clouds[2].get(), poses[2]}});
    ev.setScan(*scan);
    const auto ref = ev.evaluate(truth, ctx);
    ASSERT_(ref.ok);
    const Eigen::Matrix<double, 6, 6> expected = 2.5 * ref.Sigma_a;
    // The incremental reference equals the batch one up to rounding.
    ASSERT_LT_((r1.Sigma - expected).norm(), 1e-4 * expected.norm());
    std::cout << "[Test Passed] learned-covariance estimator\n";
}

// Small exactly representable weights, different for every entry.
std::vector<float> test_weights(size_t n, int salt)
{
    std::vector<float> v(n);
    for (size_t i = 0; i < n; i++)
    {
        v[i] = static_cast<float>((static_cast<int>(i) * 5 + salt) % 9 - 4) / 16.0f;
    }
    return v;
}

double gelu_ref(double x) { return 0.5 * x * (1.0 + std::erf(x / std::sqrt(2.0))); }

// A one-member model with nonzero weights everywhere, and an independent
// evaluation of it: feature mapping by name, standardization, one-hots,
// layernorm, GELU and the head.
void test_nonzero_model()
{
    mp2p_icp::CovFeatureEvaluator ev;
    const auto                    r = evaluate_room(ev);

    // Two finite features, stored in the opposite order to the evaluator's.
    std::vector<size_t> idx;
    for (size_t i = 0; i < r.features.size() && idx.size() < 2; i++)
    {
        if (std::isfinite(r.features[i]))
        {
            idx.push_back(i);
        }
    }
    ASSERT_EQUAL_(idx.size(), 2U);
    std::swap(idx[0], idx[1]);
    const std::vector<double> mean = {0.5, -0.25};
    const std::vector<double> stdv = {2.0, 0.5};

    const size_t      nIn    = 4;  // 2 features + 2 one-hots
    const size_t      hidden = 3;
    const auto        W1     = test_weights(hidden * nIn, 1);
    const auto        b1     = test_weights(hidden, 2);
    const auto        gain   = std::vector<float>{1.0f, 0.5f, 2.0f};
    const auto        beta   = std::vector<float>{0.125f, -0.25f, 0.0f};
    const auto        W2     = test_weights(22 * hidden, 3);
    const auto        b2     = test_weights(22, 4);
    const std::string file   = mrpt::system::getTempFileName() + "_nonzero.bin";
    const auto&       names  = mp2p_icp::CovFeatureEvaluator::featureNames();
    {
        Writer w(file);
        for (const char c : std::string("MP2PLCOV"))
        {
            w.pod<char>(c);
        }
        w.u32(2);
        w.u32(mp2p_icp::CovFeatureEvaluator::kVersion);
        w.str("nonzero");
        w.u32(2);
        for (size_t i = 0; i < 2; i++)
        {
            w.str(names.at(idx[i]));
            w.pod<double>(0.0);
            w.pod<double>(mean[i]);
            w.pod<double>(stdv[i]);
        }
        w.u32(0);
        w.u32(1);
        w.str("mode");
        w.u32(2);
        w.str("s2m");
        w.str("s2s");
        w.pod<double>(8.0);
        w.pod<double>(1e-5);
        w.u32(1);
        w.u32(3);
        w.pod<uint8_t>(0);
        w.u32(hidden);
        w.u32(nIn);
        w.floats(W1);
        w.floats(b1);
        w.pod<uint8_t>(1);
        w.u32(hidden);
        w.floats(gain);
        w.floats(beta);
        w.pod<uint8_t>(0);
        w.u32(22);
        w.u32(hidden);
        w.floats(W2);
        w.floats(b2);
    }
    mp2p_icp::LearnedCovariance lc;
    lc.load(file);

    const auto reference = [&](const mp2p_icp::CovFeatureResult& in, const std::string& mode)
    {
        std::vector<double> x(nIn);
        for (size_t i = 0; i < 2; i++)
        {
            x[i] = std::clamp(
                (static_cast<double>(in.features[idx[i]]) - mean[i]) / stdv[i], -8.0, 8.0);
        }
        x[2] = mode == "s2m" ? 1.0 : 0.0;
        x[3] = mode == "s2s" ? 1.0 : 0.0;

        std::vector<double> h(hidden);
        for (size_t i = 0; i < hidden; i++)
        {
            h[i] = b1[i];
            for (size_t j = 0; j < nIn; j++)
            {
                h[i] += static_cast<double>(W1[i * nIn + j]) * x[j];
            }
        }
        double mu = 0;
        for (const double v : h)
        {
            mu += v / static_cast<double>(hidden);
        }
        double var = 0;
        for (const double v : h)
        {
            var += (v - mu) * (v - mu) / static_cast<double>(hidden);
        }
        for (size_t i = 0; i < hidden; i++)
        {
            h[i] = gelu_ref((h[i] - mu) / std::sqrt(var + 1e-5) * gain[i] + beta[i]);
        }
        std::vector<double> o(22);
        for (size_t i = 0; i < 22; i++)
        {
            o[i] = b2[i];
            for (size_t j = 0; j < hidden; j++)
            {
                o[i] += static_cast<double>(W2[i * hidden + j]) * h[j];
            }
        }
        Eigen::Matrix<double, 6, 6> Lm = Eigen::Matrix<double, 6, 6>::Zero();
        size_t                      k  = 6;
        for (int i = 0; i < 6; i++)
        {
            Lm(i, i) = std::exp(o[static_cast<size_t>(i)]);
            for (int j = 0; j < i; j++)
            {
                Lm(i, j) = o[k++];
            }
        }
        const Eigen::Matrix<double, 6, 6> A = in.L_a * Lm;
        return std::pair<Eigen::Matrix<double, 6, 6>, double>(
            A * A.transpose(), 1.0 / (1.0 + std::exp(-o[21])));
    };

    auto r2 = r;
    r2.features[idx[0]] += 1.0f;
    std::vector<Eigen::Matrix<double, 6, 6>> seen;
    const mp2p_icp::CovFeatureResult*        inputs[] = {&r, &r2};
    for (const auto* in : inputs)
    {
        for (const std::string mode : {"s2m", "s2s"})
        {
            const auto pred        = lc.predict(*in, {{"mode", mode}});
            const auto [Sigma, pf] = reference(*in, mode);
            ASSERT_LT_((pred.Sigma - Sigma).norm(), 1e-9 * Sigma.norm());
            ASSERT_LT_(std::abs(pred.pFail - pf), 1e-12);
            for (const auto& other : seen)
            {
                ASSERT_GT_((other - pred.Sigma).norm(), 1e-3 * Sigma.norm());
            }
            seen.push_back(pred.Sigma);
        }
    }

    // A failed load keeps the model loaded before.
    bool threw = false;
    try
    {
        lc.load(write_model("_bad.bin", {1.0}, mp2p_icp::CovFeatureEvaluator::kVersion + 1));
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);
    ASSERT_EQUAL_(lc.note(), std::string("nonzero"));
    std::cout << "[Test Passed] learned-covariance nonzero model\n";
}

// When the worker falls behind: only the newest waiting evaluation survives,
// and queued scans that leave the window unseen are dropped without changing
// the result.
void test_estimator_overload()
{
    mp2p_icp::LearnedCovarianceEstimator::Parameters p;
    p.model_file   = write_model("_ovl.bin", {2.0, 1.0}, mp2p_icp::CovFeatureEvaluator::kVersion);
    p.window_scans = 2;
    p.context      = {{"mode", "s2m"}};

    bool threw = false;
    try
    {
        auto q        = p;
        q.max_waiting = 0;
        mp2p_icp::LearnedCovarianceEstimator bad(q);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);

    p.max_waiting = 1;
    mp2p_icp::LearnedCovarianceEstimator est(p);

    // A large cloud keeps the worker busy (a fraction of a second) while the
    // requests below are queued (microseconds).
    auto big = mrpt::maps::CSimplePointsMap::Create();
    for (uint32_t k = 0; k < 100; k++)
    {
        const auto  c  = room(100 + k, mrpt::poses::CPose3D::Identity());
        const auto& xs = c->getPointsBufferRef_x();
        const auto& ys = c->getPointsBufferRef_y();
        const auto& zs = c->getPointsBufferRef_z();
        for (size_t i = 0; i < xs.size(); i++)
        {
            big->insertPointFast(xs[i], ys[i], zs[i]);
        }
    }
    big->mark_as_modified();

    const auto truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.3, -0.2, 0.05, 0.1, 0.0, 0.0);
    const auto scan  = room(99, truth);
    mp2p_icp::CovFeatureContext ctx;
    ctx.initialGuess = truth;

    est.pushToWindow(big, mrpt::poses::CPose3D::Identity());
    std::vector<std::shared_future<mp2p_icp::LearnedCovarianceEstimator::Result>> f;
    for (int i = 0; i < 5; i++)
    {
        f.push_back(est.submit({scan, truth, ctx}));
    }
    for (size_t i = 1; i < 4; i++)
    {
        ASSERT_(!f[i].get().ok);
    }
    ASSERT_(f[4].get().ok);

    est.pushToWindow(big, mrpt::poses::CPose3D::Identity());
    std::vector<mrpt::poses::CPose3D>        poses;
    std::vector<mrpt::maps::CPointsMap::Ptr> clouds;
    for (int k = 0; k < 5; k++)
    {
        poses.push_back(
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.05 * k, 0.0, 0.0, 0.0, 0.0, 0.0));
        clouds.push_back(room(200 + static_cast<uint32_t>(k), poses.back()));
        est.pushToWindow(clouds.back(), poses.back());
    }
    const auto rw = est.submit({scan, truth, ctx}).get();
    ASSERT_(rw.ok);

    mp2p_icp::CovFeatureEvaluator ev;
    ev.setReference({{clouds[3].get(), poses[3]}, {clouds[4].get(), poses[4]}});
    ev.setScan(*scan);
    const auto ref = ev.evaluate(truth, ctx);
    ASSERT_(ref.ok);
    const Eigen::Matrix<double, 6, 6> expected = 2.5 * ref.Sigma_a;
    ASSERT_LT_((rw.Sigma - expected).norm(), 1e-4 * expected.norm());
    std::cout << "[Test Passed] learned-covariance estimator overload\n";
}

}  // namespace

int main()
{
    try
    {
        test_model_file();
        test_estimator();
        test_nonzero_model();
        test_estimator_overload();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
