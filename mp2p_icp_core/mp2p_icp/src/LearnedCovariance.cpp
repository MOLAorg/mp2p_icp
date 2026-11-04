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
 * @file   LearnedCovariance.cpp
 * @brief  Learned correction of the analytic registration covariance.
 */

#include <mp2p_icp/LearnedCovariance.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <cmath>
#include <fstream>

using namespace mp2p_icp;

namespace
{
std::string expect_word(std::istream& is, const std::string& word)
{
    std::string w;
    is >> w;
    ASSERTMSG_(is && w == word, "LearnedCovariance: expected '" + word + "', got '" + w + "'");
    return w;
}

size_t read_count(std::istream& is, const std::string& word)
{
    expect_word(is, word);
    size_t n = 0;
    is >> n;
    ASSERTMSG_(is, "LearnedCovariance: bad count after '" + word + "'");
    return n;
}

Eigen::VectorXd read_vector(std::istream& is, size_t n)
{
    Eigen::VectorXd v(static_cast<Eigen::Index>(n));
    for (size_t i = 0; i < n; i++)
    {
        is >> v[static_cast<Eigen::Index>(i)];
    }
    ASSERTMSG_(is, "LearnedCovariance: truncated vector");
    return v;
}

double gelu(double x) { return 0.5 * x * (1.0 + std::erf(x / std::sqrt(2.0))); }

}  // namespace

void LearnedCovariance::load(const std::string& file)
{
    std::ifstream is(file);
    ASSERTMSG_(is.is_open(), "LearnedCovariance: cannot open " + file);

    expect_word(is, "mp2p_icp_learned_covariance");
    int version = 0;
    is >> version;
    ASSERTMSG_(version == 1, "LearnedCovariance: unsupported format version");

    const size_t nf = read_count(is, "features");
    featNames_.resize(nf);
    median_.resize(static_cast<Eigen::Index>(nf));
    mean_.resize(static_cast<Eigen::Index>(nf));
    std_.resize(static_cast<Eigen::Index>(nf));
    for (size_t i = 0; i < nf; i++)
    {
        const auto k = static_cast<Eigen::Index>(i);
        is >> featNames_[i] >> median_[k] >> mean_[k] >> std_[k];
    }
    ASSERTMSG_(is, "LearnedCovariance: truncated feature table");

    const size_t nn = read_count(is, "nan_indicators");
    nanIdx_.clear();
    for (size_t i = 0; i < nn; i++)
    {
        std::string n;
        is >> n;
        const auto it = std::find(featNames_.begin(), featNames_.end(), n);
        ASSERTMSG_(it != featNames_.end(), "LearnedCovariance: unknown indicator " + n);
        nanIdx_.push_back(static_cast<size_t>(it - featNames_.begin()));
    }

    const size_t nc = read_count(is, "context");
    context_.clear();
    for (size_t i = 0; i < nc; i++)
    {
        std::string key;
        size_t      nv = 0;
        is >> key >> nv;
        std::vector<std::string> vs(nv);
        for (auto& v : vs)
        {
            is >> v;
        }
        context_.emplace_back(key, vs);
    }

    expect_word(is, "clip");
    is >> clip_;
    expect_word(is, "layernorm_eps");
    is >> lnEps_;

    const size_t nm = read_count(is, "members");
    members_.clear();
    for (size_t m = 0; m < nm; m++)
    {
        const size_t nl = read_count(is, "layers");
        Member       mem;
        for (size_t l = 0; l < nl; l++)
        {
            std::string kind;
            is >> kind;
            Layer ly;
            if (kind == "linear")
            {
                size_t rows = 0;
                size_t cols = 0;
                is >> rows >> cols;
                ly.W.resize(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
                for (size_t r = 0; r < rows; r++)
                {
                    for (size_t c = 0; c < cols; c++)
                    {
                        is >> ly.W(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
                    }
                }
                ly.b = read_vector(is, rows);
            }
            else
            {
                ASSERTMSG_(kind == "layernorm", "LearnedCovariance: unknown layer " + kind);
                size_t n = 0;
                is >> n;
                ly.linear = false;
                ly.w      = read_vector(is, n);
                ly.b      = read_vector(is, n);
            }
            mem.push_back(std::move(ly));
        }
        ASSERTMSG_(
            !mem.empty() && mem.back().linear && mem.back().W.rows() == 22,
            "LearnedCovariance: the last layer must be a 22-output linear head");
        members_.push_back(std::move(mem));
    }

    // Map the model's features onto the evaluator's, by name.
    const auto& evNames = CovFeatureEvaluator::featureNames();
    featMap_.clear();
    for (const auto& n : featNames_)
    {
        const auto it = std::find(evNames.begin(), evNames.end(), n);
        ASSERTMSG_(
            it != evNames.end(), "LearnedCovariance: the evaluator has no feature named " + n);
        featMap_.push_back(static_cast<int>(it - evNames.begin()));
    }
}

Eigen::VectorXd LearnedCovariance::input(
    const CovFeatureResult& r, const std::map<std::string, std::string>& context) const
{
    size_t nCtx = 0;
    for (const auto& [k, vs] : context_)
    {
        nCtx += vs.size();
    }
    const auto      nf = static_cast<Eigen::Index>(featNames_.size());
    Eigen::VectorXd x(nf + static_cast<Eigen::Index>(nanIdx_.size() + nCtx));

    std::vector<bool> isNaN(featNames_.size(), false);
    for (Eigen::Index i = 0; i < nf; i++)
    {
        double v = r.features.at(static_cast<size_t>(featMap_[static_cast<size_t>(i)]));
        if (!std::isfinite(v))
        {
            isNaN[static_cast<size_t>(i)] = true;
            v                             = median_[i];
        }
        x[i] = std::clamp((v - mean_[i]) / std_[i], -clip_, clip_);
    }
    Eigen::Index k = nf;
    for (const auto idx : nanIdx_)
    {
        x[k++] = isNaN[idx] ? 1.0 : 0.0;
    }
    for (const auto& [key, vs] : context_)
    {
        const auto        it  = context.find(key);
        const std::string val = it != context.end() ? it->second : std::string();
        for (const auto& v : vs)
        {
            x[k++] = (v == val) ? 1.0 : 0.0;
        }
    }
    return x;
}

LearnedCovariance::Output LearnedCovariance::predict(
    const CovFeatureResult& r, const std::map<std::string, std::string>& context) const
{
    ASSERTMSG_(loaded(), "LearnedCovariance: load() a model first");
    ASSERTMSG_(r.ok, "LearnedCovariance: the evaluator result is not usable (ok=false)");

    const Eigen::VectorXd x = input(r, context);

    Output out;
    for (const auto& mem : members_)
    {
        Eigen::VectorXd h = x;
        for (const auto& ly : mem)
        {
            if (ly.linear)
            {
                h = ly.W * h + ly.b;
            }
            else
            {
                const double    mu  = h.mean();
                const double    var = (h.array() - mu).square().mean();
                Eigen::VectorXd n   = (h.array() - mu) / std::sqrt(var + lnEps_);
                h                   = n.cwiseProduct(ly.w) + ly.b;
                for (Eigen::Index i = 0; i < h.size(); i++)
                {
                    h[i] = gelu(h[i]);
                }
            }
        }
        // Head: s (6), strictly lower entries of L_m in row-major order (15),
        // failure logit (1).
        Eigen::Matrix<double, 6, 6> Lm = Eigen::Matrix<double, 6, 6>::Zero();
        for (int i = 0; i < 6; i++)
        {
            Lm(i, i) = std::exp(h[i]);
        }
        int o = 6;
        for (int i = 1; i < 6; i++)
        {
            for (int j = 0; j < i; j++)
            {
                Lm(i, j) = h[o++];
            }
        }
        const Eigen::Matrix<double, 6, 6> A = r.L_a * Lm;
        out.Sigma += A * A.transpose();
        out.pFail += 1.0 / (1.0 + std::exp(-h[21]));
    }
    const auto n = static_cast<double>(members_.size());
    out.Sigma /= n;
    out.pFail /= n;
    return out;
}
