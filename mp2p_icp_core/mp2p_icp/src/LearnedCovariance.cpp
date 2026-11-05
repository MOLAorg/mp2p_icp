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
#include <mrpt/core/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>

using namespace mp2p_icp;

namespace
{
constexpr uint32_t kFormatVersion = 2;

/** Reads the little-endian binary model file. */
class ModelReader
{
   public:
    explicit ModelReader(std::istream& is) : is_(is) {}

    template <typename T>
    T pod()
    {
        T v{};
        is_.read(reinterpret_cast<char*>(&v), sizeof(T));
        ASSERTMSG_(is_.good(), "LearnedCovariance: truncated model file");
        return v;
    }

    size_t count(size_t maxValue)
    {
        const auto n = static_cast<size_t>(pod<uint32_t>());
        ASSERTMSG_(n <= maxValue, "LearnedCovariance: corrupt model file (bad count)");
        return n;
    }

    std::string str()
    {
        std::string s(count(1U << 20), '\0');
        is_.read(s.data(), static_cast<std::streamsize>(s.size()));
        ASSERTMSG_(is_.good(), "LearnedCovariance: truncated model file");
        return s;
    }

    Eigen::VectorXf floats(size_t n)
    {
        Eigen::VectorXf v(static_cast<Eigen::Index>(n));
        is_.read(
            reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(n * sizeof(float)));
        ASSERTMSG_(is_.good(), "LearnedCovariance: truncated model file");
        return v;
    }

   private:
    std::istream& is_;
};

bool host_is_little_endian()
{
    const uint16_t one   = 1;
    uint8_t        first = 0;
    std::memcpy(&first, &one, 1);
    return first == 1;
}

double gelu(double x) { return 0.5 * x * (1.0 + std::erf(x / std::sqrt(2.0))); }

}  // namespace

void LearnedCovariance::load(const std::string& file)
{
    // Parse into a fresh object, so a failed load leaves this one unchanged.
    LearnedCovariance m;
    m.parse(file);
    *this = std::move(m);
}

void LearnedCovariance::parse(const std::string& file)
{
    ASSERTMSG_(host_is_little_endian(), "LearnedCovariance: big-endian hosts are not supported");

    std::ifstream is(file, std::ios::binary);
    ASSERTMSG_(is.is_open(), "LearnedCovariance: cannot open " + file);
    ModelReader rd(is);

    std::array<char, 8> magic{};
    is.read(magic.data(), magic.size());
    ASSERTMSG_(
        is.good() && std::string(magic.data(), magic.size()) == "MP2PLCOV",
        "LearnedCovariance: not a learned-covariance model file: " + file);
    const auto version = rd.pod<uint32_t>();
    ASSERTMSG_(
        version == kFormatVersion,
        mrpt::format("LearnedCovariance: unsupported format version %u", version));
    const auto featVersion = rd.pod<uint32_t>();
    ASSERTMSG_(
        featVersion == static_cast<uint32_t>(CovFeatureEvaluator::kVersion),
        mrpt::format(
            "LearnedCovariance: the model was trained on features version %u, this evaluator "
            "computes version %d",
            featVersion, CovFeatureEvaluator::kVersion));
    note_ = rd.str();

    const size_t nf = rd.count(100000);
    featNames_.resize(nf);
    median_.resize(static_cast<Eigen::Index>(nf));
    mean_.resize(static_cast<Eigen::Index>(nf));
    std_.resize(static_cast<Eigen::Index>(nf));
    for (size_t i = 0; i < nf; i++)
    {
        const auto k  = static_cast<Eigen::Index>(i);
        featNames_[i] = rd.str();
        median_[k]    = rd.pod<double>();
        mean_[k]      = rd.pod<double>();
        std_[k]       = rd.pod<double>();
        ASSERTMSG_(
            std::isfinite(median_[k]) && std::isfinite(mean_[k]) && std::isfinite(std_[k]) &&
                std_[k] > 0,
            "LearnedCovariance: invalid statistics of feature " + featNames_[i]);
    }

    const size_t nn = rd.count(nf);
    nanIdx_.clear();
    for (size_t i = 0; i < nn; i++)
    {
        const std::string n  = rd.str();
        const auto        it = std::find(featNames_.begin(), featNames_.end(), n);
        ASSERTMSG_(it != featNames_.end(), "LearnedCovariance: unknown indicator " + n);
        nanIdx_.push_back(static_cast<size_t>(it - featNames_.begin()));
    }

    const size_t nc = rd.count(1000);
    context_.clear();
    for (size_t i = 0; i < nc; i++)
    {
        std::string              key = rd.str();
        std::vector<std::string> vs(rd.count(100000));
        for (auto& v : vs)
        {
            v = rd.str();
        }
        context_.emplace_back(std::move(key), std::move(vs));
    }

    clip_  = rd.pod<double>();
    lnEps_ = rd.pod<double>();
    ASSERTMSG_(
        std::isfinite(clip_) && clip_ > 0 && std::isfinite(lnEps_) && lnEps_ > 0,
        "LearnedCovariance: invalid clip or layernorm epsilon");

    const size_t nm = rd.count(1000);
    members_.clear();
    for (size_t m = 0; m < nm; m++)
    {
        const size_t nl = rd.count(1000);
        Member       mem;
        for (size_t l = 0; l < nl; l++)
        {
            const auto kind = rd.pod<uint8_t>();
            Layer      ly;
            if (kind == 0)
            {
                const size_t rows = rd.count(1U << 16);
                const size_t cols = rd.count(1U << 16);
                // Row-major on disk:
                const Eigen::VectorXf w = rd.floats(rows * cols);
                ly.W =
                    Eigen::Map<const Eigen::Matrix<
                        float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                        w.data(), static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols))
                        .cast<double>();
                ly.b = rd.floats(rows).cast<double>();
            }
            else
            {
                ASSERTMSG_(kind == 1, "LearnedCovariance: unknown layer kind");
                const size_t n = rd.count(1U << 16);
                ly.linear      = false;
                ly.w           = rd.floats(n).cast<double>();
                ly.b           = rd.floats(n).cast<double>();
            }
            ASSERTMSG_(
                ly.W.allFinite() && ly.w.allFinite() && ly.b.allFinite(),
                "LearnedCovariance: non-finite weights");
            mem.push_back(std::move(ly));
        }
        ASSERTMSG_(
            !mem.empty() && mem.back().linear && mem.back().W.rows() == 22,
            "LearnedCovariance: the last layer must be a 22-output linear head");
        members_.push_back(std::move(mem));
    }
    ASSERTMSG_(!members_.empty(), "LearnedCovariance: the model has no members");

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

    // Each layer's width must match its input: the first one, the features
    // plus the indicators and the context one-hots.
    size_t nIn = nf + nanIdx_.size();
    for (const auto& [k, vs] : context_)
    {
        nIn += vs.size();
    }
    for (const auto& mem : members_)
    {
        auto width = static_cast<Eigen::Index>(nIn);
        for (const auto& ly : mem)
        {
            if (ly.linear)
            {
                ASSERTMSG_(
                    ly.W.cols() == width,
                    "LearnedCovariance: a linear layer does not match its input width");
                width = ly.W.rows();
            }
            else
            {
                ASSERTMSG_(
                    ly.w.size() == width,
                    "LearnedCovariance: a layernorm does not match its input width");
            }
        }
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
    ASSERTMSG_(
        out.Sigma.allFinite() && std::isfinite(out.pFail),
        "LearnedCovariance: non-finite prediction");
    return out;
}
