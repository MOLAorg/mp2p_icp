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
 * @file   IncrementCovarianceModel.cpp
 * @brief  Learned covariance of the pose increments of a LiDAR odometry.
 */

#include <mp2p_icp/IncrementCovarianceModel.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SO.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <map>

using namespace mp2p_icp;

namespace
{
constexpr int kFormatVersion = 1;

std::vector<double> numbers(const mrpt::containers::yaml::node_t& n, const char* what)
{
    ASSERTMSG_(
        n.isSequence(), mrpt::format("IncrementCovarianceModel: '%s' must be a sequence", what));
    std::vector<double> v;
    for (const auto& e : n.asSequence())
    {
        v.push_back(e.as<double>());
    }
    return v;
}

double scalar(const mrpt::containers::yaml& d, const char* key)
{
    ASSERTMSG_(d.has(key), mrpt::format("IncrementCovarianceModel: missing '%s'", key));
    return d[key].as<double>();
}
}  // namespace

void IncrementCovarianceModel::load(const std::string& yamlFile)
{
    load(mrpt::containers::yaml::FromFile(yamlFile));
}

void IncrementCovarianceModel::load(const mrpt::containers::yaml& doc)
{
    ASSERTMSG_(
        doc.has("format") && doc["format"].as<std::string>() == "mp2p_icp.IncrementCovarianceModel",
        "IncrementCovarianceModel: not a model file of this class ('format')");
    ASSERTMSG_(
        doc.has("version") && doc["version"].as<int>() == kFormatVersion,
        "IncrementCovarianceModel: unsupported 'version'");

    // Parse everything into a new model first, so a malformed file leaves the
    // current one untouched:
    IncrementCovarianceModel m;
    m.description_ = doc.has("description") ? doc["description"].as<std::string>() : "";

    ASSERTMSG_(doc.has("output"), "IncrementCovarianceModel: missing 'output'");
    const mrpt::containers::yaml out(doc["output"].node());
    ASSERTMSG_(
        out.has("type") && out["type"].as<std::string>() == "per_axis_information_eigenbasis",
        "IncrementCovarianceModel: unsupported 'output.type'");
    m.rotLength_    = scalar(out, "rotation_length_scale");
    m.floorTrans_   = scalar(out, "floor_translation_var");
    m.floorRot_     = scalar(out, "floor_rotation_var");
    m.rotVarPerRad_ = scalar(out, "rotation_var_per_rad");
    ASSERTMSG_(m.rotLength_ > 0, "IncrementCovarianceModel: rotation_length_scale must be > 0");
    if (out.has("z_clip"))
    {
        const auto zc = numbers(out["z_clip"].node(), "output.z_clip");
        ASSERTMSG_(zc.size() == 2 && zc[0] < zc[1], "IncrementCovarianceModel: bad 'z_clip'");
        m.zMin_ = zc[0];
        m.zMax_ = zc[1];
    }

    static const std::map<std::string, Feature> kFeatures = {
        {"log_lambda", Feature::LogLambda},  {"translational_share", Feature::TranslationalShare},
        {"z_share", Feature::ZShare},        {"yaw_share", Feature::YawShare},
        {"iterations", Feature::Iterations}, {"quality", Feature::Quality},
        {"speed", Feature::Speed},           {"turn_rate", Feature::TurnRate}};
    ASSERTMSG_(doc.has("features"), "IncrementCovarianceModel: missing 'features'");
    const auto featureSeq = doc["features"].asSequence();
    for (const auto& f : featureSeq)
    {
        const auto name = f.as<std::string>();
        const auto it   = kFeatures.find(name);
        ASSERTMSG_(
            it != kFeatures.end(),
            mrpt::format("IncrementCovarianceModel: unknown feature '%s'", name.c_str()));
        m.featureNames_.push_back(name);
        m.features_.push_back(it->second);
    }
    const auto nf = m.features_.size();
    ASSERTMSG_(nf > 0, "IncrementCovarianceModel: no features");

    const auto mean = numbers(doc["feature_mean"].node(), "feature_mean");
    const auto sd   = numbers(doc["feature_std"].node(), "feature_std");
    ASSERTMSG_(
        mean.size() == nf && sd.size() == nf,
        "IncrementCovarianceModel: feature_mean/feature_std sizes differ from 'features'");
    m.mean_ = Eigen::Map<const Eigen::VectorXd>(mean.data(), static_cast<Eigen::Index>(nf));
    m.std_  = Eigen::Map<const Eigen::VectorXd>(sd.data(), static_cast<Eigen::Index>(nf));
    ASSERTMSG_((m.std_.array() > 0).all(), "IncrementCovarianceModel: feature_std must be > 0");

    ASSERTMSG_(doc.has("layers"), "IncrementCovarianceModel: missing 'layers'");
    Eigen::Index nIn      = static_cast<Eigen::Index>(nf);
    const auto   layerSeq = doc["layers"].asSequence();
    for (const auto& ln : layerSeq)
    {
        const mrpt::containers::yaml l(ln);
        Layer                        L;
        const auto act = l.has("activation") ? l["activation"].as<std::string>() : "linear";
        ASSERTMSG_(
            act == "linear" || act == "tanh",
            mrpt::format("IncrementCovarianceModel: unknown activation '%s'", act.c_str()));
        L.act = act == "tanh" ? Activation::Tanh : Activation::Linear;

        const auto rows = l["weights"].asSequence();
        const auto bias = numbers(l["bias"].node(), "layers.bias");
        ASSERTMSG_(
            !rows.empty() && rows.size() == bias.size(),
            "IncrementCovarianceModel: a layer's weights and bias have different output sizes");
        L.W.resize(static_cast<Eigen::Index>(rows.size()), nIn);
        for (size_t r = 0; r < rows.size(); r++)
        {
            const auto row = numbers(rows[r], "layers.weights row");
            ASSERTMSG_(
                static_cast<Eigen::Index>(row.size()) == nIn,
                "IncrementCovarianceModel: a layer's input size does not match the previous one");
            for (size_t c = 0; c < row.size(); c++)
            {
                L.W(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = row[c];
            }
        }
        L.b =
            Eigen::Map<const Eigen::VectorXd>(bias.data(), static_cast<Eigen::Index>(bias.size()));
        nIn = L.W.rows();
        m.layers_.push_back(std::move(L));
    }
    ASSERTMSG_(!m.layers_.empty(), "IncrementCovarianceModel: no layers");
    ASSERTMSG_(nIn == 1, "IncrementCovarianceModel: the last layer must have one output");

    *this = std::move(m);
}

mrpt::math::CMatrixDouble66 IncrementCovarianceModel::predict(const Input& in) const
{
    ASSERTMSG_(loaded(), "IncrementCovarianceModel: no model loaded");

    // Information with rotations scaled by L: variables (t, L * theta).
    Eigen::Matrix<double, 6, 1> sInv;
    sInv << 1, 1, 1, 1.0 / rotLength_, 1.0 / rotLength_, 1.0 / rotLength_;
    const Eigen::Matrix<double, 6, 6> H  = in.measurement_information.asEigen();
    Eigen::Matrix<double, 6, 6>       Hs = sInv.asDiagonal() * H * sInv.asDiagonal();
    Hs                                   = 0.5 * (Hs + Hs.transpose()).eval();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(Hs);
    const Eigen::Matrix<double, 6, 1> lambda = eig.eigenvalues();  // ascending
    const Eigen::Matrix<double, 6, 6> V      = eig.eigenvectors();

    const double dist = in.increment.translation().norm();
    const double dang =
        mrpt::poses::Lie::SO<3>::log(in.increment.getRotationMatrix()).asEigen().norm();
    const double dt  = std::max(in.dt, 0.05);
    const double ell = dist + rotLength_ * dang;

    Eigen::VectorXd             x(static_cast<Eigen::Index>(features_.size()));
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; i++)
    {
        const auto v = V.col(i);
        for (size_t f = 0; f < features_.size(); f++)
        {
            double val = 0;
            switch (features_[f])
            {
                case Feature::LogLambda:
                    val = std::log(std::max(lambda[i], 1e-9));
                    break;
                case Feature::TranslationalShare:
                    val = v.head<3>().squaredNorm();
                    break;
                case Feature::ZShare:
                    val = v[2] * v[2];
                    break;
                case Feature::YawShare:
                    val = v[5] * v[5];
                    break;
                case Feature::Iterations:
                    val = static_cast<double>(in.iterations);
                    break;
                case Feature::Quality:
                    val = in.quality;
                    break;
                case Feature::Speed:
                    val = std::min(dist / dt, 2.0);
                    break;
                case Feature::TurnRate:
                    val = std::min(dang / dt, 1.0);
                    break;
            }
            x[static_cast<Eigen::Index>(f)] = val;
        }
        Eigen::VectorXd a = ((x - mean_).array() / std_.array()).matrix();
        for (const auto& L : layers_)
        {
            a = L.W * a + L.b;
            if (L.act == Activation::Tanh)
            {
                a = a.array().tanh().matrix();
            }
        }
        const double z = std::clamp(a[0], zMin_, zMax_);
        M += (ell * std::exp(z)) * (v * v.transpose());
    }

    Eigen::Matrix<double, 6, 6> Q = sInv.asDiagonal() * M * sInv.asDiagonal();
    for (int i = 0; i < 3; i++)
    {
        Q(i, i) += floorTrans_;
        Q(i + 3, i + 3) += floorRot_ + rotVarPerRad_ * dang;
    }
    mrpt::math::CMatrixDouble66 out;
    out.asEigen() = 0.5 * (Q + Q.transpose());
    return out;
}
