/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WeightParameters.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/WeightParameters.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>

IMPLEMENTS_MRPT_OBJECT(
    WeightParameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t WeightParameters::serializeGetVersion() const { return 0; }
void    WeightParameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << use_scale_outlier_detector << scale_outlier_threshold
        << use_robust_kernel << currentEstimateForRobust << robust_kernel_param
        << robust_kernel_scale;
    pair_weights.serializeTo(out);
}
void WeightParameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> use_scale_outlier_detector >> scale_outlier_threshold >>
                use_robust_kernel >> currentEstimateForRobust >>
                robust_kernel_param >> robust_kernel_scale;
            pair_weights.serializeFrom(in);
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void WeightParameters::load_from(const mrpt::containers::Parameters& p)
{
    MCP_LOAD_REQ(p, use_scale_outlier_detector);
    MCP_LOAD_OPT(p, scale_outlier_threshold);

    MCP_LOAD_REQ(p, use_robust_kernel);
    MCP_LOAD_OPT_DEG(p, robust_kernel_param);
    MCP_LOAD_OPT(p, robust_kernel_scale);

    if (p.has("pair_weights")) pair_weights.load_from(p["pair_weights"]);
}
void WeightParameters::save_to(mrpt::containers::Parameters& p) const
{
    MCP_SAVE(p, use_scale_outlier_detector);
    MCP_SAVE(p, scale_outlier_threshold);

    MCP_SAVE(p, use_robust_kernel);
    MCP_SAVE_DEG(p, robust_kernel_param);
    MCP_SAVE(p, robust_kernel_scale);

    auto a = mrpt::containers::Parameters::Map();
    pair_weights.save_to(a);
    p["pair_weights"] = a;
}

void WeightParameters::PairWeights::load_from(
    const mrpt::containers::Parameters& p)
{
    MCP_LOAD_REQ(p, pt2pt);
    MCP_LOAD_REQ(p, pt2pl);
    MCP_LOAD_REQ(p, pt2ln);

    MCP_LOAD_REQ(p, ln2ln);
    MCP_LOAD_REQ(p, pl2pl);
}

void WeightParameters::PairWeights::save_to(
    mrpt::containers::Parameters& p) const
{
    MCP_SAVE(p, pt2pt);
    MCP_SAVE(p, pt2pl);
    MCP_SAVE(p, pt2ln);

    MCP_SAVE(p, ln2ln);
    MCP_SAVE(p, pl2pl);
}

void WeightParameters::PairWeights::serializeTo(
    mrpt::serialization::CArchive& out) const
{
    out << pt2pt << pt2pl << pt2ln << ln2ln << pl2pl;
}
void WeightParameters::PairWeights::serializeFrom(
    mrpt::serialization::CArchive& in)
{
    in >> pt2pt >> pt2pl >> pt2ln >> ln2ln >> pl2pl;
}
