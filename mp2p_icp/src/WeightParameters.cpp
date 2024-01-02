/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(
    WeightParameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t WeightParameters::serializeGetVersion() const { return 1; }
void    WeightParameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << use_scale_outlier_detector << scale_outlier_threshold
        << robust_kernel << currentEstimateForRobust << robust_kernel_param;

    pair_weights.serializeTo(out);
}
void WeightParameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        case 1:
        {
            in >> use_scale_outlier_detector >> scale_outlier_threshold;

            if (version < 1)
            {
                bool dummy_use_robust_kernel;
                in >> dummy_use_robust_kernel;
            }
            else
                in >> robust_kernel;

            in >> currentEstimateForRobust >> robust_kernel_param;

            if (version < 1)
            {
                double dummy_robust_kernel_scale;
                in >> dummy_robust_kernel_scale;
            }
            pair_weights.serializeFrom(in);
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void WeightParameters::load_from(const mrpt::containers::yaml& p)
{
    MCP_LOAD_REQ(p, use_scale_outlier_detector);
    MCP_LOAD_OPT(p, scale_outlier_threshold);

    MCP_LOAD_REQ(p, robust_kernel);
    MCP_LOAD_OPT(p, robust_kernel_param);

    if (p.has("pair_weights")) pair_weights.load_from(p["pair_weights"]);
}
void WeightParameters::save_to(mrpt::containers::yaml& p) const
{
    MCP_SAVE(p, use_scale_outlier_detector);
    MCP_SAVE(p, scale_outlier_threshold);

#if MRPT_VERSION >= 0x020b03
    MCP_SAVE(p, robust_kernel);
#else
    MCP_SAVE(p, mrpt::typemeta::enum2str(robust_kernel));
#endif

    MCP_SAVE(p, robust_kernel_param);

    mrpt::containers::yaml a = mrpt::containers::yaml::Map();
    pair_weights.save_to(a);
    p["pair_weights"] = a;
}
