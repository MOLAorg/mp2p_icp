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

#include <mp2p_icp/Parameters.h>
#include <mrpt/core/get_env.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

IMPLEMENTS_MRPT_OBJECT(Parameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// this env var can be used to enforce generating files despite the actual
// parameter files.
const bool MP2P_ICP_GENERATE_DEBUG_FILES =
    mrpt::get_env<bool>("MP2P_ICP_GENERATE_DEBUG_FILES", false);

// Implementation of the CSerializable virtual interface:
uint8_t Parameters::serializeGetVersion() const { return 3; }
void    Parameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << maxIterations << minAbsStep_trans << minAbsStep_rot;
    out << generateDebugFiles << debugFileNameFormat;
    out << debugPrintIterationProgress;
    out << decimationDebugFiles;
    out << saveIterationDetails << decimationIterationDetails;  // v2
    const uint8_t covMethod = static_cast<uint8_t>(covariance_params.method);
    out << covMethod << covariance_params.defaultPointSigma << covariance_params.floor_sigma_xyz
        << covariance_params.floor_sigma_angles << covariance_params.finDif_xyz
        << covariance_params.finDif_angles;  // v3
}
void Parameters::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
    *this = Parameters();

    switch (version)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        {
            in >> maxIterations >> minAbsStep_trans >> minAbsStep_rot;
            in >> generateDebugFiles >> debugFileNameFormat;
            in >> debugPrintIterationProgress;
            if (version >= 1)
            {
                in >> decimationDebugFiles;
            }
            if (version >= 2)
            {
                in >> saveIterationDetails >> decimationIterationDetails;
            }
            if (version >= 3)
            {
                uint8_t covMethod = 0;
                in >> covMethod >> covariance_params.defaultPointSigma >>
                    covariance_params.floor_sigma_xyz >> covariance_params.floor_sigma_angles >>
                    covariance_params.finDif_xyz >> covariance_params.finDif_angles;
                covariance_params.method = static_cast<CovarianceParameters::Method>(covMethod);
            }
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    generateDebugFiles = generateDebugFiles || MP2P_ICP_GENERATE_DEBUG_FILES;
}

void Parameters::load_from(const mrpt::containers::yaml& p)
{
    MCP_LOAD_REQ(p, maxIterations);
    MCP_LOAD_OPT(p, minAbsStep_trans);
    MCP_LOAD_OPT(p, minAbsStep_rot);
    MCP_LOAD_OPT(p, generateDebugFiles);
    MCP_LOAD_OPT(p, debugFileNameFormat);
    MCP_LOAD_OPT(p, debugPrintIterationProgress);
    MCP_LOAD_OPT(p, decimationDebugFiles);
    MCP_LOAD_OPT(p, saveIterationDetails);
    MCP_LOAD_OPT(p, decimationIterationDetails);

    if (p.has("quality_checkpoints"))
    {
        ASSERT_(
            p["quality_checkpoints"].isSequence() &&
            !p["quality_checkpoints"].asSequenceRange().empty());

        quality_checkpoints.clear();
        for (const auto& e : p["quality_checkpoints"].asSequenceRange())
        {
            ASSERTMSG_(
                e.isMap(),
                "Entries within 'quality_checkpoints' must be a Map. See "
                "mp2p_icp::Parameters docs.");

            quality_checkpoints.emplace(
                e.asMap().at("iteration").as<size_t>(),
                e.asMap().at("minimum_quality").as<double>());
        }
    }

    if (p.has("covariance"))
    {
        covariance_params.load_from(p["covariance"]);
    }

    generateDebugFiles = generateDebugFiles || MP2P_ICP_GENERATE_DEBUG_FILES;
}

void Parameters::save_to(mrpt::containers::yaml& p) const
{
    MCP_SAVE(p, maxIterations);
    MCP_SAVE(p, minAbsStep_trans);
    MCP_SAVE(p, minAbsStep_rot);
    MCP_SAVE(p, generateDebugFiles);
    MCP_SAVE(p, debugFileNameFormat);
    MCP_SAVE(p, debugPrintIterationProgress);
    MCP_SAVE(p, decimationDebugFiles);
    MCP_SAVE(p, saveIterationDetails);
    MCP_SAVE(p, decimationIterationDetails);

    mrpt::containers::yaml covYaml = mrpt::containers::yaml::Map();
    covariance_params.save_to(covYaml);
    p["covariance"] = covYaml;
}
