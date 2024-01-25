/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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
uint8_t Parameters::serializeGetVersion() const { return 2; }
void    Parameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << maxIterations << minAbsStep_trans << minAbsStep_rot;
    out << generateDebugFiles << debugFileNameFormat;
    out << debugPrintIterationProgress;
    out << decimationDebugFiles;
    out << saveIterationDetails << decimationIterationDetails;  // v2
}
void Parameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    *this = Parameters();

    switch (version)
    {
        case 0:
        case 1:
        case 2:
        {
            in >> maxIterations >> minAbsStep_trans >> minAbsStep_rot;
            in >> generateDebugFiles >> debugFileNameFormat;
            in >> debugPrintIterationProgress;
            if (version >= 1) in >> decimationDebugFiles;
            if (version >= 2)
                in >> saveIterationDetails >> decimationIterationDetails;
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
}
