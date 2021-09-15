/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/Parameters.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

IMPLEMENTS_MRPT_OBJECT(Parameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t Parameters::serializeGetVersion() const { return 0; }
void    Parameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << maxIterations << minAbsStep_trans << minAbsStep_rot;
    out << generateDebugFiles << debugFileNameFormat;
}
void Parameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    *this = Parameters();

    switch (version)
    {
        case 0:
        {
            in >> maxIterations >> minAbsStep_trans >> minAbsStep_rot;
            in >> generateDebugFiles >> debugFileNameFormat;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void Parameters::load_from(const mrpt::containers::yaml& p)
{
    MCP_LOAD_REQ(p, maxIterations);
    MCP_LOAD_OPT(p, minAbsStep_trans);
    MCP_LOAD_OPT(p, minAbsStep_rot);
    MCP_LOAD_OPT(p, generateDebugFiles);
    MCP_LOAD_OPT(p, debugFileNameFormat);
}
void Parameters::save_to(mrpt::containers::yaml& p) const
{
    MCP_SAVE(p, maxIterations);
    MCP_SAVE(p, minAbsStep_trans);
    MCP_SAVE(p, minAbsStep_rot);
    MCP_SAVE(p, generateDebugFiles);
    MCP_SAVE(p, debugFileNameFormat);
}
