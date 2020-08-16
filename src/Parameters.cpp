/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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
    out << maxIterations << maxPairsPerLayer << minAbsStep_trans
        << minAbsStep_rot << pairingsWeightParameters;
}
void Parameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> maxIterations >> maxPairsPerLayer >> minAbsStep_trans >>
                minAbsStep_rot >> pairingsWeightParameters;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void Parameters::load_from(const mrpt::containers::yaml& p)
{
    MCP_LOAD_REQ(p, maxIterations);
    MCP_LOAD_REQ(p, maxPairsPerLayer);
    MCP_LOAD_OPT(p, minAbsStep_trans);
    MCP_LOAD_OPT(p, minAbsStep_rot);

    if (p.has("pairingsWeightParameters"))
        pairingsWeightParameters.load_from(p["pairingsWeightParameters"]);
}
void Parameters::save_to(mrpt::containers::yaml& p) const
{
    MCP_SAVE(p, maxIterations);
    MCP_SAVE(p, maxPairsPerLayer);
    MCP_SAVE(p, minAbsStep_trans);
    MCP_SAVE(p, minAbsStep_rot);

     mrpt::containers::yaml pp = mrpt::containers::yaml::Map();
    pairingsWeightParameters.save_to(pp);
    p["pairingsWeightParameters"] = std::move(pp);
}
