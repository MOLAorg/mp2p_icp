/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
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
        << minAbsStep_rot << pt2pl_layer;

    MRPT_TODO("serialize pairingsWeightParameters");
}
void Parameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> maxIterations >> maxPairsPerLayer >> minAbsStep_trans >>
                minAbsStep_rot >> pt2pl_layer;

            MRPT_TODO("deserialize pairingsWeightParameters");
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

void Parameters::loadFrom(const mrpt::containers::Parameters& p)
{
    MCP_LOAD_REQ(p, maxIterations);
    MCP_LOAD_REQ(p, maxPairsPerLayer);
    MCP_LOAD_OPT(p, minAbsStep_trans);
    MCP_LOAD_OPT(p, minAbsStep_rot);

    if (p.has("pairingsWeightParameters"))
        pairingsWeightParameters.loadFrom(p["pairingsWeightParameters"]);
}
void Parameters::saveTo(mrpt::containers::Parameters& p) const
{
    MCP_SAVE(p, maxIterations);
    MCP_SAVE(p, maxPairsPerLayer);
    MCP_SAVE(p, minAbsStep_trans);
    MCP_SAVE(p, minAbsStep_rot);

    auto pp = mrpt::containers::Parameters::Map();
    pairingsWeightParameters.saveTo(pp);
    p["pairingsWeightParameters"] = std::move(pp);
}
