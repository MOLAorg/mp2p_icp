/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PairWeights.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/PairWeights.h>
#include <mrpt/serialization/CArchive.h>

using namespace mp2p_icp;

void PairWeights::load_from(const mrpt::containers::yaml& p)
{
    MCP_LOAD_REQ(p, pt2pt);
    MCP_LOAD_REQ(p, pt2pl);
    MCP_LOAD_REQ(p, pt2ln);

    MCP_LOAD_REQ(p, ln2ln);
    MCP_LOAD_REQ(p, pl2pl);
}

void PairWeights::save_to(mrpt::containers::yaml& p) const
{
    MCP_SAVE(p, pt2pt);
    MCP_SAVE(p, pt2pl);
    MCP_SAVE(p, pt2ln);

    MCP_SAVE(p, ln2ln);
    MCP_SAVE(p, pl2pl);
}

void PairWeights::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << pt2pt << pt2pl << pt2ln << ln2ln << pl2pl;
}
void PairWeights::serializeFrom(mrpt::serialization::CArchive& in)
{
    in >> pt2pt >> pt2pl >> pt2ln >> ln2ln >> pl2pl;
}