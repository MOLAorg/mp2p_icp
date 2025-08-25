/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
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