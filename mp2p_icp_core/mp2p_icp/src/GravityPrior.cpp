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
 * @file   GravityPrior.cpp
 * @brief  Yaw-free, rank-2 gravity ("verticality") observation for solvers
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/GravityPrior.h>

namespace mrpt::serialization
{
CArchive& operator<<(CArchive& out, const mp2p_icp::GravityPrior& obj)
{
    out.WriteAs<uint8_t>(0);  // serialization version
    out << obj.up_body << obj.up_map << obj.sigma_rad;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::GravityPrior& obj)
{
    const auto version = in.ReadAs<uint8_t>();
    switch (version)
    {
        case 0:
            in >> obj.up_body >> obj.up_map >> obj.sigma_rad;
            break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    }
    return in;
}

}  // namespace mrpt::serialization
