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

#include <mp2p_icp/point_with_cov_pair_t.h>

#include <sstream>

std::string mp2p_icp::point_with_cov_pair_t::asString() const
{
    std::ostringstream oss;
    oss << "point_with_cov_pair_t:\n";
    oss << "  global: " << global << "\n";
    oss << "  local : " << local << "\n";
    oss << "  global_idx: " << global_idx << "\n";
    oss << "  local_idx : " << local_idx << "\n";
    oss << "  cov_inv:\n" << cov_inv << "\n";
    return oss.str();
}