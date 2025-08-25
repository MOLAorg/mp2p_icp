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
 * @file   load_xyz_file.h
 * @brief  Unit tests common utilities
 * @author Jose Luis Blanco Claraco
 * @date   July 11, 2020
 */
#pragma once

#include <mrpt/maps/CSimplePointsMap.h>

#include <string>

namespace mp2p_icp
{
/** Loads a pointcloud from an ASCII "XYZ file", storing an Nx3 matrix (each row
 * is a point). If the filename extension ends in ".gz", it is uncompressed
 * automatically.
 *
 * \ingroup mp2p_icp_map_grp
 */
mrpt::maps::CSimplePointsMap::Ptr load_xyz_file(const std::string& fil);

}  // namespace mp2p_icp