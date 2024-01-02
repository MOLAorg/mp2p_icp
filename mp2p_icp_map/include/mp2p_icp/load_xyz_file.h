/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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