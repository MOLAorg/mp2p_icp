/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <string>

/** Loads from XYZ file, possibly gz-compressed */
mrpt::maps::CSimplePointsMap::Ptr load_xyz_file(const std::string& fil);
