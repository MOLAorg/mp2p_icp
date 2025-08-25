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
 * @file   plane_patch.h
 * @brief  Defines plane_patch_t
 * @author Jose Luis Blanco Claraco
 * @date   Oct 17, 2021
 */
#pragma once

#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

struct plane_patch_t
{
    mrpt::math::TPlane   plane;
    mrpt::math::TPoint3D centroid;

    plane_patch_t() = default;
    plane_patch_t(const mrpt::math::TPlane3D& pl, const mrpt::math::TPoint3D& center)
        : plane(pl), centroid(center)
    {
    }
};

/** @} */

}  // namespace mp2p_icp
