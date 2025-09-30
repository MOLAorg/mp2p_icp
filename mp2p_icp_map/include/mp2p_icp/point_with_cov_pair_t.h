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
 * @file   point_with_cov_pair_t.h
 * @brief  Defines point_with_cov_pair_t
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2025
 */
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/typemeta/TTypeName.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** point-with-cov correspondences */
struct point_with_cov_pair_t
{
    mrpt::math::TPoint3Df global;
    mrpt::math::TPoint3Df local;
    uint32_t              global_idx = std::numeric_limits<uint32_t>::max();
    uint32_t              local_idx  = std::numeric_limits<uint32_t>::max();

    /** The Mahalanobis distance weight matrix.
     * Following GICP \cite segal2009gicp this should be:
     *  `(COV_{global} + R*COV_{local}*R^T)^{-1}`
     */
    mrpt::math::CMatrixFloat33 cov_inv;

    /** Retuns a printable description of the structure, mostly for debugging */
    std::string asString() const;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::point_with_cov_pair_t)
};

using MatchedPointWithCovList = std::vector<point_with_cov_pair_t>;

/** @} */

}  // namespace mp2p_icp
