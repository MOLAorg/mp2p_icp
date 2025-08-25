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
 * @file   PairWeights.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/serialization/serialization_frwds.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** Relative weight of points, lines, and planes. They will be automatically
 * normalized to sum the unity, so feel free of setting weights at any
 * convenient scale.
 */
struct PairWeights
{
    /** Weight of point-to-point pairs. Note that finer control of weights
     * can be achieved with `Pairings::point_weights`, so this `pt2pt` field
     * will be honored only if `Pairings::point_weights` is empty.
     */
    double pt2pt = 1.0;

    double pt2ln = 1.0;  //!< Weight of point-to-line pairs
    double pt2pl = 1.0;  //!< Weight of point-to-plane pairs

    double ln2ln = 1.0;  //!< Weight of line-to-line pairs
    double pl2pl = 1.0;  //!< Weight of plane-to-plane pairs

    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;
    void serializeTo(mrpt::serialization::CArchive& out) const;
    void serializeFrom(mrpt::serialization::CArchive& in);
};

/** @} */

}  // namespace mp2p_icp
