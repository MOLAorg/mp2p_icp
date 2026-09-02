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

/** Relative weight of points, lines, and planes.
 *
 * \note Solvers read these differently, and the difference matters:
 *
 * - Solver_OLAE normalizes them to sum to unity, so only their ratios are
 *   meaningful there and any convenient scale will do.
 *
 * - Solver_GaussNewton uses them as given. In a weighted least-squares sum a
 *   weight IS an inverse variance: a pairing contributes `w * J^t * J` to the
 *   normal equations, and its robust-kernel argument is whitened by the same
 *   `w`. So there `w = 1/sigma^2` for the assumed residual sigma of that pair
 *   type, and the ABSOLUTE scale matters, not just the ratios.
 *
 * The whitening is what puts every residual type on one scale before the robust
 * kernel compares it against `robustKernelParam`. Cov-to-cov pairings need no
 * weight for this: their residual is a Mahalanobis norm and is already whitened.
 *
 * This all becomes load-bearing when one ICP pipeline mixes pair types. A metric
 * point-to-plane residual and a dimensionless cov-to-cov one are not comparable
 * at equal weight: leaving both at 1.0 lets the cov-to-cov block dominate the
 * normal equations by the inverse of its own surface regularization, and leaves
 * the geometric block with a robust kernel that never fires.
 */
struct PairWeights
{
    double pt2pt = 1.0;  //!< Weight of point-to-point pairs

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
