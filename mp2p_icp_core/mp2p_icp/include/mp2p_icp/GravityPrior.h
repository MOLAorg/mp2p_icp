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
 * @file   GravityPrior.h
 * @brief  Yaw-free, rank-2 gravity ("verticality") observation for solvers
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/math/TPoint3D.h>  // TVector3D
#include <mrpt/serialization/CArchive.h>
#include <mrpt/typemeta/TTypeName.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** A gravity ("verticality") observation for the optimal-transformation
 *  solvers.
 *
 * It constrains the two TILT degrees of freedom only, leaving rotation about
 * the gravity axis (yaw) and all three translations exactly free. Use it to
 * keep a solution gravity-aligned without the parameterization hazards of
 * encoding tilt into a 6x6 SE(3) prior information matrix: a diagonal there
 * only isolates roll/pitch when yaw is near zero, and it inevitably couples
 * into translation.
 *
 * The residual is the horizontal component of the predicted "up" vector,
 * \f[
 *    r(R) = B^\top \, (R \, u_{body}) \in \mathbb{R}^2
 * \f]
 * where \f$ B \f$ (3x2) is an orthonormal basis of the plane orthogonal to
 * `up_map`, weighted isotropically by \f$ 1/\sigma^2 \f$. \f$ r = 0 \f$ exactly
 * when \f$ R\,u_{body} \f$ is parallel to `up_map`, i.e. when the solution is
 * level; for small tilt \f$ \|r\| \f$ is the tilt angle in radians.
 *
 * Isotropic weighting is what makes the cost invariant to rotation about
 * gravity. The induced 6x6 information \f$ J^\top W J \f$ then has rank 2, a
 * zero translation block, and null space exactly \f$ span(u_{body}) \f$, i.e.
 * rotation about the gravity axis - so yaw stays free at ANY attitude, not
 * only near yaw=0.
 *
 * \ingroup mp2p_icp_grp
 */
struct GravityPrior
{
    GravityPrior() = default;

    /** Measured gravity "up" direction in the BODY/vehicle frame, e.g. the
     *  normalized accelerometer specific force while quasi-static. Need not be
     *  normalized by the caller. */
    mrpt::math::TVector3D up_body{0, 0, 1};

    /** Gravity "up" direction expressed in the MAP/global frame. Use (0,0,1)
     *  for a gravity-aligned map, or the direction captured at map origin if
     *  the map frame is not exactly level. Need not be normalized. */
    mrpt::math::TVector3D up_map{0, 0, 1};

    /** Standard deviation [rad] of the tilt observation. Smaller means a
     *  stronger, closer-to-mandatory leveling constraint. */
    double sigma_rad = 0.02;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::GravityPrior)
};

/** @} */

}  // namespace mp2p_icp

namespace mrpt::serialization
{
/** Serialization of mp2p_icp::GravityPrior, so that the verticality
 *  observation actually handed to the solver can be recovered from an
 *  `.icplog` afterwards. Without it the only way to know the effective
 *  `sigma_rad` (which `adaptive_sigma` changes at run time) is to re-run with
 *  debug-level logging enabled.
 */
CArchive& operator<<(CArchive& out, const mp2p_icp::GravityPrior& obj);
CArchive& operator>>(CArchive& in, mp2p_icp::GravityPrior& obj);

}  // namespace mrpt::serialization
