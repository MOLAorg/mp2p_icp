/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   covariance.h
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */
#pragma once

#include <mp2p_icp/Pairings.h>

namespace mp2p_icp
{
struct CovarianceParameters
{
    // Finite difference deltas:
    double finDif_xyz    = 1e-7;
    double finDif_angles = 1e-7;
};

/** Covariance estimation methods for an ICP result.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 covariance(
    const Pairings&             finalPairings,
    const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& p);

}  // namespace mp2p_icp
