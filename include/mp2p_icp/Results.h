/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <cstdint>
#include "IterTermReason.h"

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct Results
{
    /** The found value (mean + covariance) of the optimal transformation of
     * m2 wrt m1. */
    mrpt::poses::CPose3DPDFGaussian optimal_tf;

    /** Found scale (if supported by the underlying algorithm) for optimal_tf */
    double optimal_scale{1.0};

    /** The number of executed iterations until convergence */
    size_t nIterations{0};

    IterTermReason terminationReason{IterTermReason::Undefined};

    /** A measure of the 'quality' of the matching.
     * Different modules are available to evaluate the quality in ICP_Base.
     */
    double quality{0};

    /** A copy of the pairings found in the last ICP iteration. */
    Pairings finalPairings;
};
/** @} */

}  // namespace mp2p_icp
