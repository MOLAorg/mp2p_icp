/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
#pragma once

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <cstdint>
#include "IterTermReason.h"

namespace mp2p_icp
{
struct Results
{
    /** The found value (mean + covariance) of the optimal transformation of
     * m2 wrt m1. */
    mrpt::poses::CPose3DPDFGaussian optimal_tf;

    /** The number of executed iterations until convergence */
    size_t nIterations{0};

    IterTermReason terminationReason{IterTermReason::Undefined};

    /** A goodness measure for the alignment, it is a [0,1] range indicator
     * of percentage of correspondences. */
    double goodness{0};

    /** A measure of the 'quality' of the local minimum of the sqr. error
     * found by the method. Higher values are better. Low values will be
     * found in ill-conditioned situations (e.g. a corridor) */
    double quality{0};
};

}  // namespace mp2p_icp
