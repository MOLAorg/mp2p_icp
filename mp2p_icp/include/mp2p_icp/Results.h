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
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cstdint>
#include <iosfwd>

#include "IterTermReason.h"

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct Results
{
    /** The found value (mean + covariance) of the optimal transformation of
     * "local" wrt "global". */
    mrpt::poses::CPose3DPDFGaussian optimal_tf;

    /** Found scale (if supported by the underlying algorithm) for optimal_tf */
    double optimalScale = 1.0;

    /** The number of executed iterations until convergence */
    size_t nIterations = 0;

    IterTermReason terminationReason{IterTermReason::Undefined};

    /** A measure of the 'quality' of the matching.
     * Different modules are available to evaluate the quality in ICP_Base.
     */
    double quality = 0;

    /** A copy of the pairings found in the last ICP iteration. */
    Pairings finalPairings;

    void serializeTo(mrpt::serialization::CArchive& out) const;
    void serializeFrom(mrpt::serialization::CArchive& in);

    /** Print all results in human-friendly format to the given output, for
     * example, use `std::cout`.
     */
    void print(std::ostream& o) const;
};

mrpt::serialization::CArchive& operator<<(mrpt::serialization::CArchive& out, const Results& obj);

mrpt::serialization::CArchive& operator>>(mrpt::serialization::CArchive& in, Results& obj);

/** @} */

}  // namespace mp2p_icp
