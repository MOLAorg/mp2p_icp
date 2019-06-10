/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

namespace mp2p_icp
{
/** Reason of iterating termination */
enum class IterTermReason
{
    Undefined = 0,
    NoPairings,
    MaxIterations,
    Stalled
};

}  // namespace mp2p_icp
