/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mp2p_icp
{
/** Reason of iterating termination.
 * \ingroup mp2p_icp_grp
 */
enum class IterTermReason : uint8_t
{
    Undefined = 0,
    NoPairings,
    SolverError,
    MaxIterations,
    Stalled
};

}  // namespace mp2p_icp

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp, mp2p_icp::IterTermReason)
MRPT_FILL_ENUM(IterTermReason::Undefined);
MRPT_FILL_ENUM(IterTermReason::NoPairings);
MRPT_FILL_ENUM(IterTermReason::SolverError);
MRPT_FILL_ENUM(IterTermReason::MaxIterations);
MRPT_FILL_ENUM(IterTermReason::Stalled);
MRPT_ENUM_TYPE_END()
