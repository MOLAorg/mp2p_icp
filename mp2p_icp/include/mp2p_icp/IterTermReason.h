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
    Stalled,
    QualityCheckpointFailed,
    HookRequest
};

}  // namespace mp2p_icp

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp, mp2p_icp::IterTermReason)
MRPT_FILL_ENUM(IterTermReason::Undefined);
MRPT_FILL_ENUM(IterTermReason::NoPairings);
MRPT_FILL_ENUM(IterTermReason::SolverError);
MRPT_FILL_ENUM(IterTermReason::MaxIterations);
MRPT_FILL_ENUM(IterTermReason::Stalled);
MRPT_FILL_ENUM(IterTermReason::QualityCheckpointFailed);
MRPT_FILL_ENUM(IterTermReason::HookRequest);
MRPT_ENUM_TYPE_END()
