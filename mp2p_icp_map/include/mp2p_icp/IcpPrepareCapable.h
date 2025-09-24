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
 * @file   IcpPrepareCapable.h
 * @brief  Virtual interface for maps needing getting ready for ICP.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2025
 */
#pragma once

#include <mrpt/math/TBoundingBox.h>
#include <mrpt/poses/CPose3D.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** Virtual interface for maps needing getting ready for ICP */
class IcpPrepareCapable
{
   public:
    IcpPrepareCapable() = default;
    virtual ~IcpPrepareCapable();

    IcpPrepareCapable(const IcpPrepareCapable&)            = default;
    IcpPrepareCapable& operator=(const IcpPrepareCapable&) = default;
    IcpPrepareCapable(IcpPrepareCapable&&)                 = default;
    IcpPrepareCapable& operator=(IcpPrepareCapable&&)      = default;

    /** Prepare the global map for ICP with a given pose as the current ICP estimate for local map
     * This will be called only once per whole ICP run.
     */
    virtual void icp_get_prepared_as_global(
        const mrpt::poses::CPose3D&                     icp_ref_point,
        const std::optional<mrpt::math::TBoundingBoxf>& local_map_roi = std::nullopt) const = 0;

    /// Optionally, clean up after ICP is done.
    virtual void icp_cleanup() const {}
};

/** @} */

}  // namespace mp2p_icp
