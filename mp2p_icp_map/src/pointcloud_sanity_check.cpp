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
 * @file   pointcloud_sanity_check.cpp
 * @brief  Checks for consistent length of field vectors.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 11, 2024
 */

#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>

bool mp2p_icp::pointcloud_sanity_check(const mrpt::maps::CPointsMap& pc, bool printWarnings)
{
    bool         ok = true;
    const size_t n  = pc.size();

    if (auto* pcIRT = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(&pc); pcIRT)
    {
        if (pcIRT->hasIntensityField() && pcIRT->getPointsBufferRef_intensity()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] XYZIRT WARNING: Intensity channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_intensity()->size() << " expected=" << n
                          << std::endl;
        }
        if (pcIRT->hasRingField() && pcIRT->getPointsBufferRef_ring()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] XYZIRT WARNING: Ring channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_ring()->size() << " expected=" << n
                          << std::endl;
        }
        if (pcIRT->hasTimeField() && pcIRT->getPointsBufferRef_timestamp()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] XYZIRT WARNING: Timestamp channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_timestamp()->size() << " expected=" << n
                          << std::endl;
        }
    }
    else if (auto* pcI = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(&pc); pcI)
    {
        if (pcI->getPointsBufferRef_intensity() && pcI->getPointsBufferRef_intensity()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] XYZI WARNING: Intensity channel has "
                             "incorrect length="
                          << pcI->getPointsBufferRef_intensity()->size() << " expected=" << n
                          << std::endl;
        }
    }
    return ok;
}
