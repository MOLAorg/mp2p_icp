/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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

bool mp2p_icp::pointcloud_sanity_check(
    const mrpt::maps::CPointsMap& pc, bool printWarnings)
{
    bool         ok = true;
    const size_t n  = pc.size();

    if (auto* pcIRT = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(&pc);
        pcIRT)
    {
        if (pcIRT->hasIntensityField() &&
            pcIRT->getPointsBufferRef_intensity()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] WARNING: Intensity channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_intensity()->size()
                          << " expected=" << n << std::endl;
        }
        if (pcIRT->hasRingField() &&
            pcIRT->getPointsBufferRef_ring()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] WARNING: Ring channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_ring()->size()
                          << " expected=" << n << std::endl;
        }
        if (pcIRT->hasTimeField() &&
            pcIRT->getPointsBufferRef_timestamp()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] WARNING: Timestamp channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_timestamp()->size()
                          << " expected=" << n << std::endl;
        }
    }
    else if (auto* pcI = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(&pc);
             pcI)
    {
        if (pcI->getPointsBufferRef_intensity() &&
            pcI->getPointsBufferRef_intensity()->size() != n)
        {
            ok = false;
            if (printWarnings)
                std::cerr << "[mp2p_icp] WARNING: Intensity channel has "
                             "incorrect length="
                          << pcIRT->getPointsBufferRef_intensity()->size()
                          << " expected=" << n << std::endl;
        }
    }
    return ok;
}
