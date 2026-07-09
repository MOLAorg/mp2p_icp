/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
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
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/version.h>

bool mp2p_icp::pointcloud_sanity_check(const mrpt::maps::CPointsMap& pc, bool printWarnings)
{
    bool         ok = true;
    const size_t n  = pc.size();

    for (const auto& field : pc.getPointFieldNames_float())
    {
        const auto& vec = pc.getPointsBufferRef_float_field(field);
        if (vec && !vec->empty() && vec->size() != n)
        {
            ok = false;
            if (printWarnings)
            {
                std::cerr << "[mp2p_icp] CPointsMap WARNING: Float field '" << field
                          << "' has incorrect length=" << vec->size() << " expected=" << n
                          << std::endl;
            }
        }
    }
    for (const auto& field : pc.getPointFieldNames_uint16())
    {
        const auto& vec = pc.getPointsBufferRef_uint16_field(field);
        if (vec && !vec->empty() && vec->size() != n)
        {
            ok = false;
            if (printWarnings)
            {
                std::cerr << "[mp2p_icp] CPointsMap WARNING: uint16 field '" << field
                          << "' has incorrect length=" << vec->size() << " expected=" << n
                          << std::endl;
            }
        }
    }

    for (const auto& field : pc.getPointFieldNames_double())
    {
        const auto& vec = pc.getPointsBufferRef_double_field(field);
        if (vec && !vec->empty() && vec->size() != n)
        {
            ok = false;
            if (printWarnings)
            {
                std::cerr << "[mp2p_icp] CPointsMap WARNING: Double field '" << field
                          << "' has incorrect length=" << vec->size() << " expected=" << n
                          << std::endl;
            }
        }
    }
    for (const auto& field : pc.getPointFieldNames_uint8())
    {
        const auto& vec = pc.getPointsBufferRef_uint8_field(field);
        if (vec && !vec->empty() && vec->size() != n)
        {
            ok = false;
            if (printWarnings)
            {
                std::cerr << "[mp2p_icp] CPointsMap WARNING: uint8 field '" << field
                          << "' has incorrect length=" << vec->size() << " expected=" << n
                          << std::endl;
            }
        }
    }

    return ok;
}
