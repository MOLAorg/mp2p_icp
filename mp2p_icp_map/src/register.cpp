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
 * @file   register.cpp
 * @brief  RTTI registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/core/initializer.h>

/** \defgroup mp2p_icp_map_grp mp2p_icp_map library
 * Core data structures and utilities for mp2p_icp
 *
 */

MRPT_INITIALIZER(register_mp2p_icp_map)
{
    using mrpt::rtti::registerClass;

    registerClass(CLASS_ID(mp2p_icp::metric_map_t));
}
