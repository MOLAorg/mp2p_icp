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
 * @file   MetricMapMergeCapable.cpp
 * @brief  Virtual interface for map layers capable of being merged with others
 * @author Jose Luis Blanco Claraco
 * @date   Sep 24, 2025
 */

#include <mp2p_icp/MetricMapMergeCapable.h>

namespace mp2p_icp
{
MetricMapMergeCapable::~MetricMapMergeCapable() = default;

}  // namespace mp2p_icp
