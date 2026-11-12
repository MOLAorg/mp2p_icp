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
// A library built against the C++ SDK of the mp2p-icp wheel, with the MOLA
// CMake functions.
#pragma once

#include <mp2p_icp/metricmap.h>

#include <memory>

namespace sdk_consumer
{
/** A map with a "raw" point cloud layer of `n` points along the X axis */
std::shared_ptr<mp2p_icp::metric_map_t> make_map(size_t n);

/** Number of points in all the layers of the map */
size_t count_points(const mp2p_icp::metric_map_t& m);

}  // namespace sdk_consumer
