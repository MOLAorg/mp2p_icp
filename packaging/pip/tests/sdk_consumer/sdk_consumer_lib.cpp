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

#include <mrpt/maps/CSimplePointsMap.h>
#include <sdk_consumer_lib.h>

namespace sdk_consumer
{
std::shared_ptr<mp2p_icp::metric_map_t> make_map(size_t n)
{
    auto m   = std::make_shared<mp2p_icp::metric_map_t>();
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (size_t i = 0; i < n; i++)
    {
        pts->insertPoint(static_cast<float>(i), 0.0f, 0.0f);
    }
    m->layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = pts;
    return m;
}

size_t count_points(const mp2p_icp::metric_map_t& m) { return m.size(); }

}  // namespace sdk_consumer
