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

#include <mp2p_icp/NearestPlaneCapable.h>

#include <cmath>

using namespace mp2p_icp;

NearestPlaneCapable::~NearestPlaneCapable() = default;

void NearestPlaneCapable::nn_visit_pt2pl_candidates(
    const mrpt::math::TPoint3Df& point, const float max_search_distance,
    const plane_candidate_visitor_t& visitor) const
{
    // Generic fallback: a map that can only report its best match contributes
    // exactly that one candidate.
    const NearestPlaneResult r = nn_search_pt2pl(point, max_search_distance);
    if (!r.pairing)
    {
        return;
    }

    PlaneCandidate c;
    c.pairing  = *r.pairing;
    c.distance = r.distance;

    const auto&  ctr = r.pairing->pl_global.centroid;
    const double dx  = ctr.x - point.x;
    const double dy  = ctr.y - point.y;
    const double dz  = ctr.z - point.z;

    c.centroidDistance = static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));

    visitor(c);
}
