/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Point2Plane.cpp
 * @brief  Pointcloud matcher: point to plane-fit of nearby points
 * @author Jose Luis Blanco Claraco
 * @date   July 21, 2020
 */

#include <mp2p_icp/Matcher_Point2Plane.h>
#include <mp2p_icp/estimate_points_eigen.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Point2Plane, Matcher, mp2p_icp)

using namespace mp2p_icp;

Matcher_Point2Plane::Matcher_Point2Plane()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Point2Plane");
}

void Matcher_Point2Plane::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);
    DECLARE_PARAMETER_REQ(params, distanceThreshold);
}

void Matcher_Point2Plane::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName, const layer_name_t& localName,
    Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

    const mp2p_icp::NearestPlaneCapable& nnGlobal =
        *mp2p_icp::MapToNP(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax},
            distanceThreshold + bounding_box_intersection_check_epsilon_))
        return;

    // Prepare output: no correspondences initially:
    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    // Loop for each point in local map:
    // --------------------------------------------------
    // searchRadius

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
            continue;  // skip, already paired.

        // Don't discard **global** map points if already used by another
        // matcher, since the assumption of "plane" features implies that
        // many local points may match the *same* "global plane", so it's ok
        // to have multiple-local-to-one-global pairings.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map.
        const NearestPlaneCapable::NearestPlaneResult np =
            nnGlobal.nn_search_pt2pl({lx, ly, lz}, distanceThreshold);

        if (!np.pairing) continue;
        if (np.distance > distanceThreshold) continue;  // plane is too distant

        // OK, all conditions pass: add the new pairing:
        auto& p     = out.paired_pt2pl.emplace_back();
        p.pt_local  = {lxs[localIdx], lys[localIdx], lzs[localIdx]};
        p.pl_global = np.pairing->pl_global;

        // Mark local point as already paired:
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);

    }  // For each local point

    MRPT_END
}
