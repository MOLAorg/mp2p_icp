/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_InlierRatio.cpp
 * @brief  Pointcloud matcher: fixed ratio of inliers/outliers by distance
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */

#include <mp2p_icp/Matcher_Points_InlierRatio.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_InlierRatio, Matcher, mp2p_icp)

using namespace mp2p_icp;

Matcher_Points_InlierRatio::Matcher_Points_InlierRatio()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Points_InlierRatio");
}

void Matcher_Points_InlierRatio::initialize(
    const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    MCP_LOAD_REQ(params, inliersRatio);
}

void Matcher_Points_InlierRatio::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    ASSERT_GT_(inliersRatio, 0.0);
    ASSERT_LT_(inliersRatio, 1.0);

    const mrpt::maps::NearestNeighborsCapable& nnGlobal =
        *mp2p_icp::MapToNN(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax},
            bounding_box_intersection_check_epsilon_))
        return;

    // Loop for each point in local map:
    // --------------------------------------------------
    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    std::multimap<double, mrpt::tfest::TMatchingPair> sortedPairings;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
            continue;  // skip, already paired.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor of:
        //   (x_local, y_local, z_local)
        // In "this" (global/reference) points map.
        uint64_t              tentativeGlobalIdx = 0;
        float                 tentativeErrSqr    = 0;
        mrpt::math::TPoint3Df neighborPt;

        const bool searchOk = nnGlobal.nn_single_search(
            {lx, ly, lz},  // Look closest to this guy
            neighborPt, tentativeErrSqr, tentativeGlobalIdx);

        if (searchOk)
        {
            mrpt::tfest::TMatchingPair p;
            p.globalIdx = tentativeGlobalIdx;
            p.localIdx  = localIdx;
            p.global    = neighborPt;
            p.local     = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

            p.errorSquareAfterTransformation = tentativeErrSqr;

            // Sort by distance:
            sortedPairings.emplace_hint(
                sortedPairings.begin(), tentativeErrSqr, p);
        }
    }  // For each local point

    // Now, keep the fraction of potential pairings according to the parameter
    // "ratio":
    const size_t nTotal = sortedPairings.size();
    ASSERT_(nTotal > 0);

    const auto nKeep = mrpt::round(double(nTotal) * inliersRatio);

    // Prepare output: no correspondences initially:
    auto itEnd = sortedPairings.begin();
    std::advance(itEnd, nKeep);

    for (auto it = sortedPairings.begin(); it != itEnd; ++it)
    {
        const auto localIdx  = it->second.localIdx;
        const auto globalIdx = it->second.globalIdx;

        // Filter out if global alread assigned:
        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName)[globalIdx])
            continue;  // skip, global point already paired.

        out.paired_pt2pt.push_back(it->second);

        // Mark local & global points as already paired:
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);
        ms.globalPairedBitField.point_layers[globalName].mark_as_set(globalIdx);
    }

    MRPT_END
}
