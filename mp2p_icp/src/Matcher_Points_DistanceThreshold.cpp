/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_DistanceThreshold.cpp
 * @brief  Pointcloud matcher: fixed distance thresholds
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_DistanceThreshold, Matcher, mp2p_icp)

using namespace mp2p_icp;

Matcher_Points_DistanceThreshold::Matcher_Points_DistanceThreshold()
{
    mrpt::system::COutputLogger::setLoggerName(
        "Matcher_Points_DistanceThreshold");
}

void Matcher_Points_DistanceThreshold::initialize(
    const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    MCP_LOAD_REQ(params, threshold);
    MCP_LOAD_OPT(params, pairingsPerPoint);

    ASSERT_(pairingsPerPoint >= 1);
}

void Matcher_Points_DistanceThreshold::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    const layer_name_t& globalName, const layer_name_t& localName,
    Pairings& out) const
{
    MRPT_START

    const mrpt::maps::NearestNeighborsCapable& nnGlobal =
        *mp2p_icp::MapToNN(pcGlobalMap, true /*throw if cannot convert*/);

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax},
            bounding_box_intersection_check_epsilon_))
        return;

    // Prepare output: no correspondences initially:
    out.paired_pt2pt.reserve(out.paired_pt2pt.size() + pcLocal.size());

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(threshold);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    // In order to find the closest association for each global point, we must
    // first build this temporary list of *potential* associations, indexed by
    // global point indices, and sorted by errSqr:
    std::map<uint64_t, std::map<float, mrpt::tfest::TMatchingPair>>
        candidateMatchesForGlobal;

    const auto lambdaAddPair = [this, &candidateMatchesForGlobal, &lxs, &lys,
                                &lzs, &ms, &globalName](
                                   const size_t                 localIdx,
                                   const mrpt::math::TPoint3Df& globalPt,
                                   const uint64_t               globalIdxOrID,
                                   const float                  errSqr) {
        // Filter out if global alread assigned, in another matcher up the
        // pipeline, for example.
        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName)[globalIdxOrID])
            return;  // skip, global point already paired.

        // Save new correspondence:
        auto& p = candidateMatchesForGlobal[globalIdxOrID][errSqr];

        p.globalIdx = globalIdxOrID;
        p.localIdx  = localIdx;
        p.global    = globalPt;
        p.local     = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

        p.errorSquareAfterTransformation = errSqr;
    };

    // Declared out of the loop to avoid memory reallocations (!)
    std::vector<size_t>                neighborIndices;
    std::vector<float>                 neighborSqrDists;
    std::vector<mrpt::math::TPoint3Df> neighborPts;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
            continue;  // skip, already paired.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map.
        nnGlobal.nn_radius_search(
            {lx, ly, lz},  // Look closest to this guy
            maxDistForCorrespondenceSquared, neighborPts, neighborSqrDists,
            neighborIndices, pairingsPerPoint);

        // Distance below the threshold??
        for (size_t k = 0; k < neighborIndices.size(); k++)
        {
            const auto tentativeErrSqr = neighborSqrDists.at(k);
            if (tentativeErrSqr >= maxDistForCorrespondenceSquared)
                break;  // skip this and the rest.

            lambdaAddPair(
                localIdx, neighborPts.at(k), neighborIndices.at(k),
                tentativeErrSqr);
        }

    }  // For each local point

    // Now, process candidates pairing and store them in `out.paired_pt2pt`:
    for (const auto& kv : candidateMatchesForGlobal)
    {
        const auto globalIdx = kv.first;

        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName)[globalIdx])
            continue;  // skip, global point already paired.

        const auto& pairs = kv.second;
        ASSERT_(!pairs.empty());

        // take the one with the smallest error (std::map sorts them by sqrErr):
        const auto& bestPair = pairs.begin()->second;

        out.paired_pt2pt.emplace_back(bestPair);

        // Mark local & global points as already paired:
        if (!allowMatchAlreadyMatchedGlobalPoints_)
        {
            const auto localIdx = bestPair.localIdx;
            ms.localPairedBitField.point_layers[localName].mark_as_set(
                localIdx);
            ms.globalPairedBitField.point_layers[globalName].mark_as_set(
                globalIdx);
        }
    }

    MRPT_END
}
