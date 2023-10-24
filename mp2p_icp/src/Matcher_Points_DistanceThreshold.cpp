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
    [[maybe_unused]] const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    const auto* pcGlobalPtr = mp2p_icp::MapToPointsMap(pcGlobalMap);
    if (!pcGlobalPtr)
        THROW_EXCEPTION_FMT(
            "This class only supports global maps of types convertible to "
            "point cloud types, but found type '%s'",
            pcGlobalMap.GetRuntimeClass()->className);
    const auto& pcGlobal = *pcGlobalPtr;

    // Empty maps?  Nothing to do
    if (pcGlobal.empty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobal.boundingBox().intersection(
            {tl.localMin, tl.localMax},
            bounding_box_intersection_check_epsilon_))
        return;

    // Prepare output: no correspondences initially:
    out.paired_pt2pt.reserve(out.paired_pt2pt.size() + pcLocal.size());

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(threshold);

    const auto& gxs = pcGlobal.getPointsBufferRef_x();
    const auto& gys = pcGlobal.getPointsBufferRef_y();
    const auto& gzs = pcGlobal.getPointsBufferRef_z();

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    // In order to find the closest association for each global point, we must
    // first build this temporary list of *potential* associations, indexed by
    // global point indices, and sorted by errSqr:
    std::map<size_t, std::map<float, mrpt::tfest::TMatchingPair>>
        candidateMatchesForGlobal;

    const auto lambdaAddPair = [this, &candidateMatchesForGlobal, &lxs, &lys,
                                &lzs, &gxs, &gys, &gzs, &ms, &globalName](
                                   const size_t localIdx,
                                   const size_t globalIdx, const float errSqr) {
        // Filter out if global alread assigned:
        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName).at(globalIdx))
            return;  // skip, global point already paired.

        // Save new correspondence:
        auto& p = candidateMatchesForGlobal[globalIdx][errSqr];

        p.globalIdx = globalIdx;
        p.localIdx  = localIdx;
        p.global    = {gxs[globalIdx], gys[globalIdx], gzs[globalIdx]};
        p.local     = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

        p.errorSquareAfterTransformation = errSqr;
    };

    // Declared out of the loop to avoid memory reallocations (!)
    std::vector<size_t> neighborIndices;
    std::vector<float>  neighborSqrDists;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName).at(localIdx))
            continue;  // skip, already paired.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor(s) of:
        //   (x_local, y_local, z_local)
        // In "this" (global/reference) points map.
        if (pairingsPerPoint == 1)
        {
            float        tentativeErrSqr;
            const size_t tentativeGlobalIdx = pcGlobal.kdTreeClosestPoint3D(
                lx, ly, lz,  // Look closest to this guy
                tentativeErrSqr  // save here the min. distance squared
            );

            // Distance below the threshold??
            if (tentativeErrSqr < maxDistForCorrespondenceSquared)
                lambdaAddPair(localIdx, tentativeGlobalIdx, tentativeErrSqr);
        }  // End of test_match
        else
        {
            pcGlobal.kdTreeNClosestPoint3DIdx(
                lx, ly, lz, pairingsPerPoint, neighborIndices,
                neighborSqrDists);

            // Distance below the threshold??
            for (size_t k = 0; k < neighborIndices.size(); k++)
            {
                const auto tentativeErrSqr = neighborSqrDists.at(k);
                if (tentativeErrSqr >= maxDistForCorrespondenceSquared)
                    break;  // skip this and the rest.

                lambdaAddPair(localIdx, neighborIndices.at(k), tentativeErrSqr);
            }
        }
    }  // For each local point

    // Now, process candidates pairing and store them in `out.paired_pt2pt`:
    for (const auto& kv : candidateMatchesForGlobal)
    {
        const auto globalIdx = kv.first;

        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName).at(globalIdx))
            continue;  // skip, global point already paired.

        const auto& pairs = kv.second;
        ASSERT_(!pairs.empty());

        // take the one with the smallest error (std::map sorts them by sqrErr):
        const auto& bestPair = pairs.begin()->second;

        out.paired_pt2pt.emplace_back(bestPair);

        // Mark local & global points as already paired:
        const auto localIdx = bestPair.localIdx;
        ms.localPairedBitField.point_layers[localName].at(localIdx)    = true;
        ms.globalPairedBitField.point_layers[globalName].at(globalIdx) = true;
    }

    MRPT_END
}
