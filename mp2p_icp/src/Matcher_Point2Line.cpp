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
 * @file   Matcher_Point2Line.cpp
 * @brief  Pointcloud matcher: point to line-fit of nearby points
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2021
 */

#include <mp2p_icp/Matcher_Point2Line.h>
#include <mp2p_icp/estimate_points_eigen.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Point2Line, Matcher, mp2p_icp)

using namespace mp2p_icp;

Matcher_Point2Line::Matcher_Point2Line()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Point2Line");
}

void Matcher_Point2Line::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    MCP_LOAD_REQ(params, distanceThreshold);
    MCP_LOAD_REQ(params, knn);
    MCP_LOAD_REQ(params, lineEigenThreshold);
    MCP_LOAD_REQ(params, minimumLinePoints);
    ASSERT_GE_(minimumLinePoints, 2UL);
}

void Matcher_Point2Line::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName, const layer_name_t& localName,
    Pairings& out) const
{
    MRPT_START

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
            distanceThreshold + bounding_box_intersection_check_epsilon_))
        return;

    // Prepare output: no correspondences initially:
    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(distanceThreshold);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    std::vector<float>                 kddSqrDist;
    std::vector<uint64_t>              kddIdxs;
    std::vector<mrpt::math::TPoint3Df> kddPts;
    std::vector<float>                 kddXs, kddYs, kddZs;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
            continue;  // skip, already paired.

        // Don't discard **global** map points if already used by another
        // matcher, since the assumption of "line" features implies that
        // many local points may match the *same* "global line", so it's ok
        // to have multiple-local-to-one-global pairings.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map.
        nnGlobal.nn_multiple_search(
            {lx, ly, lz},  // Look closest to this guy
            knn, kddPts, kddSqrDist, kddIdxs);

        // Filter the list of neighbors by maximum distance threshold:

        // Faster common case: all points are valid:
        if (!kddSqrDist.empty() && kddSqrDist.back() < maxDistForCorrespondenceSquared)
        {
            // Nothing to do: all knn points are within the range.
        }
        else
        {
            for (size_t j = 0; j < kddSqrDist.size(); j++)
            {
                if (kddSqrDist[j] > maxDistForCorrespondenceSquared)
                {
                    kddIdxs.resize(j);
                    kddSqrDist.resize(j);
                    break;
                }
            }
        }

        // minimum: 2 points to be able to fit a line
        if (kddIdxs.size() < minimumLinePoints) continue;

        mp2p_icp::vector_of_points_to_xyz(kddPts, kddXs, kddYs, kddZs);

        const PointCloudEigen& eig = mp2p_icp::estimate_points_eigen(
            kddXs.data(), kddYs.data(), kddZs.data(), std::nullopt, kddPts.size());

        // Do these points look like a line?
#if 0
        std::cout << "eig values: " << eig.eigVals[0] << " " << eig.eigVals[1]
                  << " " << eig.eigVals[2]
                  << " eigvec0: " << eig.eigVectors[0].asString() << "\n"
                  << " eigvec1: " << eig.eigVectors[1].asString() << "\n"
                  << " eigvec2: " << eig.eigVectors[2].asString() << "\n";
#endif

        // e0/e{1,2} must be < lineEigenThreshold:
        if (eig.eigVals[0] > lineEigenThreshold * eig.eigVals[2]) continue;
        if (eig.eigVals[1] > lineEigenThreshold * eig.eigVals[2]) continue;

        auto& p    = out.paired_pt2ln.emplace_back();
        p.pt_local = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

        const auto& normal   = eig.eigVectors[2];
        p.ln_global.pBase    = {eig.meanCov.mean.x(), eig.meanCov.mean.y(), eig.meanCov.mean.z()};
        p.ln_global.director = normal.unitarize();

        // Mark local point as already paired:
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);

    }  // For each local point

    MRPT_END
}
