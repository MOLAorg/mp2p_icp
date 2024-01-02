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
#include <mrpt/version.h>

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
    DECLARE_PARAMETER_REQ(params, searchRadius);
    MCP_LOAD_REQ(params, knn);
    DECLARE_PARAMETER_REQ(params, planeEigenThreshold);
    MCP_LOAD_REQ(params, minimumPlanePoints);
    ASSERT_GE_(minimumPlanePoints, 3UL);

    MCP_LOAD_OPT(params, localPointMustFitPlaneToo);
    MCP_LOAD_OPT(params, localToGlobalPlaneMinAbsCosine);
}

// Filter the list of neighbors by maximum distance threshold:
static void filterListByMaxDist(
    std::vector<size_t>& kddIdxs, std::vector<float>& kddSqrDist,
    const float maxDistForCorrespondenceSquared)
{
    // Faster common case: all points are valid:
    if (!kddSqrDist.empty() &&
        kddSqrDist.back() < maxDistForCorrespondenceSquared)
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
}

void Matcher_Point2Plane::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

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
    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(searchRadius);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    std::vector<float>                 kddSqrDist, kddSqrDistLocal;
    std::vector<size_t>                kddIdxs, kddIdxsLocal;
    std::vector<mrpt::math::TPoint3Df> kddPts, kddPtsLocal;
    std::vector<float>                 kddXs, kddYs, kddZs;

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
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map.
        nnGlobal.nn_multiple_search(
            {lx, ly, lz},  // Look closest to this guy
            knn, kddPts, kddSqrDist, kddIdxs);

        // Filter the list of neighbors by maximum distance threshold:
        filterListByMaxDist(
            kddIdxs, kddSqrDist, maxDistForCorrespondenceSquared);

        // minimum: 3 points to be able to fit a plane
        if (kddIdxs.size() < minimumPlanePoints) continue;

        mp2p_icp::vector_of_points_to_xyz(kddPts, kddXs, kddYs, kddZs);

        const PointCloudEigen& eig = mp2p_icp::estimate_points_eigen(
            kddXs.data(), kddYs.data(), kddZs.data(), std::nullopt,
            kddPts.size());

        // Do these points look like a plane?
#if 0
        std::cout << "eig values: " << eig.eigVals[0] << " " << eig.eigVals[1]
                  << " " << eig.eigVals[2]
                  << " eigvec0: " << eig.eigVectors[0].asString() << "\n"
                  << " eigvec1: " << eig.eigVectors[1].asString() << "\n"
                  << " eigvec2: " << eig.eigVectors[2].asString() << "\n";
#endif

        // e0/e2 must be < planeEigenThreshold:
        if (eig.eigVals[0] > planeEigenThreshold * eig.eigVals[2]) continue;

        const auto&                normal        = eig.eigVectors[0];
        const mrpt::math::TPoint3D planeCentroid = {
            eig.meanCov.mean.x(), eig.meanCov.mean.y(), eig.meanCov.mean.z()};

        const auto   thePlane    = mrpt::math::TPlane(planeCentroid, normal);
        const double ptPlaneDist = std::abs(thePlane.distance({lx, ly, lz}));

        if (ptPlaneDist > distanceThreshold) continue;  // plane is too distant

        // Next, check plane-to-plane angle:
        if (localPointMustFitPlaneToo)
        {
            pcLocal.kdTreeNClosestPoint3DIdx(
                // Look closest to this guy
                lxs[localIdx], lys[localIdx], lzs[localIdx],
                // This max number of matches
                knn, kddIdxsLocal, kddSqrDistLocal);

            filterListByMaxDist(
                kddIdxsLocal, kddSqrDistLocal, maxDistForCorrespondenceSquared);

            // minimum: 3 points to be able to fit a plane
            if (kddIdxsLocal.size() < minimumPlanePoints) continue;

            const PointCloudEigen& eigLocal = mp2p_icp::estimate_points_eigen(
                tl.x_locals.data(), tl.y_locals.data(), tl.z_locals.data(),
                kddIdxsLocal);

            // Do these points look like a plane?
            // e0/e2 must be < planeEigenThreshold:
            if (eigLocal.eigVals[0] > planeEigenThreshold * eigLocal.eigVals[2])
                continue;

            const auto& normalLocal = eigLocal.eigVectors[0];

            const float dotProd = normalLocal.x * normal.x +
                                  normalLocal.y * normal.y +
                                  normalLocal.z * normal.z;

            // Angle too large?
            if (std::abs(dotProd) < localToGlobalPlaneMinAbsCosine) continue;
        }

        // OK, all conditions pass: add the new pairing:
        auto& p              = out.paired_pt2pl.emplace_back();
        p.pt_local           = {lxs[localIdx], lys[localIdx], lzs[localIdx]};
        p.pl_global.centroid = planeCentroid;

        p.pl_global.plane = thePlane;

        // Mark local point as already paired:
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);

    }  // For each local point

    MRPT_END
}
