/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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

    MCP_LOAD_REQ(params, distanceThreshold);
    MCP_LOAD_REQ(params, knn);
    MCP_LOAD_REQ(params, planeEigenThreshold);
}

void Matcher_Point2Plane::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    const auto* pcGlobalPtr =
        dynamic_cast<const mrpt::maps::CPointsMap*>(&pcGlobalMap);
    if (!pcGlobalPtr)
        THROW_EXCEPTION_FMT(
            "This class only supports global maps of point cloud types, but "
            "found type '%s'",
            pcGlobalMap.GetRuntimeClass()->className);
    const auto& pcGlobal = *pcGlobalPtr;

    // Empty maps?  Nothing to do
    if (pcGlobal.empty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
#if MRPT_VERSION >= 0x218
    if (!pcGlobal.boundingBox().intersection({tl.localMin, tl.localMax}))
        return;
#else
    mrpt::math::TPoint3Df globalMin, globalMax;
    pcGlobal.boundingBox(
        globalMin.x, globalMax.x, globalMin.y, globalMax.y, globalMin.z,
        globalMax.z);
    // No need to compute: Is matching = null?
    if (tl.localMin.x > globalMax.x || tl.localMax.x < globalMin.x ||
        tl.localMin.y > globalMax.y || tl.localMax.y < globalMin.y)
        return;
#endif
    // Prepare output: no correspondences initially:
    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared =
        mrpt::square(distanceThreshold);

    const auto& gxs = pcGlobal.getPointsBufferRef_x();
    const auto& gys = pcGlobal.getPointsBufferRef_y();
    const auto& gzs = pcGlobal.getPointsBufferRef_z();

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    std::vector<float>  kddSqrDist;
    std::vector<size_t> kddIdxs;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName).at(localIdx))
            continue;  // skip, already paired.

        // Don't discard **global** map points if already used by another
        // matcher, since the assumption of "plane" features implies that
        // many local points may match the *same* "global plane", so it's ok
        // to have multiple-local-to-one-global pairings.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        // Use a KD-tree to look for the nearnest neighbor of:
        //   (x_local, y_local, z_local)
        // In "this" (global/reference) points map.

        pcGlobal.kdTreeNClosestPoint3DIdx(
            lx, ly, lz,  // Look closest to this guy
            knn,  // This max number of matches
            kddIdxs, kddSqrDist);

        // Filter the list of neighbors by maximum distance threshold:

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

        // minimum: 3 points to be able to fit a plane
        if (kddIdxs.size() < 3) continue;

        const PointCloudEigen& eig = mp2p_icp::estimate_points_eigen(
            gxs.data(), gys.data(), gzs.data(), kddIdxs);

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

        auto& p            = out.paired_pt2pl.emplace_back();
        p.pt_other         = {lxs[localIdx], lys[localIdx], lzs[localIdx]};
        p.pl_this.centroid = {
            eig.meanCov.mean.x(), eig.meanCov.mean.y(), eig.meanCov.mean.z()};

        const auto& normal = eig.eigVectors[0];
        p.pl_this.plane    = mrpt::math::TPlane(p.pl_this.centroid, normal);

        // Mark local point as already paired:
        ms.localPairedBitField.point_layers[localName].at(localIdx) = true;

    }  // For each local point

    MRPT_END
}
