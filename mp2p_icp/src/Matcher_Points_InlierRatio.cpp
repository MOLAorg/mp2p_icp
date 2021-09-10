/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
    const mrpt::maps::CPointsMap& pcGlobal,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, Pairings& out) const
{
    MRPT_START

    ASSERT_GT_(inliersRatio, 0.0);
    ASSERT_LT_(inliersRatio, 1.0);

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

    // Loop for each point in local map:
    // --------------------------------------------------
    const auto& gxs = pcGlobal.getPointsBufferRef_x();
    const auto& gys = pcGlobal.getPointsBufferRef_y();
    const auto& gzs = pcGlobal.getPointsBufferRef_z();

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    std::multimap<double, mrpt::tfest::TMatchingPair> sortedPairings;

    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        {
            // Use a KD-tree to look for the nearnest neighbor of:
            //   (x_local, y_local, z_local)
            // In "this" (global/reference) points map.

            float        tentativeErrSqr;
            const size_t tentativeGlobalIdx = pcGlobal.kdTreeClosestPoint3D(
                lx, ly, lz,  // Look closest to this guy
                tentativeErrSqr  // save here the min. distance squared
            );

            mrpt::tfest::TMatchingPair p;
            p.this_idx = tentativeGlobalIdx;
            p.this_x   = gxs[tentativeGlobalIdx];
            p.this_y   = gys[tentativeGlobalIdx];
            p.this_z   = gzs[tentativeGlobalIdx];

            p.other_idx = localIdx;
            p.other_x   = lxs[localIdx];
            p.other_y   = lys[localIdx];
            p.other_z   = lzs[localIdx];

            p.errorSquareAfterTransformation = tentativeErrSqr;

            // Sort by distance:
            sortedPairings.emplace_hint(
                sortedPairings.begin(), tentativeErrSqr, p);

        }  // End of test_match
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
        out.paired_pt2pt.push_back(it->second);

    MRPT_END
}
