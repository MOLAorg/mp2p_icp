/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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
#include <mrpt/math/TPoint3D.h>

#include <chrono>
#include <numeric>  // iota
#include <random>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_DistanceThreshold, Matcher, mp2p_icp);

using namespace mp2p_icp;

Matcher_Points_DistanceThreshold::Matcher_Points_DistanceThreshold()
{
    mrpt::system::COutputLogger::setLoggerName(
        "Matcher_Points_DistanceThreshold");
}

void Matcher_Points_DistanceThreshold::initialize(
    const mrpt::containers::Parameters& params)
{
    threshold_ = params["threshold"];

    if (params.has("pointLayerWeights"))
        initializeLayerWeights(params["pointLayerWeights"]);

    maxLocalPointsPerLayer_ =
        params.getOrDefault("maxLocalPointsPerLayer", maxLocalPointsPerLayer_);
    localPointsSampleSeed_ =
        params.getOrDefault("localPointsSampleSeed", localPointsSampleSeed_);
}

void Matcher_Points_DistanceThreshold::implMatchOneLayer(
    const mrpt::maps::CPointsMap& pcGlobal,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, Pairings& out) const
{
    MRPT_START

    const size_t nLocalPoints = pcLocal.size();

    const auto            fMax = std::numeric_limits<float>::max();
    mrpt::math::TPoint3Df localMin(fMax, fMax, fMax);
    mrpt::math::TPoint3Df localMax(-fMax, -fMax, -fMax);

    // Empty maps?  Nothing to do
    if (pcGlobal.empty() || pcLocal.empty()) return;

    // Prepare output: no correspondences initially:
    out.paired_points.reserve(out.paired_points.size() + nLocalPoints);

    // Select and transform local points: all, or a random subset:
    mrpt::aligned_std_vector<float> x_locals, y_locals, z_locals;

    const auto lambdaKeepBBox = [&](float x, float y, float z) {
        mrpt::keep_max(localMax.x, x);
        mrpt::keep_max(localMax.y, y);
        mrpt::keep_max(localMax.z, z);

        mrpt::keep_min(localMin.x, x);
        mrpt::keep_min(localMin.y, y);
        mrpt::keep_min(localMin.z, z);
    };

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    // Reordering indexes, used only if we had to pick random indexes:
    std::optional<std::vector<std::size_t>> idxs;

    if (nLocalPoints <= maxLocalPointsPerLayer_)
    {
        // All points:
        x_locals.resize(nLocalPoints);
        y_locals.resize(nLocalPoints);
        z_locals.resize(nLocalPoints);

        for (size_t i = 0; i < nLocalPoints; i++)
        {
            localPose.composePoint(
                lxs[i], lys[i], lzs[i], x_locals[i], y_locals[i], z_locals[i]);
            lambdaKeepBBox(x_locals[i], y_locals[i], z_locals[i]);
        }
    }
    else
    {
        // random subset:
        idxs.emplace(maxLocalPointsPerLayer_);
        std::iota(idxs->begin(), idxs->end(), 0);

        const unsigned int seed =
            localPointsSampleSeed_ != 0
                ? localPointsSampleSeed_
                : std::chrono::system_clock::now().time_since_epoch().count();

        MRPT_TODO("Partial shuffle only?");
        std::shuffle(
            idxs->begin(), idxs->end(), std::default_random_engine(seed));

        x_locals.resize(localPointsSampleSeed_);
        y_locals.resize(localPointsSampleSeed_);
        z_locals.resize(localPointsSampleSeed_);

        for (size_t ri = 0; ri < localPointsSampleSeed_; ri++)
        {
            const auto i = (*idxs)[ri];
            localPose.composePoint(
                lxs[i], lys[i], lzs[i], x_locals[ri], y_locals[ri],
                z_locals[ri]);
            lambdaKeepBBox(x_locals[ri], y_locals[ri], z_locals[ri]);
        }
    }

    // Try to do matching only if the bounding boxes have some overlap:
    mrpt::math::TPoint3Df globalMin, globalMax;
    pcGlobal.boundingBox(
        globalMin.x, globalMax.x, globalMin.y, globalMax.y, globalMin.z,
        globalMax.z);

    // No need to compute: Is matching = null?
    if (localMin.x > globalMax.x || localMax.x < globalMin.x ||
        localMin.y > globalMax.y || localMax.y < globalMin.y)
        return;

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(threshold_);

    const auto& gxs = pcGlobal.getPointsBufferRef_x();
    const auto& gys = pcGlobal.getPointsBufferRef_y();
    const auto& gzs = pcGlobal.getPointsBufferRef_z();

    for (size_t i = 0; i < x_locals.size(); i++)
    {
        size_t localIdx = idxs.has_value() ? (*idxs)[i] : i;

        // For speed-up:
        const float lx = x_locals[i], ly = y_locals[i], lz = z_locals[i];

        {
            // Use a KD-tree to look for the nearnest neighbor of:
            //   (x_local, y_local, z_local)
            // In "this" (global/reference) points map.

            float        tentativeErrSqr;
            const size_t tentativeGlobalIdx = pcGlobal.kdTreeClosestPoint3D(
                lx, ly, lz,  // Look closest to this guy
                tentativeErrSqr  // save here the min. distance squared
            );

            // Distance below the threshold??
            if (tentativeErrSqr < maxDistForCorrespondenceSquared)
            {
                // Save new correspondence:
                auto& p = out.paired_points.emplace_back();

                p.this_idx = tentativeGlobalIdx;
                p.this_x   = gxs[tentativeGlobalIdx];
                p.this_y   = gys[tentativeGlobalIdx];
                p.this_z   = gzs[tentativeGlobalIdx];

                p.other_idx = localIdx;
                p.other_x   = lxs[localIdx];
                p.other_y   = lys[localIdx];
                p.other_z   = lzs[localIdx];

                p.errorSquareAfterTransformation = tentativeErrSqr;
            }
        }  // End of test_match
    }  // For each local point

    MRPT_END
}
