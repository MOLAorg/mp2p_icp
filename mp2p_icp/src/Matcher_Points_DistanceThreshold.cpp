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
 * @file   Matcher_Points_DistanceThreshold.cpp
 * @brief  Pointcloud matcher: fixed distance thresholds
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#endif

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_DistanceThreshold, Matcher, mp2p_icp)

using namespace mp2p_icp;

Matcher_Points_DistanceThreshold::Matcher_Points_DistanceThreshold()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Points_DistanceThreshold");
}

void Matcher_Points_DistanceThreshold::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    DECLARE_PARAMETER_REQ(params, threshold);
    DECLARE_PARAMETER_REQ(params, thresholdAngularDeg);
    DECLARE_PARAMETER_OPT(params, pairingsPerPoint);
}

void Matcher_Points_DistanceThreshold::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

    ASSERT_(pairingsPerPoint >= 1);
    ASSERT_GT_(threshold, .0);
    ASSERT_GE_(thresholdAngularDeg, .0);

    const mrpt::maps::NearestNeighborsCapable& nnGlobal =
        *mp2p_icp::MapToNN(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size() * pairingsPerPoint;

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty()) return;

    const TransformedLocalPointCloud tl = transform_local_to_global(
        pcLocal, localPose, maxLocalPointsPerLayer_, localPointsSampleSeed_);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax}, threshold + bounding_box_intersection_check_epsilon_))
        return;

    // Prepare output: no correspondences initially:
    out.paired_pt2pt.reserve(out.paired_pt2pt.size() + pcLocal.size());

    // Loop for each point in local map:
    // --------------------------------------------------
    const float maxDistForCorrespondenceSquared = mrpt::square(threshold);
    const float angularThresholdFactorSquared   = mrpt::square(mrpt::DEG2RAD(thresholdAngularDeg));

    const auto&  lxs       = pcLocal.getPointsBufferRef_x();
    const auto&  lys       = pcLocal.getPointsBufferRef_y();
    const auto&  lzs       = pcLocal.getPointsBufferRef_z();
    const size_t nLocalPts = lxs.size();

    // Make sure the 3D kd-trees (if used internally) are up to date, from this
    // single-thread call before entering into parallelization:
    nnGlobal.nn_prepare_for_3d_queries();

    const auto lambdaAddPair = [this, &ms, &globalName, &localName, &lxs, &lys, &lzs](
                                   mrpt::tfest::TMatchingPairList& outPairs, const size_t localIdx,
                                   const mrpt::math::TPoint3Df& globalPt,
                                   const uint64_t globalIdxOrID, const float errSqr)
    {
        // Filter out if global alread assigned, in another matcher up the
        // pipeline, for example.
        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName)[globalIdxOrID])
            return;  // skip, global point already paired.

        // Save new correspondence:
        auto& p = outPairs.emplace_back();

        p.globalIdx = globalIdxOrID;
        p.localIdx  = localIdx;
        p.global    = globalPt;
        p.local     = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

        p.errorSquareAfterTransformation = errSqr;

        // Mark local & global points as already paired:
        if (!allowMatchAlreadyMatchedGlobalPoints_)
        {
            ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);
            ms.globalPairedBitField.point_layers[globalName].mark_as_set(globalIdxOrID);
        }
    };

#if defined(MP2P_HAS_TBB)
    // For the TBB lambdas:
    // TBB call structure based on the beautiful implementation in KISS-ICP.
    using Result = mrpt::tfest::TMatchingPairList;

    auto newPairs = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, nLocalPts},
        // Identity
        Result(),
        // 1st lambda: Parallel computation
        [&](const tbb::blocked_range<size_t>& r, Result res) -> Result
        {
            res.reserve(r.size());
            std::vector<uint64_t>              neighborIndices;
            std::vector<float>                 neighborSqrDists;
            std::vector<mrpt::math::TPoint3Df> neighborPts;
            for (size_t i = r.begin(); i < r.end(); i++)
            {
                const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

                if (!allowMatchAlreadyMatchedPoints_ &&
                    ms.localPairedBitField.point_layers.at(localName)[localIdx])
                    continue;  // skip, already paired.

                // For speed-up:
                const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

                const float localNormSqr = mrpt::square(lx) + mrpt::square(ly) + mrpt::square(lz);

                // Use a KD-tree to look for the nearnest neighbor(s) of
                // (x_local, y_local, z_local) in the global map.
                if (pairingsPerPoint == 1)
                {
                    neighborIndices.resize(1);
                    neighborSqrDists.resize(1);
                    neighborPts.resize(1);

                    if (!nnGlobal.nn_single_search(
                            {lx, ly, lz},  // Look closest to this guy
                            neighborPts[0], neighborSqrDists[0], neighborIndices[0]))
                    {
                        neighborIndices.clear();
                        neighborSqrDists.clear();
                        neighborPts.clear();
                    }
                }
                else
                {
                    // Use nn_radius_search() which provides a maximum search
                    // distance:
                    nnGlobal.nn_radius_search(
                        {lx, ly, lz},  // Look closest to this guy
                        maxDistForCorrespondenceSquared, neighborPts, neighborSqrDists,
                        neighborIndices, pairingsPerPoint);
                }

                // Distance below the threshold??
                for (size_t k = 0; k < neighborIndices.size(); k++)
                {
                    const auto tentativeErrSqr = neighborSqrDists.at(k);

                    const float finalThresSqr = maxDistForCorrespondenceSquared +
                                                angularThresholdFactorSquared * localNormSqr;

                    if (tentativeErrSqr >= finalThresSqr) break;  // skip this and the rest.

                    lambdaAddPair(
                        res, localIdx, neighborPts.at(k), neighborIndices.at(k), tentativeErrSqr);
                }
            }
            return res;
        },
        // 2nd lambda: Parallel reduction
        [](Result a, const Result& b) -> Result
        {
            a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
            return a;
        });

    out.paired_pt2pt.insert(
        out.paired_pt2pt.end(), std::make_move_iterator(newPairs.begin()),
        std::make_move_iterator(newPairs.end()));
#else

    out.paired_pt2pt.reserve(nLocalPts);

    std::vector<uint64_t>              neighborIndices;
    std::vector<float>                 neighborSqrDists;
    std::vector<mrpt::math::TPoint3Df> neighborPts;

    for (size_t i = 0; i < nLocalPts; i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
            continue;  // skip, already paired.

        // For speed-up:
        const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

        const float localNormSqr = mrpt::square(lx) + mrpt::square(ly) + mrpt::square(lz);

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map.
        if (pairingsPerPoint == 1)
        {
            neighborIndices.resize(1);
            neighborSqrDists.resize(1);
            neighborPts.resize(1);

            if (!nnGlobal.nn_single_search(
                    {lx, ly, lz},  // Look closest to this guy
                    neighborPts[0], neighborSqrDists[0], neighborIndices[0]))
            {
                neighborIndices.clear();
                neighborSqrDists.clear();
                neighborPts.clear();
            }
        }
        else
        {
            nnGlobal.nn_multiple_search(
                {lx, ly, lz},  // Look closest to this guy
                pairingsPerPoint, neighborPts, neighborSqrDists, neighborIndices);
        }

        // Distance below the threshold??
        for (size_t k = 0; k < neighborIndices.size(); k++)
        {
            const auto tentativeErrSqr = neighborSqrDists.at(k);

            const float finalThresSqr =
                maxDistForCorrespondenceSquared + angularThresholdFactorSquared * localNormSqr;

            if (tentativeErrSqr >= finalThresSqr) break;  // skip this and the rest.

            lambdaAddPair(
                out.paired_pt2pt, localIdx, neighborPts.at(k), neighborIndices.at(k),
                tentativeErrSqr);
        }
    }
#endif

    MRPT_END
}
