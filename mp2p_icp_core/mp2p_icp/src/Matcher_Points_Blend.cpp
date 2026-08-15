/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Matcher_Points_Blend.cpp
 * @brief  Pointcloud matcher: point to a distance-weighted blend of nearby
 *         map points
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_Points_Blend.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#endif

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_Blend, Matcher, mp2p_icp)

using namespace mp2p_icp;

namespace
{
/** Radial fade: 1 at the query point, and exactly zero, with zero derivative,
 *  at r=R. The zero derivative is the point: it is what makes a map point
 *  crossing the neighborhood boundary a continuous event.
 */
double smoothTaper(const double rSqr, const double RSqr)
{
    if (rSqr >= RSqr)
    {
        return 0;
    }
    const double v = 1.0 - rSqr / RSqr;
    return v * v;
}
}  // namespace

Matcher_Points_Blend::Matcher_Points_Blend()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Points_Blend");
}

void Matcher_Points_Blend::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    DECLARE_PARAMETER_REQ(params, threshold);
    DECLARE_PARAMETER_REQ(params, thresholdAngularDeg);
    DECLARE_PARAMETER_OPT(params, temperature);
    DECLARE_PARAMETER_OPT(params, searchRadius);
    DECLARE_PARAMETER_OPT(params, minWeightSum);

    if (params.has("maxNeighbors"))
    {
        maxNeighbors = params["maxNeighbors"].as<uint32_t>();
    }
    if (params.has("smoothCutoff"))
    {
        smoothCutoff = params["smoothCutoff"].as<bool>();
    }
}

void Matcher_Points_Blend::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
    const layer_name_t& localName, Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

    ASSERT_GT_(threshold, .0);
    ASSERT_GE_(thresholdAngularDeg, .0);
    // A negative value would silently select the zero-temperature path, or
    // silently fall back to `threshold`, turning off the requested behavior.
    ASSERT_GE_(temperature, .0);
    ASSERT_GE_(searchRadius, .0);

    const auto threshold_f = static_cast<float>(threshold);

    const mrpt::maps::NearestNeighborsCapable& nnGlobal =
        *mp2p_icp::MapToNN(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty())
    {
        return;
    }

    const TransformedLocalPointCloud tl = transform_local_to_global(pcLocal, localPose);

    const bool blending = temperature > 0;

    // Blending window. Falling back to `threshold` keeps the parameter set
    // minimal: a map point that could never form a pairing anyway has nothing
    // to contribute to the target either.
    const double blendRadius = searchRadius > 0 ? searchRadius : threshold;

    // At zero temperature nothing is blended, and the search reverts to
    // `threshold` so that this whole path, the bounding-box test included,
    // matches Matcher_Points_DistanceThreshold exactly.
    const auto searchRadius_f =
        blending ? static_cast<float>(std::max(threshold, blendRadius)) : threshold_f;

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax}, searchRadius_f + bounding_box_intersection_check_epsilon_))
    {
        return;
    }

    // Prepare output: no correspondences initially:
    out.paired_pt2pt.reserve(out.paired_pt2pt.size() + pcLocal.size());

    const float maxDistForCorrespondenceSquared = mrpt::square(threshold_f);
    const float angularThresholdFactorSquared =
        mrpt::square(mrpt::DEG2RAD(static_cast<float>(thresholdAngularDeg)));

    const auto&  lxs       = pcLocal.getPointsBufferRef_x();
    const auto&  lys       = pcLocal.getPointsBufferRef_y();
    const auto&  lzs       = pcLocal.getPointsBufferRef_z();
    const size_t nLocalPts = lxs.size();

    const double blendRadiusSqr = blendRadius * blendRadius;
    const double invTwoTSqr     = blending ? 0.5 / (temperature * temperature) : 0;

    // Zero means "every point in the radius", and that is the only value that
    // keeps this a radius query. Any other value makes nn_radius_search() sort
    // its candidates and keep the best few, which is a top-k rule: a map point
    // entering or leaving that list is exactly the hard flip this matcher
    // exists to remove, and some map classes cap the list far lower than a
    // caller would guess.
    const size_t maxNN = maxNeighbors;

    // Make sure the 3D kd-trees (if used internally) are up to date, from this
    // single-thread call before entering into parallelization:
    nnGlobal.nn_prepare_for_3d_queries();

    const auto lambdaAddPair = [this, &ms, &globalName, &localName, &lxs, &lys, &lzs](
                                   mrpt::tfest::TMatchingPairList& outPairs, const size_t localIdx,
                                   const mrpt::math::TPoint3Df& globalPt,
                                   const uint64_t globalIdxOrID, const float errSqr)
    {
        // Filter out if global already assigned, in another matcher up the
        // pipeline, for example.
        if (!allowMatchAlreadyMatchedGlobalPoints_ &&
            ms.globalPairedBitField.point_layers.at(globalName)[globalIdxOrID])
        {
            return;  // skip, global point already paired.
        }

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

    // Resolves the target for one local point and appends the pairing, if any.
    // Shared by the parallel and serial paths so they cannot drift apart.
    const auto lambdaOnePoint = [&](mrpt::tfest::TMatchingPairList& outPairs, const size_t i,
                                    std::vector<mrpt::math::TPoint3Df>& neighborPts,
                                    std::vector<float>&                 neighborSqrDists,
                                    std::vector<uint64_t>&              neighborIndices)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
        {
            return;  // skip, already paired.
        }

        const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

        const float localNormSqr = mrpt::square(lx) + mrpt::square(ly) + mrpt::square(lz);
        const float finalThresSqr =
            maxDistForCorrespondenceSquared + angularThresholdFactorSquared * localNormSqr;

        if (!blending)
        {
            // Zero temperature: keep the single nearest map point. This is the
            // control end of the blend, and it deliberately runs the very same
            // query as Matcher_Points_DistanceThreshold so that it reproduces
            // it exactly, ties included.
            neighborIndices.resize(1);
            neighborSqrDists.resize(1);
            neighborPts.resize(1);

            if (!nnGlobal.nn_single_search(
                    {lx, ly, lz}, neighborPts[0], neighborSqrDists[0], neighborIndices[0]))
            {
                return;
            }
            if (neighborSqrDists[0] >= finalThresSqr)
            {
                return;
            }
            lambdaAddPair(
                outPairs, localIdx, neighborPts[0], neighborIndices[0], neighborSqrDists[0]);
            return;
        }

        nnGlobal.nn_radius_search(
            {lx, ly, lz}, mrpt::square(searchRadius_f), neighborPts, neighborSqrDists,
            neighborIndices, maxNN);

        if (neighborIndices.empty())
        {
            return;
        }

        // Offsetting all the exponents by the smallest one is exactly cancelled
        // by the normalization below, and it keeps exp() from underflowing to
        // zero for every neighbor at small temperatures.
        double minSqrDist = std::numeric_limits<double>::max();
        size_t nearestK   = 0;

        for (size_t k = 0; k < neighborIndices.size(); k++)
        {
            const double dSqr = neighborSqrDists[k];
            if (dSqr >= blendRadiusSqr)
            {
                continue;  // outside the taper: contributes nothing
            }
            if (dSqr < minSqrDist)
            {
                minSqrDist = dSqr;
                nearestK   = k;
            }
        }

        if (minSqrDist == std::numeric_limits<double>::max())
        {
            return;  // no neighbor inside the blending window
        }

        double               sumW = 0;
        mrpt::math::TPoint3D blended(0, 0, 0);

        for (size_t k = 0; k < neighborIndices.size(); k++)
        {
            const double dSqr  = neighborSqrDists[k];
            const double taper = smoothCutoff ? smoothTaper(dSqr, blendRadiusSqr)
                                              : (dSqr < blendRadiusSqr ? 1.0 : 0.0);
            if (taper <= 0)
            {
                continue;
            }
            const double w = taper * std::exp(-(dSqr - minSqrDist) * invTwoTSqr);
            if (w <= 0)
            {
                continue;
            }
            sumW += w;
            blended.x += w * neighborPts[k].x;
            blended.y += w * neighborPts[k].y;
            blended.z += w * neighborPts[k].z;
        }

        // The first test is the one that always has to be there: it is what
        // makes the normalization below safe.
        if (sumW <= 0 || sumW < minWeightSum)
        {
            return;
        }

        blended *= 1.0 / sumW;

        const mrpt::math::TPoint3Df target(
            static_cast<float>(blended.x), static_cast<float>(blended.y),
            static_cast<float>(blended.z));

        const float errSqr =
            mrpt::square(target.x - lx) + mrpt::square(target.y - ly) + mrpt::square(target.z - lz);

        if (errSqr >= finalThresSqr)
        {
            return;  // blended target is too distant
        }

        // The blended target is not a map point, so no index truly denotes it.
        // The nearest neighbor's index is the one that keeps the already-paired
        // bookkeeping closest to what the nearest-neighbor matcher would do.
        lambdaAddPair(outPairs, localIdx, target, neighborIndices[nearestK], errSqr);
    };

#if defined(MP2P_HAS_TBB)
    // TBB call structure mirrors Matcher_Points_DistanceThreshold: the pairing
    // ORDER is part of the pipeline's determinism, so this must not be quietly
    // rewritten as a serial loop.
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
                lambdaOnePoint(res, i, neighborPts, neighborSqrDists, neighborIndices);
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
        lambdaOnePoint(out.paired_pt2pt, i, neighborPts, neighborSqrDists, neighborIndices);
    }
#endif

    MRPT_END
}
