/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Adaptive.cpp
 * @brief  Pointcloud matcher: smart adaptive matcher
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2023
 */

#include <mp2p_icp/Matcher_Adaptive.h>
#include <mrpt/containers/vector_with_small_size_optimization.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/math/CHistogram.h>  // CHistogram
#include <mrpt/math/distributions.h>  // confidenceIntervalsFromHistogram()
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Adaptive, Matcher, mp2p_icp)

using namespace mp2p_icp;

void Matcher_Adaptive::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    MCP_LOAD_REQ(params, confidenceInterval);
    MCP_LOAD_REQ(params, firstToSecondDistanceMax);
    MCP_LOAD_REQ(params, absoluteMaxSearchDistance);

    ASSERT_LT_(confidenceInterval, 1.0);
    ASSERT_GT_(confidenceInterval, 0.0);
}

void Matcher_Adaptive::implMatchOneLayer(
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
    const float absoluteMaxDistSqr = mrpt::square(absoluteMaxSearchDistance);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    // In order to find the closest association for each global point, we must
    // first build this temporary list of *potential* associations, indexed by
    // global point indices, and sorted by errSqr:

    // List of all 1st and 2nd closest pairings to each
    constexpr size_t MAX_CORRS_PER_LOCAL = 10;

    std::vector<mrpt::containers::vector_with_small_size_optimization<
        mrpt::tfest::TMatchingPair, MAX_CORRS_PER_LOCAL>>
        matchesPerLocal;
    matchesPerLocal.resize(tl.x_locals.size());

    // Calculate limits for the histogram:
    std::optional<float> minSqrErrorForHistogram;
    std::optional<float> maxSqrErrorForHistogram;

    const auto lambdaAddPair =
        [&matchesPerLocal, &lxs, &lys, &lzs](
            const size_t localIdx, const mrpt::math::TPoint3Df& globalPt,
            const uint64_t globalIdxOrID, const float errSqr) {
            auto& ps = matchesPerLocal.at(localIdx);
            if (ps.size() >= MAX_CORRS_PER_LOCAL) return;

            mrpt::tfest::TMatchingPair p;
            p.globalIdx = globalIdxOrID;
            p.localIdx  = localIdx;
            p.global    = globalPt;
            p.local     = {lxs[localIdx], lys[localIdx], lzs[localIdx]};
            p.errorSquareAfterTransformation = errSqr;

            ps.push_back(p);
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
        {
            // skip, already paired, e.g. by another Matcher in the pipeline
            // before me:
            continue;
        }

        const float lx = tl.x_locals[i], ly = tl.y_locals[i],
                    lz = tl.z_locals[i];

        constexpr size_t maxNNPoints = 2;

        // Use a KD-tree to look for the nearnest neighbor(s) of
        // (x_local, y_local, z_local) in the global map:
        nnGlobal.nn_radius_search(
            {lx, ly, lz},  // Look closest to this guy
            absoluteMaxDistSqr, neighborPts, neighborSqrDists, neighborIndices,
            maxNNPoints);

        for (size_t k = 0; k < neighborIndices.size(); k++)
        {
            const auto tentativeErrSqr = neighborSqrDists.at(k);

            if (k <= 1)
            {
                // keep max of 1st and 2nd closest point errors for the
                // histogram:
                if (maxSqrErrorForHistogram)
                {
                    mrpt::keep_max(*maxSqrErrorForHistogram, tentativeErrSqr);
                    mrpt::keep_min(*minSqrErrorForHistogram, tentativeErrSqr);
                }
                else
                {
                    maxSqrErrorForHistogram = tentativeErrSqr;
                    minSqrErrorForHistogram = tentativeErrSqr;
                }
            }

            lambdaAddPair(
                localIdx, neighborPts.at(k), neighborIndices.at(k),
                tentativeErrSqr);
        }

    }  // For each local point

    // Now, estimate the probability distribution (histogram) of the
    // 1st/2nd points:
    mrpt::math::CHistogram hist(
        *minSqrErrorForHistogram, *maxSqrErrorForHistogram, 50);

    for (const auto& mspl : matchesPerLocal)
        for (size_t i = 0; i < std::min(mspl.size(), 2UL); i++)
            hist.add(mspl[i].errorSquareAfterTransformation);

    std::vector<double> histXs, histValues;
    hist.getHistogramNormalized(histXs, histValues);

    double ci_low = 0, ci_high = 0;
    mrpt::math::confidenceIntervalsFromHistogram(
        histXs, histValues, ci_low, ci_high, 1.0 - confidenceInterval);

#if 0
    printf("Histograms:\n");
    for (auto v : histXs) printf("%.02f ", v);
    printf("\n");
    for (auto v : histValues) printf("%.02f ", v);
    printf("\n");
#endif
#if 0
    printf("CI_HIGH: %.03f => %.03f meters\n", ci_high, std::sqrt(ci_high));
#endif

    // Take the confidence interval limit as the definitive maximum squared
    // distance for correspondences:
    const double maxCorrDistSqr = ci_high;

    const float maxSqr1to2 = mrpt::square(firstToSecondDistanceMax);

    // Now, process candidates pairing and store them in `out.paired_pt2pt`:
    for (const auto& mspl : matchesPerLocal)
    {
        for (size_t i = 0; i < std::min(mspl.size(), 2UL); i++)
        {
            const auto& p         = mspl.at(i);
            const auto  globalIdx = p.globalIdx;

            if (!allowMatchAlreadyMatchedGlobalPoints_ &&
                ms.globalPairedBitField.point_layers.at(globalName)[globalIdx])
                continue;  // skip, global point already paired.

            // too large error for the adaptive threshold?
            if (p.errorSquareAfterTransformation >= maxCorrDistSqr) continue;

            if (i == 1 &&
                mspl[i].errorSquareAfterTransformation >
                    mspl[0].errorSquareAfterTransformation * maxSqr1to2)
            {
                break;
            }

            out.paired_pt2pt.emplace_back(p);

            // Mark local & global points as already paired:
            if (!allowMatchAlreadyMatchedGlobalPoints_)
            {
                const auto localIdx = p.localIdx;
                ms.localPairedBitField.point_layers[localName].mark_as_set(
                    localIdx);
            }
        }
    }

// Now, mark global idxs as used. It's done after the loop above
// to allow multiple local -> global pairs with the same global.
#if 0
                // Mark local & global points as already paired:
            ms.globalPairedBitField.point_layers[globalName].mark_as_set(
                globalIdx);
#endif

    MRPT_END
}
