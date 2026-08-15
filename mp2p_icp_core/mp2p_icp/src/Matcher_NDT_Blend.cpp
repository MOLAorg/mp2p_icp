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
 * @file   Matcher_NDT_Blend.cpp
 * @brief  Pointcloud matcher: point to a likelihood-weighted blend of nearby
 *         plane models
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_NDT_Blend.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(Matcher_NDT_Blend, Matcher, mp2p_icp)

using namespace mp2p_icp;

namespace
{
/** Radial fade: 1 at the query point, and exactly zero, with zero derivative,
 *  at r=R. The zero derivative is the point: it is what makes a candidate
 *  crossing the window boundary a continuous event.
 */
double smoothTaper(const double r, const double R)
{
    if (r >= R)
    {
        return 0;
    }
    const double u = r / R;
    const double v = 1.0 - u * u;
    return v * v;
}

/** Whether to collect and report the blend's weight distribution.
 *
 *  Off unless MP2P_ICP_BLEND_STATS=1, because it costs an extra candidate
 *  enumeration on the zero-temperature path and is of no use in production.
 */
bool blendStatsEnabled()
{
    static const bool enabled = []()
    {
        const char* v = ::getenv("MP2P_ICP_BLEND_STATS");
        return v != nullptr && v[0] == '1' && v[1] == '\0';
    }();
    return enabled;
}
}  // namespace

Matcher_NDT_Blend::Matcher_NDT_Blend()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_NDT_Blend");

    // The application sets its own logger's level, not this one's, so the
    // statistics would otherwise be filtered out by the very verbosity setting
    // that is supposed to reveal them.
    if (blendStatsEnabled())
    {
        this->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
}

void Matcher_NDT_Blend::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    DECLARE_PARAMETER_REQ(params, distanceThreshold);
    DECLARE_PARAMETER_OPT(params, temperature);
    DECLARE_PARAMETER_OPT(params, searchRadius);
    DECLARE_PARAMETER_OPT(params, minWeightSum);

    if (params.has("smoothCutoff"))
    {
        smoothCutoff = params["smoothCutoff"].as<bool>();
    }
}

void Matcher_NDT_Blend::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName, const layer_name_t& localName,  // NOLINT
    Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // A negative value would silently select the zero-temperature path, or
    // silently fall back to distanceThreshold, and so would quietly turn off
    // the very behavior the caller asked for.
    ASSERT_GE_(temperature, .0);
    ASSERT_GE_(searchRadius, .0);
    ASSERT_GT_(distanceThreshold, .0);

    const mp2p_icp::NearestPlaneCapable& nnGlobal =
        *mp2p_icp::MapToNP(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty())
    {
        return;
    }

    const TransformedLocalPointCloud tl = transform_local_to_global(pcLocal, localPose);

    const bool blending = temperature > 0;

    // Blending window. Falling back to distanceThreshold keeps the parameter
    // set minimal for maps whose plane models live at the query's own scale.
    const double blendRadius = searchRadius > 0 ? searchRadius : distanceThreshold;

    // The map enumerates candidates by cell index, so the region it walks must
    // cover the whole taper. Otherwise a candidate could still hold a nonzero
    // weight at the moment it drops out of the enumeration.
    //
    // At zero temperature nothing is blended, and the search reverts to
    // distanceThreshold so that this whole path, the bounding-box test
    // included, matches Matcher_Point2Plane exactly.
    const auto enumRadius = blending ? static_cast<float>(std::max(distanceThreshold, blendRadius))
                                     : static_cast<float>(distanceThreshold);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax}, enumRadius + bounding_box_intersection_check_epsilon_))
    {
        return;
    }

    // Prepare output: no correspondences initially:
    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    const double invTwoTSqr = blending ? 0.5 / (temperature * temperature) : 0;

    std::vector<NearestPlaneCapable::PlaneCandidate> candidates;
    std::vector<double>                              weights;

    // How concentrated the blend actually is. "How many candidates does a
    // query really combine" is the question that decides whether this matcher
    // is smoothing anything or is only an argmin over a restricted window, and
    // it is cheap enough to answer directly instead of arguing about it.
    const bool collectStats = blendStatsEnabled();

    size_t statQueries    = 0;
    size_t statEnumerated = 0;
    size_t statInWindow   = 0;
    double statEffective  = 0;  // sum over queries of 1/sum(w_norm^2)
    double statTopWeight  = 0;
    double statWinnerDist = 0;  // centroid distance of the heaviest candidate

    // Loop for each point in local map:
    // --------------------------------------------------
    for (size_t i = 0; i < tl.x_locals.size(); i++)
    {
        const size_t localIdx = tl.idxs.has_value() ? (*tl.idxs)[i] : i;

        if (!allowMatchAlreadyMatchedPoints_ &&
            ms.localPairedBitField.point_layers.at(localName)[localIdx])
        {
            continue;  // skip, already paired.
        }

        // As in Matcher_Point2Plane: global map points are not marked as used,
        // since many local points may legitimately match the same plane.

        const float                 lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];
        const mrpt::math::TPoint3Df queryPt = {lx, ly, lz};

        mp2p_icp::plane_patch_t targetPlane;

        if (!blending)
        {
            // Zero temperature: keep the single closest plane. This is the
            // control end of the blend, and it deliberately runs the very same
            // query as Matcher_Point2Plane so that it reproduces it exactly.
            const NearestPlaneCapable::NearestPlaneResult np =
                nnGlobal.nn_search_pt2pl(queryPt, static_cast<float>(distanceThreshold));

            if (!np.pairing)
            {
                continue;
            }
            if (np.distance > distanceThreshold)
            {
                continue;  // plane is too distant
            }
            targetPlane = np.pairing->pl_global;

            if (collectStats)
            {
                // One candidate, by definition. What is worth recording here is
                // how far the winner's cell actually sits from the query, since
                // an argmin on the plane distance is free to select a distant
                // cell whose plane merely happens to graze the query point.
                size_t nEnum = 0;
                nnGlobal.nn_visit_pt2pl_candidates(
                    queryPt, enumRadius,
                    [&nEnum](const NearestPlaneCapable::PlaneCandidate&) { nEnum++; });

                statQueries++;
                statEnumerated += nEnum;
                statInWindow += 1;
                statEffective += 1.0;
                statTopWeight += 1.0;
                statWinnerDist += (targetPlane.centroid - mrpt::math::TPoint3D(queryPt)).norm();
            }
        }
        else
        {
            candidates.clear();
            nnGlobal.nn_visit_pt2pl_candidates(
                queryPt, enumRadius,
                [&candidates](const NearestPlaneCapable::PlaneCandidate& c)
                { candidates.push_back(c); });

            if (candidates.empty())
            {
                continue;
            }

            // Fades first, so that the reference energy below is taken over the
            // candidates that actually contribute.
            weights.clear();
            weights.reserve(candidates.size());

            double minEnergy = std::numeric_limits<double>::max();

            for (const auto& c : candidates)
            {
                const double taper = smoothCutoff ? smoothTaper(c.centroidDistance, blendRadius)
                                                  : (c.centroidDistance < blendRadius ? 1.0 : 0.0);

                weights.push_back(taper);

                if (taper > 0)
                {
                    const double energy = static_cast<double>(c.distance) * c.distance;
                    minEnergy           = std::min(minEnergy, energy);
                }
            }

            if (minEnergy == std::numeric_limits<double>::max())
            {
                continue;  // no candidate inside the blending window
            }

            // Offsetting all energies by the smallest one is exactly cancelled
            // by the normalization below, and it keeps exp() from underflowing
            // to zero for every candidate at small temperatures.
            double               sumW = 0;
            mrpt::math::TPoint3D blendedCentroid(0, 0, 0);
            Eigen::Matrix3d      normalTensor = Eigen::Matrix3d::Zero();

            for (size_t k = 0; k < candidates.size(); k++)
            {
                if (weights[k] <= 0)
                {
                    continue;
                }
                const auto&  c      = candidates[k];
                const double energy = static_cast<double>(c.distance) * c.distance - minEnergy;
                const double w      = weights[k] * std::exp(-energy * invTwoTSqr);
                weights[k]          = w;
                if (w <= 0)
                {
                    continue;
                }

                sumW += w;

                blendedCentroid.x += w * c.pairing.pl_global.centroid.x;
                blendedCentroid.y += w * c.pairing.pl_global.centroid.y;
                blendedCentroid.z += w * c.pairing.pl_global.centroid.z;

                // Sign-free accumulation of the normal directions:
                const auto&     co = c.pairing.pl_global.plane.coefs;
                Eigen::Vector3d n(co[0], co[1], co[2]);
                const double    nNorm = n.norm();
                if (nNorm <= 0)
                {
                    continue;
                }
                n /= nNorm;
                normalTensor.noalias() += w * n * n.transpose();
            }

            // The first test is the one that always has to be there: it is what
            // makes the normalization below safe.
            if (sumW <= 0 || sumW < minWeightSum)
            {
                continue;
            }

            blendedCentroid *= 1.0 / sumW;

            if (collectStats)
            {
                double sumSqr    = 0;
                double topW      = 0;
                double topDist   = 0;
                size_t nInWindow = 0;

                for (size_t k = 0; k < candidates.size(); k++)
                {
                    const double wn = weights[k] / sumW;
                    if (wn <= 0)
                    {
                        continue;
                    }
                    nInWindow++;
                    sumSqr += wn * wn;
                    if (wn > topW)
                    {
                        topW    = wn;
                        topDist = candidates[k].centroidDistance;
                    }
                }

                statQueries++;
                statEnumerated += candidates.size();
                statInWindow += nInWindow;
                // Inverse participation ratio: 1 for a pure argmin, and the
                // count of candidates when they contribute equally.
                statEffective += sumSqr > 0 ? 1.0 / sumSqr : 0.0;
                statTopWeight += topW;
                statWinnerDist += topDist;
            }

            // The blended direction is the dominant eigenvector of the weighted
            // structure tensor, which is what makes the result independent of
            // each candidate's arbitrary normal sign.
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(normalTensor);
            const Eigen::Vector3d blendedNormal = es.eigenvectors().col(2);

            const auto thePlane = mrpt::math::TPlane::FromPointAndNormal(
                blendedCentroid, {blendedNormal.x(), blendedNormal.y(), blendedNormal.z()});

            if (thePlane.distance(mrpt::math::TPoint3D(lx, ly, lz)) > distanceThreshold)
            {
                continue;  // blended plane is too distant
            }

            targetPlane.plane    = thePlane;
            targetPlane.centroid = blendedCentroid;
        }

        // OK, all conditions pass: add the new pairing:
        auto& p     = out.paired_pt2pl.emplace_back();
        p.pt_local  = {lxs[localIdx], lys[localIdx], lzs[localIdx]};
        p.pl_global = targetPlane;

        // Mark local point as already paired:
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);

    }  // For each local point

    if (collectStats && statQueries > 0)
    {
        const double inv = 1.0 / static_cast<double>(statQueries);
        MRPT_LOG_DEBUG_FMT(
            "blendstats T=%g R=%g queries=%zu enumerated=%.3f inWindow=%.3f effective=%.3f "
            "topWeight=%.4f winnerCentroidDist=%.4f",
            temperature, blendRadius, statQueries, statEnumerated * inv, statInWindow * inv,
            statEffective * inv, statTopWeight * inv, statWinnerDist * inv);
    }

    MRPT_END
}
