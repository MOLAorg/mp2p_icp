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
 * @file   Matcher_Cov2Cov.cpp
 * @brief  Point-to-point with associated local covariance matcher
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2025
 */

#include <mp2p_icp/Matcher_Cov2Cov.h>
#include <mp2p_icp/NearestPointWithCovCapable.h>
#include <mrpt/core/bits_math.h>

#include <algorithm>
#include <cmath>

using namespace mp2p_icp;

namespace
{
/** |cos| of the angle between the beam and the surface normal.
 *
 * The pairing's information matrix is the inverse of a sum of two point
 * covariances. On a locally planar neighborhood that inverse is largest along
 * the surface normal and small in the two tangential directions, so its
 * dominant eigenvector IS the normal and the normalized quadratic form along
 * the beam direction approximates cos^2 of the incidence angle, without
 * needing an eigendecomposition per correspondence.
 *
 * An isotropic neighborhood, which has no meaningful normal, yields 1/3 for
 * every direction, so it lands mid-scale and is left essentially unweighted
 * rather than being assigned a spurious orientation.
 */
[[nodiscard]] double incidenceCosine(
    const mrpt::math::CMatrixFloat33& covInv, const mrpt::math::TPoint3Df& localPt, double range)
{
    const double trace = covInv(0, 0) + covInv(1, 1) + covInv(2, 2);
    if (trace <= 0 || range < 1e-3)
    {
        return 1.0;  // no information: do not reweight
    }

    const double ux = localPt.x / range;
    const double uy = localPt.y / range;
    const double uz = localPt.z / range;

    const double quad = ux * (covInv(0, 0) * ux + covInv(0, 1) * uy + covInv(0, 2) * uz) +
                        uy * (covInv(1, 0) * ux + covInv(1, 1) * uy + covInv(1, 2) * uz) +
                        uz * (covInv(2, 0) * ux + covInv(2, 1) * uy + covInv(2, 2) * uz);

    return std::sqrt(std::clamp(quad / trace, 0.0, 1.0));
}
}  // namespace

IMPLEMENTS_MRPT_OBJECT(Matcher_Cov2Cov, Matcher, mp2p_icp)

bool Matcher_Cov2Cov::impl_match(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, [[maybe_unused]] const MatchContext& mc,
    [[maybe_unused]] MatchState& ms, Pairings& out) const
{
    MRPT_START

    out = Pairings();

    // Analyze layer pairs:
    for (const auto& [globalLayerName, localLayerName] : layer_matches)
    {
        auto itLocal = pcLocal.layers.find(localLayerName);
        if (itLocal == pcLocal.layers.end())
        {
            THROW_EXCEPTION_FMT(
                "Local layer '%s' not found trying to matching global layer '%s'",
                localLayerName.c_str(), globalLayerName.c_str());
        }

        auto itGlobal = pcGlobal.layers.find(globalLayerName);
        if (itGlobal == pcGlobal.layers.end())
        {
            THROW_EXCEPTION_FMT(
                "Global layer '%s' not found trying to matching local layer '%s'",
                globalLayerName.c_str(), localLayerName.c_str());
        }

        const mrpt::maps::CMetricMap::Ptr& glLayerMap = itGlobal->second;
        ASSERT_(glLayerMap);
        const auto glLayer =
            std::dynamic_pointer_cast<mp2p_icp::NearestPointWithCovCapable>(glLayerMap);
        if (!glLayer)
        {
            THROW_EXCEPTION_FMT(
                "Global layer map must implement mp2p_icp::NearestPointWithCovCapable, but "
                "found type '%s'",
                glLayerMap->GetRuntimeClass()->className);
        }

        const mrpt::maps::CMetricMap::Ptr& lcLayerMap = itLocal->second;
        ASSERT_(lcLayerMap);
        const auto lcLayer =
            std::dynamic_pointer_cast<mp2p_icp::NearestPointWithCovCapable>(lcLayerMap);
        if (!lcLayer)
        {
            THROW_EXCEPTION_FMT(
                "Local layer map must implement mp2p_icp::NearestPointWithCovCapable, but "
                "found type '%s'",
                lcLayerMap->GetRuntimeClass()->className);
        }

        out.potential_pairings += lcLayer->point_count();

        // matcher implementation:
        const size_t firstNewPairing = out.paired_cov2cov.size();

        glLayer->nn_search_cov2cov(
            *lcLayer, localPose, matchingDistanceProfile(), out.paired_cov2cov);

        // Optional per-point weighting by range. `cov_inv` is the information
        // matrix of the correspondence, so scaling it IS the weight: the
        // solver multiplies both the gradient and the Hessian contribution by
        // it, and whitens the robust-kernel argument with the same factor.
        //
        // The range is taken in the local point's own untransformed frame,
        // i.e. from the sensor, matching how the range-adaptive matching
        // distance above defines it.
        const auto wRange     = pointWeightByRange();
        const auto wIncidence = pointWeightByIncidence();

        if (wRange.enabled() || wIncidence.enabled())
        {
            for (size_t i = firstNewPairing; i < out.paired_cov2cov.size(); i++)
            {
                auto&        pairing = out.paired_cov2cov[i];
                const auto&  lp      = pairing.local;
                const double range   = std::sqrt(
                    mrpt::square(lp.x) + mrpt::square(lp.y) + mrpt::square(lp.z));

                double weight = 1.0;

                if (wRange.enabled())
                {
                    weight *= wRange(range);
                }

                if (wIncidence.enabled())
                {
                    weight *= wIncidence(incidenceCosine(pairing.cov_inv, lp, range));
                }

                const float wf = static_cast<float>(weight);
                for (int r = 0; r < 3; r++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        pairing.cov_inv(r, c) *= wf;
                    }
                }
            }
        }
    }

    return true;
    MRPT_END
}

MatchingDistanceProfile Matcher_Cov2Cov::matchingDistanceProfile() const
{
    MatchingDistanceProfile p(threshold);  // flat, the default

    if (thresholdFar > 0 && thresholdFar != threshold)
    {
        p = MatchingDistanceProfile(
            threshold, thresholdFar, thresholdKneeRange, thresholdTransitionWidth);
    }

    return p;
}

PointWeightByIncidence Matcher_Cov2Cov::pointWeightByIncidence() const
{
    PointWeightByIncidence w;
    w.alpha     = incidenceWeightAlpha;
    w.refCos    = incidenceWeightRefCos;
    w.minWeight = incidenceWeightMin;
    w.maxWeight = incidenceWeightMax;
    return w;
}

PointWeightByRange Matcher_Cov2Cov::pointWeightByRange() const
{
    PointWeightByRange w;
    w.alpha     = pointWeightAlpha;
    w.refRange  = pointWeightRefRange;
    w.minWeight = pointWeightMin;
    w.maxWeight = pointWeightMax;
    return w;
}

void Matcher_Cov2Cov::initialize(const mrpt::containers::yaml& params)
{
    Matcher::initialize(params);

    DECLARE_PARAMETER_REQ(params, threshold);
    // These must be declared (not MCP_LOAD_OPT'd) so they accept dynamic
    // formulas just like `threshold` does. A plain static load silently
    // truncates an expression such as "3.0*ADAPTIVE_THRESHOLD_SIGMA" at the
    // first non-numeric character, yielding a fixed value in meters.
    DECLARE_PARAMETER_OPT(params, thresholdFar);
    DECLARE_PARAMETER_OPT(params, thresholdKneeRange);
    DECLARE_PARAMETER_OPT(params, thresholdTransitionWidth);
    DECLARE_PARAMETER_OPT(params, incidenceWeightAlpha);
    DECLARE_PARAMETER_OPT(params, incidenceWeightRefCos);
    DECLARE_PARAMETER_OPT(params, incidenceWeightMin);
    DECLARE_PARAMETER_OPT(params, incidenceWeightMax);
    DECLARE_PARAMETER_OPT(params, pointWeightAlpha);
    DECLARE_PARAMETER_OPT(params, pointWeightRefRange);
    DECLARE_PARAMETER_OPT(params, pointWeightMin);
    DECLARE_PARAMETER_OPT(params, pointWeightMax);
    MCP_LOAD_OPT(params, bounding_box_intersection_check_epsilon);

    if (params.has("layerMatches"))
    {
        const auto& p = params["layerMatches"];

        layer_matches.clear();
        ASSERT_(p.isSequence());

        // - {global: "raw", local: "decimated"}
        // - {global: "raw", local: "decimated"}
        // ...

        for (const auto& entry : p.asSequence())
        {
            ASSERT_(entry.isMap());
            const mrpt::containers::yaml em(entry);

            ASSERT_(em.has("global"));
            ASSERT_(em.has("local"));

            const std::string globalLayer = em["global"].as<std::string>();
            const std::string localLayer  = em["local"].as<std::string>();

            layer_matches.emplace_back(globalLayer, localLayer);
        }
    }
}
