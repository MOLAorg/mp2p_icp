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
}  // namespace

Matcher_NDT_Blend::Matcher_NDT_Blend()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_NDT_Blend");
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

    const mp2p_icp::NearestPlaneCapable& nnGlobal =
        *mp2p_icp::MapToNP(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty())
    {
        return;
    }

    const TransformedLocalPointCloud tl = transform_local_to_global(pcLocal, localPose);

    // Blending window. Falling back to distanceThreshold keeps the parameter
    // set minimal for maps whose plane models live at the query's own scale.
    const double blendRadius = searchRadius > 0 ? searchRadius : distanceThreshold;

    // The map enumerates candidates by cell index, so the region it walks must
    // cover the whole taper. Otherwise a candidate could still hold a nonzero
    // weight at the moment it drops out of the enumeration.
    const auto enumRadius = static_cast<float>(std::max(distanceThreshold, blendRadius));

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

    const bool   blending   = temperature > 0;
    const double invTwoTSqr = blending ? 0.5 / (temperature * temperature) : 0;

    std::vector<NearestPlaneCapable::PlaneCandidate> candidates;
    std::vector<double>                              weights;

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

    MRPT_END
}
