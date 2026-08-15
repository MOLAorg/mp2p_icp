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
 * @file   Matcher_Points_KnnPlane.cpp
 * @brief  Pointcloud matcher: k nearest map points, a least-squares plane, and
 *         fixed acceptance gates
 * @author Jose Luis Blanco Claraco
 * @date   Aug 15, 2026
 */

#include <mp2p_icp/Matcher_Points_KnnPlane.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_KnnPlane, Matcher, mp2p_icp)

using namespace mp2p_icp;

namespace
{
/** Least-squares plane through `n` points, solving `A x = -1` with a
 *  column-pivoting Householder QR in single precision.
 *
 *  This mirrors the reference implementation rather than using an eigen/PCA
 *  fit. The two agree in exact arithmetic but not numerically, and they differ
 *  in kind when the points are near-degenerate: the QR form has no solution
 *  through the origin at all, which is part of the protocol's behavior.
 *
 *  Returns false if any supporting point lies further than `maxDeviation` from
 *  the resulting plane. On success `outPlane` holds (a,b,c,d) normalized so
 *  that a^2+b^2+c^2 = 1.
 */
bool fitPlaneQr(
    const std::vector<mrpt::math::TPoint3Df>& pts, const size_t n, const float maxDeviation,
    Eigen::Vector4f& outPlane)
{
    Eigen::Matrix<float, Eigen::Dynamic, 3> A(static_cast<Eigen::Index>(n), 3);
    Eigen::Matrix<float, Eigen::Dynamic, 1> b(static_cast<Eigen::Index>(n), 1);

    b.setOnes();
    b *= -1.0f;

    for (size_t j = 0; j < n; j++)
    {
        const auto i = static_cast<Eigen::Index>(j);
        A(i, 0)      = pts[j].x;
        A(i, 1)      = pts[j].y;
        A(i, 2)      = pts[j].z;
    }

    const Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);

    const float norm = normvec.norm();
    if (!(norm > 0) || !std::isfinite(norm))
    {
        return false;
    }

    outPlane(0) = normvec(0) / norm;
    outPlane(1) = normvec(1) / norm;
    outPlane(2) = normvec(2) / norm;
    outPlane(3) = 1.0f / norm;

    for (size_t j = 0; j < n; j++)
    {
        const float d =
            outPlane(0) * pts[j].x + outPlane(1) * pts[j].y + outPlane(2) * pts[j].z + outPlane(3);
        if (std::fabs(d) > maxDeviation)
        {
            return false;
        }
    }
    return true;
}
}  // namespace

Matcher_Points_KnnPlane::Matcher_Points_KnnPlane()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Points_KnnPlane");
}

void Matcher_Points_KnnPlane::initialize(const mrpt::containers::yaml& params)
{
    Matcher_Points_Base::initialize(params);

    DECLARE_PARAMETER_OPT(params, maxNeighborDistance);
    DECLARE_PARAMETER_OPT(params, planeFitMaxDeviation);
    DECLARE_PARAMETER_OPT(params, residualGateScale);
    DECLARE_PARAMETER_OPT(params, residualGateAccept);

    if (params.has("knn"))
    {
        knn = params["knn"].as<uint32_t>();
    }
    if (params.has("rangeNormalized"))
    {
        rangeNormalized = params["rangeNormalized"].as<bool>();
    }
}

void Matcher_Points_KnnPlane::implMatchOneLayer(
    const mrpt::maps::CMetricMap& pcGlobalMap, const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, MatchState& ms,
    [[maybe_unused]] const layer_name_t& globalName, const layer_name_t& localName,  // NOLINT
    Pairings& out) const
{
    MRPT_START

    checkAllParametersAreRealized();

    ASSERT_(knn >= 1);
    ASSERT_GT_(maxNeighborDistance, .0);

    const mrpt::maps::NearestNeighborsCapable& nnGlobal =
        *mp2p_icp::MapToNN(pcGlobalMap, true /*throw if cannot convert*/);

    out.potential_pairings += pcLocal.size();

    // Empty maps?  Nothing to do
    if (pcGlobalMap.isEmpty() || pcLocal.empty())
    {
        return;
    }

    const TransformedLocalPointCloud tl = transform_local_to_global(pcLocal, localPose);

    const auto maxNeighborDistance_f = static_cast<float>(maxNeighborDistance);

    // Try to do matching only if the bounding boxes have some overlap:
    if (!pcGlobalMap.boundingBox().intersection(
            {tl.localMin, tl.localMax},
            maxNeighborDistance_f + bounding_box_intersection_check_epsilon_))
    {
        return;
    }

    out.paired_pt2pl.reserve(out.paired_pt2pl.size() + pcLocal.size() / 10);

    const auto& lxs = pcLocal.getPointsBufferRef_x();
    const auto& lys = pcLocal.getPointsBufferRef_y();
    const auto& lzs = pcLocal.getPointsBufferRef_z();

    const float maxNeighborDistSqr     = maxNeighborDistance_f * maxNeighborDistance_f;
    const auto  planeFitMaxDeviation_f = static_cast<float>(planeFitMaxDeviation);

    nnGlobal.nn_prepare_for_3d_queries();

    std::vector<mrpt::math::TPoint3Df> neighborPts;
    std::vector<float>                 neighborSqrDists;
    std::vector<uint64_t>              neighborIndices;
    Eigen::Vector4f                    plane;

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

        const float lx = tl.x_locals[i], ly = tl.y_locals[i], lz = tl.z_locals[i];

        nnGlobal.nn_multiple_search(
            {lx, ly, lz}, knn, neighborPts, neighborSqrDists, neighborIndices);

        // Gate 1: enough neighbors at all.
        //
        // The size of the result is NOT a reliable answer to that question. A
        // k-NN query asked for more neighbors than the map holds can return
        // exactly k anyway, padding the tail by repeating an earlier point
        // under a fabricated zero distance. That padding would silently supply
        // a duplicated support point to the plane fit, at the map edges and in
        // sparse regions, which is precisely where this gate has to fire. So
        // each neighbor is validated against its own coordinates instead.
        if (neighborPts.size() < static_cast<size_t>(knn))
        {
            continue;
        }

        bool paddedResult = false;
        for (size_t k = 0; k < knn; k++)
        {
            const float dx      = neighborPts[k].x - lx;
            const float dy      = neighborPts[k].y - ly;
            const float dz      = neighborPts[k].z - lz;
            const float trueSqr = dx * dx + dy * dy + dz * dz;
            if (std::fabs(trueSqr - neighborSqrDists[k]) > 1e-4f * (1.0f + trueSqr))
            {
                paddedResult = true;
                break;
            }
        }
        if (paddedResult)
        {
            continue;
        }

        // Gate 2: the farthest supporting point is within range. The query
        // returns them sorted, but the maximum is taken explicitly so this does
        // not silently depend on that.
        float farthestSqr = 0;
        for (size_t k = 0; k < knn; k++)
        {
            farthestSqr = std::max(farthestSqr, neighborSqrDists[k]);
        }
        if (farthestSqr > maxNeighborDistSqr)
        {
            continue;
        }

        // Gate 3: the plane fit is good enough everywhere it is supported.
        if (!fitPlaneQr(neighborPts, knn, planeFitMaxDeviation_f, plane))
        {
            continue;
        }

        const float dist = plane(0) * lx + plane(1) * ly + plane(2) * lz + plane(3);

        // Gate 4: the residual, optionally normalized by the point's own range.
        // This is the only gate that adapts to anything, and it adapts to a
        // static property of the measurement, never to the registration's own
        // output.
        double s = 0;
        if (rangeNormalized)
        {
            const double range = std::sqrt(
                static_cast<double>(lxs[localIdx]) * lxs[localIdx] +
                static_cast<double>(lys[localIdx]) * lys[localIdx] +
                static_cast<double>(lzs[localIdx]) * lzs[localIdx]);
            if (!(range > 0))
            {
                continue;
            }
            s = 1.0 - residualGateScale * std::fabs(dist) / std::sqrt(range);
        }
        else
        {
            s = 1.0 - residualGateScale * std::fabs(dist);
        }

        if (!(s > residualGateAccept))
        {
            continue;
        }

        // All gates pass: add the new pairing.
        mrpt::math::TPoint3D centroid(0, 0, 0);
        for (size_t k = 0; k < knn; k++)
        {
            centroid.x += neighborPts[k].x;
            centroid.y += neighborPts[k].y;
            centroid.z += neighborPts[k].z;
        }
        centroid *= 1.0 / static_cast<double>(knn);

        auto& p    = out.paired_pt2pl.emplace_back();
        p.pt_local = {lxs[localIdx], lys[localIdx], lzs[localIdx]};

        p.pl_global.centroid = centroid;
        p.pl_global.plane =
            mrpt::math::TPlane(plane(0), plane(1), plane(2), static_cast<double>(plane(3)));

        // Mark local point as already paired. As in Matcher_Point2Plane, global
        // points are not marked: many local points may legitimately be
        // supported by the same neighborhood.
        ms.localPairedBitField.point_layers[localName].mark_as_set(localIdx);

    }  // For each local point

    MRPT_END
}
