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
 * @file   FilterCurvature.cpp
 * @brief  Classifies a sorted input point cloud by local curvature
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2023
 */

#include <mp2p_icp_filters/FilterCurvature.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>

// #define DEBUG_GL

#ifdef DEBUG_GL
#include <mrpt/img/color_maps.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/Scene.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterCurvature, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterCurvature::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    MCP_LOAD_REQ(c, max_cosine);
    MCP_LOAD_REQ(c, min_clearance);
    MCP_LOAD_REQ(c, max_gap);

    MCP_LOAD_OPT(c, output_layer_larger_curvature);
    MCP_LOAD_OPT(c, output_layer_smaller_curvature);
    MCP_LOAD_OPT(c, output_layer_other);
}

FilterCurvature::FilterCurvature() = default;

void FilterCurvature::initialize(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);
}

void FilterCurvature::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Outputs:
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPcLarger = GetOrCreatePointLayer(
        inOut, params_.output_layer_larger_curvature,
        /*allow empty name for nullptr*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPcLarger) outPcLarger->reserve(outPcLarger->size() + pc.size() / 10);

    mrpt::maps::CPointsMap::Ptr outPcSmaller = GetOrCreatePointLayer(
        inOut, params_.output_layer_smaller_curvature,
        /*allow empty name for nullptr*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPcSmaller) outPcSmaller->reserve(outPcSmaller->size() + pc.size() / 10);

    mrpt::maps::CPointsMap::Ptr outPcOther = GetOrCreatePointLayer(
        inOut, params_.output_layer_other,
        /*allow empty name for nullptr*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPcOther) outPcOther->reserve(outPcOther->size() + pc.size() / 10);

    ASSERTMSG_(
        outPcLarger || outPcSmaller,
        "At least one 'output_layer_larger_curvature' or "
        "'output_layer_smaller_curvature' output layers must be provided.");

    const auto& xs       = pc.getPointsBufferRef_x();
    const auto& ys       = pc.getPointsBufferRef_y();
    const auto& zs       = pc.getPointsBufferRef_z();
    const auto* ptrRings = pc.getPointsBufferRef_ring();
    if (!ptrRings || ptrRings->empty())
    {
        THROW_EXCEPTION_FMT(
            "Error: this filter needs the input layer '%s' to has a 'ring' "
            "point channel.",
            params_.input_pointcloud_layer.c_str());
    }

    const auto& ringPerPt = *ptrRings;
    ASSERT_EQUAL_(ringPerPt.size(), xs.size());

    const size_t N = xs.size();

    const uint16_t nRings = 1 + *std::max_element(ringPerPt.begin(), ringPerPt.end());

    const auto estimPtsPerRing = N / nRings;

    MRPT_LOG_DEBUG_STREAM("nRings: " << nRings << " estimPtsPerRing: " << estimPtsPerRing);
    ASSERT_(nRings > 0 && nRings < 5000 /*something wrong?*/);

    std::vector<std::vector<size_t>> idxPerRing;
    idxPerRing.resize(nRings);
    for (auto& r : idxPerRing) r.reserve(estimPtsPerRing);

#ifdef DEBUG_GL
    auto glPts = mrpt::opengl::CPointCloudColoured::Create();
    glPts->setPointSize(4.0f);
    auto glRawPts = mrpt::opengl::CPointCloudColoured::Create();
    glRawPts->setPointSize(1.0f);
#endif

    for (size_t i = 0; i < N; i++)
    {
        auto& trg = idxPerRing.at(ringPerPt[i]);

#ifdef DEBUG_GL
        auto ringId = ringPerPt[i];
        auto col    = mrpt::img::colormap(mrpt::img::cmJET, static_cast<double>(ringId) / nRings);
        glRawPts->insertPoint({xs[i], ys[i], zs[i], col.R, col.G, col.B});
#endif

        if (!trg.empty())
        {
            // filter: minimum distance:
            auto       li     = trg.back();
            const auto lastPt = mrpt::math::TPoint3Df(xs[li], ys[li], zs[li]);
            const auto pt     = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);
            const auto d      = pt - lastPt;

            if (mrpt::max3(std::abs(d.x), std::abs(d.y), std::abs(d.z)) < params_.min_clearance)
                continue;
        }

        // accept the point:
        trg.push_back(i);

#ifdef DEBUG_GL
        glPts->insertPoint({xs[i], ys[i], zs[i], col.R, col.G, col.B});
#endif
    }

#ifdef DEBUG_GL
    {
        static int          iter = 0;
        mrpt::opengl::Scene scene;
        scene.insert(glRawPts);
        scene.insert(glPts);
        scene.saveToFile(mrpt::format("debug_curvature_%04i.3Dscene", iter++));
    }
#endif

    const float maxGapSqr = mrpt::square(params_.max_gap);

    size_t counterLarger = 0, counterLess = 0;

    for (size_t ri = 0; ri < nRings; ri++)
    {
        const auto& idxs = idxPerRing.at(ri);

        if (idxs.size() <= 3)
        {
            // If we have too few points, just accept them as they are so few we
            // cannot run the clasification method below.
            for (size_t idx = 0; idx < idxs.size(); idx++)
            {
                const size_t i = idxs[idx];
                counterLarger++;
                if (outPcLarger) outPcLarger->insertPointFrom(pc, i);
            }
            continue;
        }

        // Regular algorithm:
        for (size_t idx = 0; idx < idxs.size(); idx++)
        {
            const size_t im1 = idxs[idx > 0 ? idx - 1 : idxs.size() - 1];
            const size_t i   = idxs[idx];
            const size_t ip1 = idxs[idx < idxs.size() - 1 ? idx + 1 : 0];

            const auto pt   = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);
            const auto ptm1 = mrpt::math::TPoint3Df(xs[im1], ys[im1], zs[im1]);
            const auto ptp1 = mrpt::math::TPoint3Df(xs[ip1], ys[ip1], zs[ip1]);

            if ((pt - ptm1).sqrNorm() > maxGapSqr || (pt - ptp1).sqrNorm() > maxGapSqr)
            {
                // count borders as large curvature, if this is the edge
                // of the discontinuity that is closer to the sensor (assumed to
                // be close to the origin!)
                if (pt.sqrNorm() < ptm1.sqrNorm())
                {
                    counterLarger++;
                    if (outPcLarger) outPcLarger->insertPointFrom(pc, i);
                }
                else
                {
                    if (outPcOther) outPcOther->insertPointFrom(pc, i);
                }
                continue;
            }

            const auto v1  = (pt - ptm1);
            const auto v2  = (ptp1 - pt);
            const auto v1n = v1.norm();
            const auto v2n = v2.norm();

            const float score = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

            if (std::abs(score) < params_.max_cosine * v1n * v2n)
            {
                counterLarger++;
                if (outPcLarger) outPcLarger->insertPointFrom(pc, i);
            }
            else
            {
                counterLess++;
                if (outPcSmaller) outPcSmaller->insertPointFrom(pc, i);
            }
        }
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterCurvature] Raw input points=" << N << " larger_curvature=" << counterLarger
                                              << " smaller_curvature=" << counterLess);

    MRPT_END
}
