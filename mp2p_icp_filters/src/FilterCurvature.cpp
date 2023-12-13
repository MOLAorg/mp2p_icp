/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterCurvature.cpp
 * @brief  Classifies a sorted input point cloud by local curvature
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2023
 */

#include <mp2p_icp_filters/FilterCurvature.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterCurvature, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterCurvature::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
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
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Outputs:
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPcLarger = GetOrCreatePointLayer(
        inOut, params_.output_layer_larger_curvature,
        true /*allow empty name for nullptr*/);
    if (outPcLarger) outPcLarger->reserve(outPcLarger->size() + pc.size() / 10);

    mrpt::maps::CPointsMap* outPcSmaller = GetOrCreatePointLayer(
        inOut, params_.output_layer_smaller_curvature,
        true /*allow empty name for nullptr*/);
    if (outPcSmaller)
        outPcSmaller->reserve(outPcSmaller->size() + pc.size() / 10);

    mrpt::maps::CPointsMap* outPcOther = GetOrCreatePointLayer(
        inOut, params_.output_layer_other,
        true /*allow empty name for nullptr*/);
    if (outPcOther) outPcOther->reserve(outPcOther->size() + pc.size() / 10);

    ASSERTMSG_(
        outPcLarger || outPcSmaller,
        "At least one 'output_layer_larger_curvature' or "
        "'output_layer_smaller_curvature' output layers must be provided.");

    const auto&  xs = pc.getPointsBufferRef_x();
    const auto&  ys = pc.getPointsBufferRef_y();
    const auto&  zs = pc.getPointsBufferRef_z();
    const size_t N  = xs.size();

    const float maxGapSqr = mrpt::square(params_.max_gap);

    size_t counterLarger = 0, counterLess = 0;

    for (size_t i = 1; i + 1 < N; i++)
    {
        const auto pt = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);
        const auto ptm1 =
            mrpt::math::TPoint3Df(xs[i - 1], ys[i - 1], zs[i - 1]);
        const auto ptp1 =
            mrpt::math::TPoint3Df(xs[i + 1], ys[i + 1], zs[i + 1]);

        if ((pt - ptm1).sqrNorm() > maxGapSqr ||
            (pt - ptp1).sqrNorm() > maxGapSqr)
        {
            // count borders as large curvature, if this is the edge
            // of the discontinuity that is closer to the sensor (assumed to be
            // close to the origin!)
            if (pt.sqrNorm() < ptm1.sqrNorm())
            {
                counterLarger++;
                if (outPcLarger) outPcLarger->insertPoint(pt);
            }
            else
            {
                if (outPcOther) outPcOther->insertPoint(pt);
            }
            continue;
        }

        const auto v1  = (pt - ptm1);
        const auto v2  = (ptp1 - pt);
        const auto v1n = v1.norm();
        const auto v2n = v2.norm();

        if (v1n < params_.min_clearance || v2n < params_.min_clearance)
            continue;

        const float score = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

        if (std::abs(score) < params_.max_cosine * v1n * v2n)
        {
            counterLarger++;
            if (outPcLarger) outPcLarger->insertPoint(pt);
        }
        else
        {
            counterLess++;
            if (outPcSmaller) outPcSmaller->insertPoint(pt);
        }
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterCurvature] Raw input points="
        << N << " larger_curvature=" << counterLarger
        << " smaller_curvature=" << counterLess);

    MRPT_END
}
