/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByRange.cpp
 * @brief  Leaves or removes points by min/max range from the origin.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 14, 2023
 */

#include <mp2p_icp_filters/FilterByRange.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterByRange, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByRange::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterByRange& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, output_layer_between);
    MCP_LOAD_OPT(c, output_layer_outside);
    DECLARE_PARAMETER_IN_REQ(c, range_min, parent);
    DECLARE_PARAMETER_IN_REQ(c, range_max, parent);

    ASSERTMSG_(
        !output_layer_between.empty() || !output_layer_outside.empty(),
        "At least one 'output_layer_between' or "
        "'output_layer_outside' must be provided.");

    if (c.has("center"))
    {
        ASSERT_(
            c["center"].isSequence() && c["center"].asSequence().size() == 3);

        auto cc = c["center"].asSequence();

        for (int i = 0; i < 3; i++)
            parent.parseAndDeclareParameter(
                cc.at(i).as<std::string>(), center[i]);
    }
}

FilterByRange::FilterByRange()
{
    mrpt::system::COutputLogger::setLoggerName("FilterByRange");
}

void FilterByRange::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterByRange::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    const auto pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outBetween = GetOrCreatePointLayer(
        inOut, params_.output_layer_between, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outBetween) outBetween->reserve(outBetween->size() + pc.size() / 10);

    // Optional output layer for deleted points:
    mrpt::maps::CPointsMap* outOutside = GetOrCreatePointLayer(
        inOut, params_.output_layer_outside, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outOutside) outOutside->reserve(outOutside->size() + pc.size() / 10);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const float sqrMin = mrpt::square(params_.range_min);
    const float sqrMax = mrpt::square(params_.range_max);

    for (size_t i = 0; i < xs.size(); i++)
    {
        const float sqrNorm =
            (mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]) - params_.center)
                .sqrNorm();

        const bool isInside = sqrNorm >= sqrMin && sqrNorm <= sqrMax;

        auto* targetPc = isInside ? outBetween : outOutside;

        if (targetPc) targetPc->insertPointFrom(pc, i);
    }

    MRPT_END
}
