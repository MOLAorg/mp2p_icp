/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterAdjustTimestamps.cpp
 * @brief  Normalizes point cloud timestamps
 * @author Jose Luis Blanco Claraco
 * @date   Jun 19, 2024
 */

#include <mp2p_icp_filters/FilterAdjustTimestamps.h>
#include <mrpt/containers/yaml.h>

#include <optional>

IMPLEMENTS_MRPT_OBJECT(FilterAdjustTimestamps, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterAdjustTimestamps::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterAdjustTimestamps& parent)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    MCP_LOAD_REQ(c, method);
    MCP_LOAD_REQ(c, silently_ignore_no_timestamps);

    DECLARE_PARAMETER_IN_OPT(c, time_offset, parent);
}

FilterAdjustTimestamps::FilterAdjustTimestamps()
{
    mrpt::system::COutputLogger::setLoggerName("FilterAdjustTimestamps");
}

void FilterAdjustTimestamps::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterAdjustTimestamps::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In/out:
    auto pcPtr = inOut.point_layer(params_.pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.pointcloud_layer.c_str()));

    auto& pc = *pcPtr;

    if (pc.empty())
    {
        MRPT_LOG_WARN_STREAM(
            "Skipping time adjusting in input cloud '" << params_.pointcloud_layer
                                                       << "' because it is empty.");
        return;
    }

    auto* TsPtr = pc.getPointsBufferRef_timestamp();

    if (TsPtr == nullptr || (TsPtr->empty() && !pc.empty()))
    {
        // we don't have timestamps:
        if (params_.silently_ignore_no_timestamps)
        {
            MRPT_LOG_DEBUG_STREAM(
                "Skipping time adjusting in input cloud '" << params_.pointcloud_layer
                                                           << "' with contents: " << pc.asString()
                                                           << " due to missing timestamps.");
            return;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Cannot do time adjusting for input cloud '%s' "
                "with contents: %s due to missing timestamps.",
                params_.pointcloud_layer.c_str(), pc.asString().c_str());
        }
    }

    auto& Ts = *TsPtr;

    std::optional<float> minT, maxT;

    for (size_t i = 0; i < Ts.size(); i++)
    {
        const float t = Ts[i];

        if (!minT || t < *minT) minT = t;
        if (!maxT || t > *maxT) maxT = t;
    }
    ASSERT_(minT && maxT);

    switch (params_.method)
    {
        case TimestampAdjustMethod::MiddleIsZero:
        {
            const float dt = 0.5f * (*maxT + *minT) + params_.time_offset;
            for (auto& t : Ts) t -= dt;
        }
        break;
        case TimestampAdjustMethod::EarliestIsZero:
        {
            const float dt = *minT + params_.time_offset;
            for (auto& t : Ts) t -= dt;
        }
        break;
        case TimestampAdjustMethod::Normalize:
        {
            const float m = *minT;
            const float k = *maxT != *minT ? 1.0f / (*maxT - *minT) : 1.0f;
            for (auto& t : Ts) t = (t - m) * k + params_.time_offset;
        }
        break;

        default:
            THROW_EXCEPTION("Unknown value for 'method'");
    }

    MRPT_END
}
