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
 * @file   FilterAbsoluteTimestamp.h
 * @brief  Creates a new double-precision field with absolute timestamps.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 6, 2026
 */

#include <mp2p_icp_filters/FilterAbsoluteTimestamp.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/datetime.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(FilterAbsoluteTimestamp, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterAbsoluteTimestamp::FilterAbsoluteTimestamp() = default;

void FilterAbsoluteTimestamp::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    MCP_LOAD_OPT(c, output_field_name);
}

void FilterAbsoluteTimestamp::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c);
}

void FilterAbsoluteTimestamp::filter(mp2p_icp::metric_map_t& inOut) const
{
    auto pcPtr = inOut.layer<mrpt::maps::CPointsMap>(params.pointcloud_layer);
    if (!pcPtr || pcPtr->empty())
    {
        return;
    }

    auto& pc = *pcPtr;

    // 1. Get the relative time channel "t"
    const auto* ptrT = pc.getPointsBufferRef_float_field("t");
    if (!ptrT)
    {
        throw std::runtime_error(
            "[FilterAbsoluteTimestamp] Input layer '" + params.pointcloud_layer +
            "' does not have a 't' channel.");
    }

    // 2. Get reference base time from the context
    auto ps = this->attachedSource();
    if (!ps)
    {
        throw std::runtime_error(
            "[FilterAbsoluteTimestamp] No processing context (source) "
            "attached to the filter.");
    }

    // Ensure the field exists and get a reference to the buffer
#if MRPT_VERSION >= 0x020f04 && defined MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION
    const double refTimeSec = ps->localVelocityBuffer.get_reference_zero_time();

    // 3. Register and populate the new double channel
    const size_t N = pc.size();

    // Start with the first point without this new field, in the case of a layer accumulating points
    // over time from several scans.
    std::optional<size_t> startIdx;

    if (!pc.hasPointField(params.output_field_name))
    {
        pc.registerField_double(params.output_field_name);
        startIdx = 0;
    }

    auto absT_ptr = pc.getPointsBufferRef_double_field(params.output_field_name);
    ASSERT_(absT_ptr);
    auto& absT = *absT_ptr;

    if (!startIdx)
    {
        startIdx = absT.size();  // start after the last current point
    }

    absT.resize(N);

    const auto& relT = *ptrT;
    for (size_t i = *startIdx; i < N; i++)
    {
        absT[i] = refTimeSec + static_cast<double>(relT[i]);
    }
#else
    THROW_EXCEPTION("This filter requires MRPT>=2.15.4 and mola_imu_preintegration module.");
#endif
}