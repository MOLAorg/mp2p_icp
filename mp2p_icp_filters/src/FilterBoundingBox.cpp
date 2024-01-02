/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBoundingBox.cpp
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 10, 2021
 */

#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterBoundingBox, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterBoundingBox::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterBoundingBox& parent)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    MCP_LOAD_REQ(c, keep_bbox_contents);

    ASSERT_(
        c.has("bounding_box_min") && c["bounding_box_min"].isSequence() &&
        c["bounding_box_min"].asSequence().size() == 3);
    ASSERT_(
        c.has("bounding_box_max") && c["bounding_box_max"].isSequence() &&
        c["bounding_box_max"].asSequence().size() == 3);

    const auto bboxMin = c["bounding_box_min"].asSequence();
    const auto bboxMax = c["bounding_box_max"].asSequence();

    for (int i = 0; i < 3; i++)
    {
        parent.parseAndDeclareParameter(
            bboxMin.at(i).as<std::string>(), bounding_box.min[i]);
        parent.parseAndDeclareParameter(
            bboxMax.at(i).as<std::string>(), bounding_box.max[i]);
    }
}

FilterBoundingBox::FilterBoundingBox()
{
    mrpt::system::COutputLogger::setLoggerName("FilterBoundingBox");
}

void FilterBoundingBox::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterBoundingBox::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPc = GetOrCreatePointLayer(
        inOut, params_.output_pointcloud_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    outPc->reserve(outPc->size() + pc.size() / 10);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    for (size_t i = 0; i < xs.size(); i++)
    {
        const bool isInside =
            params_.bounding_box.containsPoint({xs[i], ys[i], zs[i]});

        if ((isInside && !params_.keep_bbox_contents) ||
            (!isInside && params_.keep_bbox_contents))
            continue;  // remove this point.

        // Otherwise, add it:
        outPc->insertPointFrom(pc, i);
    }
    outPc->mark_as_modified();

    MRPT_END
}
