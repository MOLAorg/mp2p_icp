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
 * @file   FilterTransformPointCloud.cpp
 * @brief  Rigidly transforms a point cloud layer by a pose, or its inverse.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#include <mp2p_icp/pointcloud_field_utils.h>
#include <mp2p_icp_filters/FilterTransformPointCloud.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>

IMPLEMENTS_MRPT_OBJECT(FilterTransformPointCloud, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterTransformPointCloud::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterTransformPointCloud& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    MCP_LOAD_OPT(c, invert_pose);

    ASSERTMSG_(
        input_pointcloud_layer != output_pointcloud_layer,
        "input_pointcloud_layer and output_pointcloud_layer must be different "
        "(this filter does not transform a layer in place).");

    ASSERT_(c.has("pose"));
    ASSERT_(c["pose"].isSequence() && c["pose"].asSequence().size() == 6);
    auto cc = c["pose"].asSequence();
    for (int i = 0; i < 6; i++)
    {
        parent.parseAndDeclareParameter(cc.at(i).as<std::string>(), pose[i]);
    }
}

FilterTransformPointCloud::FilterTransformPointCloud()
{
    mrpt::system::COutputLogger::setLoggerName("FilterTransformPointCloud");
}

void FilterTransformPointCloud::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterTransformPointCloud::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    ASSERTMSG_(
        inOut.layers.count(params.input_pointcloud_layer) != 0,
        mrpt::format("Input layer '%s' not found.", params.input_pointcloud_layer.c_str()));

    const auto mapPtr = inOut.layers.at(params.input_pointcloud_layer);
    ASSERT_(mapPtr);

    const mrpt::maps::CPointsMap* pcPtr = mp2p_icp::MapToPointsMap(*mapPtr);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' could not be converted into a "
                   "point cloud (class='%s')",
                   params.input_pointcloud_layer.c_str(), mapPtr->GetRuntimeClass()->className));

    // Out: always (re)create a fresh layer of the same class as the input,
    // so repeated calls (one per scan) never accumulate stale points.
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params.output_pointcloud_layer,
        /*do not allow empty*/ false,
        /* create cloud of the same type */ pcPtr->GetRuntimeClass()->className);
    outPc->clear();

    const auto basePose  = mrpt::poses::CPose3D(params.pose);
    const auto appliedTf = params.invert_pose ? -basePose : basePose;

    // insertAnotherMap() transforms XYZ by the given pose and copies all
    // other registered fields verbatim; view-direction unit vectors
    // (view_x/y/z), if present, must be rotated too or they end up expressed
    // in the wrong frame (see FilterMerge, which has the same requirement).
    outPc->insertAnotherMap(pcPtr, appliedTf);
    mp2p_icp::rotateViewDirectionFields(*outPc, appliedTf);

    MRPT_END
}
