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
 * @file   FilterMerge.cpp
 * @brief  Takes an input point cloud layer and inserts it into another one of
 * arbitrary metric map type.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2024
 */

#include <mp2p_icp/pointcloud_field_utils.h>
#include <mp2p_icp_filters/FilterMerge.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationPointCloud.h>

IMPLEMENTS_MRPT_OBJECT(FilterMerge, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterMerge::Parameters::load_from_yaml(const mrpt::containers::yaml& c, FilterMerge& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, target_layer);
    MCP_LOAD_OPT(c, input_layer_in_local_coordinates);

    if (c.has("robot_pose"))
    {
        ASSERT_(c["robot_pose"].isSequence() && c["robot_pose"].asSequence().size() == 6);

        auto cc = c["robot_pose"].asSequence();

        for (int i = 0; i < 6; i++)
        {
            parent.parseAndDeclareParameter(cc.at(i).as<std::string>(), robot_pose[i]);
        }
    }
}

FilterMerge::FilterMerge() { mrpt::system::COutputLogger::setLoggerName("FilterMerge"); }

void FilterMerge::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterMerge::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    ASSERTMSG_(
        inOut.layers.count(params.input_pointcloud_layer) != 0,
        mrpt::format("Input layer '%s' not found.", params.input_pointcloud_layer.c_str()));

    const auto mapPtr = inOut.layers.at(params.input_pointcloud_layer);
    ASSERT_(mapPtr);

    const auto pcPtr = mp2p_icp::MapToPointsMap(*mapPtr);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' could not be converted into a "
                   "point cloud (class='%s')",
                   params.input_pointcloud_layer.c_str(), mapPtr->GetRuntimeClass()->className));

    // Out:
    ASSERT_(!params.target_layer.empty());
    ASSERTMSG_(
        inOut.layers.count(params.target_layer) != 0,
        mrpt::format("Target map layer '%s' not found.", params.target_layer.c_str()));

    mrpt::maps::CMetricMap::Ptr out = inOut.layers.at(params.target_layer);

    const auto robotPose = mrpt::poses::CPose3D(params.robot_pose);

    // Transformation from the input layer frame to the robot frame (hence the "-"):
    const auto appliedTf =
        params.input_layer_in_local_coordinates ? mrpt::poses::CPose3D::Identity() : -robotPose;

    // Create fake observation for insertion:
    mrpt::obs::CObservationPointCloud obs;

    // Insertion only reads the observation, so the input cloud can be used as is if it does not
    // need to be transformed (and it is not the target map itself):
    auto inputCloud = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mapPtr);
    if (inputCloud && mapPtr != out && appliedTf == mrpt::poses::CPose3D::Identity())
    {
        obs.pointcloud = inputCloud;
    }
    else
    {
        // Create cloud of the same type than the input to avoid losing point fields:
        auto newCloudObj = pcPtr->GetRuntimeClass()->createObject();
        auto pts         = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(newCloudObj);
        ASSERTMSG_(pts, "Not a pointcloud class as input?");

        // insertAnotherMap() transforms the point XYZ coordinates by the given
        // pose, but copies all other registered fields verbatim. If the cloud
        // carries view-direction unit vectors (view_x/y/z), those must be
        // rotated too, or they will end up expressed in the wrong frame.
        pts->insertAnotherMap(pcPtr, appliedTf);
        mp2p_icp::rotateViewDirectionFields(*pts, appliedTf);

        obs.pointcloud = pts;
    }

    // Merge into map:
    out->insertObservation(obs, robotPose);

    MRPT_END
}
