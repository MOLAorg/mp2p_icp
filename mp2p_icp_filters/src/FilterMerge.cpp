/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterMerge.cpp
 * @brief  Takes an input point cloud layer and inserts it into another one of
 * arbitrary metric map type.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2024
 */

#include <mp2p_icp_filters/FilterMerge.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationPointCloud.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterMerge, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterMerge::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterMerge& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, target_layer);
    MCP_LOAD_OPT(c, input_layer_in_local_coordinates);

    if (c.has("robot_pose"))
    {
        ASSERT_(
            c["robot_pose"].isSequence() &&
            c["robot_pose"].asSequence().size() == 6);

        auto cc = c["robot_pose"].asSequence();

        for (int i = 0; i < 6; i++)
            parent.parseAndDeclareParameter(
                cc.at(i).as<std::string>(), robot_pose[i]);
    }
}

FilterMerge::FilterMerge()
{
    mrpt::system::COutputLogger::setLoggerName("FilterMerge");
}

void FilterMerge::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterMerge::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    ASSERTMSG_(
        inOut.layers.count(params_.input_pointcloud_layer) != 0,
        mrpt::format(
            "Input layer '%s' not found.",
            params_.input_pointcloud_layer.c_str()));

    const auto mapPtr = inOut.layers.at(params_.input_pointcloud_layer);
    ASSERT_(mapPtr);

    const auto pcPtr = mp2p_icp::MapToPointsMap(*mapPtr);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' could not be converted into a "
                   "point cloud (class='%s')",
                   params_.input_pointcloud_layer.c_str(),
                   mapPtr->GetRuntimeClass()->className));

    // Out:
    ASSERT_(!params_.target_layer.empty());
    ASSERTMSG_(
        inOut.layers.count(params_.target_layer) != 0,
        mrpt::format(
            "Target map layer '%s' not found.", params_.target_layer.c_str()));

    mrpt::maps::CMetricMap::Ptr out = inOut.layers.at(params_.target_layer);

    // Create fake observation for insertion:
    mrpt::obs::CObservationPointCloud obs;
    auto pts       = mrpt::maps::CSimplePointsMap::Create();
    obs.pointcloud = pts;

    // Copy the input layer here, as seen from the robot (hence the "-"):
    const auto robotPose = mrpt::poses::CPose3D(params_.robot_pose);

    if (params_.input_layer_in_local_coordinates)
    {
        pts->insertAnotherMap(pcPtr, mrpt::poses::CPose3D::Identity());
    }
    else
    {
        const auto invRobotPose = -robotPose;
        pts->insertAnotherMap(pcPtr, invRobotPose);
    }

    // Merge into map:
    out->insertObservation(obs, robotPose);

    MRPT_END
}
