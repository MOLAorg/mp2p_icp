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
 * @file   update_velocity_buffer_from_obs.cpp
 * @brief  Utility to parse CObservationComments with local velocity metadata
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2026
 */

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
#include <mp2p_icp/update_velocity_buffer_from_obs.h>
#endif

#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CObservationComment.h>

#include <iostream>

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)

void mp2p_icp::update_velocity_buffer_from_obs(
    mola::imu::LocalVelocityBuffer& localVelocityBuffer, const mrpt::obs::CObservation::Ptr& obs)
{
    auto obsComment = std::dynamic_pointer_cast<mrpt::obs::CObservationComment>(obs);
    if (!obsComment)
    {
        return;
    }

    const auto commentYaml = [&]()
    {
        try
        {
            return mrpt::containers::yaml::FromText(obsComment->text);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing YAML in comment: " << e.what() << std::endl;
            return mrpt::containers::yaml();
        }
    }();

    if (!commentYaml.isMap() || !commentYaml.has("local_velocity_buffer"))
    {
        return;
    }

    const auto lvb = commentYaml["local_velocity_buffer"];
    if (!lvb.isMap())
    {
        std::cerr << "Error: 'local_velocity_buffer' field is not a map!" << std::endl;
        return;
    }

    try
    {
        localVelocityBuffer.fromYAML(lvb);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error parsing 'local_velocity_buffer': " << e.what() << std::endl;
        return;
    }
}

#endif