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
 * @file   update_velocity_buffer_from_obs.h
 * @brief  Utility to parse CObservationComments with local velocity metadata
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2026
 */
#pragma once

#include <mrpt/obs/CObservation.h>

// Including this header requires having MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION
#include <mola_imu_preintegration/LocalVelocityBuffer.h>

namespace mp2p_icp
{

/**
 * @brief Updates a LocalVelocityBuffer instance from a given observation.
 *
 * This function inspects the provided MRPT observation and, if it contains
 * a YAML-encoded comment with a `"local_velocity_buffer"` section, parses
 * and loads that data into the given `mola::LocalVelocityBuffer` object.
 *
 * This is a required step in sm2mm() and may be reused in other parts
 * while parsing simple-maps and there is a need to run deskew filters.
 *
 * - Observations of type `mrpt::obs::CObservationComment` may include
 *   auxiliary YAML data under the key `"local_velocity_buffer"`.
 * - This function safely checks for that data, parses it, and updates
 *   the `localVelocityBuffer` contents accordingly.
 * - If the observation is not a comment, or does not contain valid YAML,
 *   or lacks the `"local_velocity_buffer"` entry, the function silently
 *   returns without modifying `localVelocityBuffer`.
 *
 * Error handling:
 * - YAML parsing errors or malformed data structures are caught internally.
 * - In such cases, the function prints an error message to `std::cerr` and
 *   leaves `localVelocityBuffer` unmodified.
 *
 * @note This functionality is only available if
 *       `MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION` is defined at compile time.
 *       Otherwise, the function performs no operation.
 *
 * @param localVelocityBuffer Reference to a `mola::LocalVelocityBuffer` instance that will
 *        be updated if valid velocity buffer data is found in the observation.
 * @param obs Shared pointer to the MRPT observation to be inspected.
 */
void update_velocity_buffer_from_obs(
    mola::imu::LocalVelocityBuffer& localVelocityBuffer, const mrpt::obs::CObservation::Ptr& obs);

}  // namespace mp2p_icp
