/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_velocity_buffer.cpp
 * @brief  Unit tests for
 * @author Jose Luis Blanco Claraco
 * @date   Aug 16, 2025
 */

#include <mp2p_icp/LocalVelocityBuffer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>

#include <iostream>

namespace
{
void unit_test()
{
    mp2p_icp::LocalVelocityBuffer buffer;

    const double reference_time = 1755345252;

    // Add some velocities
    buffer.add_linear_velocity(reference_time, {1.0, 2.0, 3.0});
    buffer.add_angular_velocity(reference_time, {0.1, 0.2, 0.3});

    // Check the contents
    ASSERT_EQUAL_(buffer.get_linear_velocities().size(), 1);
    ASSERT_EQUAL_(buffer.get_angular_velocities().size(), 1);

    // Check the reference time
    buffer.set_reference_zero_time(reference_time);
    ASSERT_EQUAL_(buffer.get_reference_zero_time(), reference_time);

    // Clear the buffer and check it's empty
    buffer.clear();
    ASSERT_EQUAL_(buffer.get_linear_velocities().size(), 0);
    ASSERT_EQUAL_(buffer.get_angular_velocities().size(), 0);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
