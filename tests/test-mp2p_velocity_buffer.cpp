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
void unit_test_basic()
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

void unit_test_yaml()
{
    mp2p_icp::LocalVelocityBuffer buffer;

    const double reference_time = 1755345252;

    // Add some velocities
    buffer.add_linear_velocity(reference_time, {1.0, 2.0, 3.0});
    buffer.add_angular_velocity(reference_time, {0.1, 0.2, 0.3});
    buffer.add_linear_velocity(reference_time + 0.1, {4.0, 5.0, 6.0});
    buffer.add_angular_velocity(reference_time + 0.1, {0.4, 0.5, 0.6});

    buffer.set_reference_zero_time(reference_time);

    // Serialize to YAML
    const auto yaml    = buffer.toYAML();
    const auto yamlStr = [&]()
    {
        std::stringstream ss;
        ss << yaml;
        return ss.str();
    }();

    std::cout << "Serialized YAML:\n" << yamlStr << "\n\n";

    // Deserialize from YAML
    mp2p_icp::LocalVelocityBuffer buffer2;
    buffer2.fromYAML(mrpt::containers::yaml::FromText(yamlStr));

    std::cout << "Done parsing.\n";

    // Check that the contents are the same
    ASSERT_EQUAL_(buffer2.get_reference_zero_time(), buffer.get_reference_zero_time());
    ASSERT_EQUAL_(buffer2.get_linear_velocities().size(), buffer.get_linear_velocities().size());
    ASSERT_EQUAL_(buffer2.get_angular_velocities().size(), buffer.get_angular_velocities().size());

    for (const auto& [time, vel] : buffer.get_linear_velocities())
    {
        const auto it = buffer2.get_linear_velocities().find(time);
        ASSERT_(it != buffer2.get_linear_velocities().end());
        ASSERT_(it->second == vel);  // Assuming operator== is defined for LinearVelocity
    }

    for (const auto& [time, w] : buffer.get_angular_velocities())
    {
        const auto it = buffer2.get_angular_velocities().find(time);
        ASSERT_(it != buffer2.get_angular_velocities().end());
        ASSERT_(it->second == w);  // Assuming operator== is defined for AngularVelocity
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test_basic();
        unit_test_yaml();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
