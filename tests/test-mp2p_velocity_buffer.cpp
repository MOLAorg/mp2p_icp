/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   test-mp2p_velocity_buffer.cpp
 * @brief  Unit tests for
 * @author Jose Luis Blanco Claraco
 * @date   Aug 16, 2025
 */

#include <mp2p_icp/LocalVelocityBuffer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/system/os.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{
void unit_test_basic_api()
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

    std::cout << "✅ LocalVelocityBuffer unit_test_basic_api passed!" << std::endl;
}

void unit_test_basic_yaml()
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
    std::cout << "✅ LocalVelocityBuffer unit_test_basic_yaml passed!" << std::endl;
}

void unit_test_yaml_roundtrip()
{
    LocalVelocityBuffer buf;

    // Fill with sample data
    buf.parameters.max_time_window        = 2.0;
    buf.parameters.tolerance_search_stamp = 1e-2;
    buf.set_reference_zero_time(123.456);

    LocalVelocityBuffer::LinearVelocity     v{1.0, 2.0, 3.0};
    LocalVelocityBuffer::AngularVelocity    w{0.1, 0.2, 0.3};
    LocalVelocityBuffer::LinearAcceleration a{9.8, 0.0, -9.8};
    LocalVelocityBuffer::SO3                R = mrpt::math::CMatrixDouble33::Identity();

    buf.add_linear_velocity(123.400, v);
    buf.add_angular_velocity(123.410, w);
    buf.add_linear_acceleration(123.420, a);
    buf.add_orientation(123.430, R);

    // Serialize
    auto yml = buf.toYAML();

    yml.printAsYAML();

    // Deserialize into a new buffer
    LocalVelocityBuffer buf2;
    buf2.fromYAML(yml);

    // Checks
    ASSERT_EQUAL_(buf2.parameters.max_time_window, buf.parameters.max_time_window);
    ASSERT_EQUAL_(buf2.parameters.tolerance_search_stamp, buf.parameters.tolerance_search_stamp);
    ASSERT_EQUAL_(buf2.get_reference_zero_time(), buf.get_reference_zero_time());

    // linear velocities
    ASSERT_EQUAL_(buf2.get_linear_velocities().size(), buf.get_linear_velocities().size());
    for (const auto& [t, v1] : buf.get_linear_velocities())
    {
        const auto it = buf2.get_linear_velocities().find(t);
        ASSERT_(it != buf2.get_linear_velocities().end());
        ASSERT_(v1 == it->second);
    }

    // angular velocities
    ASSERT_EQUAL_(buf2.get_angular_velocities().size(), buf.get_angular_velocities().size());
    for (const auto& [t, w1] : buf.get_angular_velocities())
    {
        const auto it = buf2.get_angular_velocities().find(t);
        ASSERT_(it != buf2.get_angular_velocities().end());
        ASSERT_(w1 == it->second);
    }

    // linear accelerations
    ASSERT_EQUAL_(buf2.get_linear_accelerations().size(), buf.get_linear_accelerations().size());
    for (const auto& [t, a1] : buf.get_linear_accelerations())
    {
        const auto it = buf2.get_linear_accelerations().find(t);
        ASSERT_(it != buf2.get_linear_accelerations().end());
        ASSERT_(a1 == it->second);
    }

    // orientations
    ASSERT_EQUAL_(buf2.get_orientations().size(), buf.get_orientations().size());
    for (const auto& [t, R1] : buf.get_orientations())
    {
        const auto it = buf2.get_orientations().find(t);
        ASSERT_(it != buf2.get_orientations().end());
        ASSERT_(R1 == it->second);
    }

    std::cout << "✅ LocalVelocityBuffer unit_test_yaml_roundtrip passed!" << std::endl;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test_basic_api();
        unit_test_basic_yaml();
        unit_test_yaml_roundtrip();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
