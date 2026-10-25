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
 * @file   test-mp2p_yaml_variables.cpp
 * @brief  Unit tests for ${VAR|default} expansion in YAML files
 * @author Jose Luis Blanco Claraco
 * @date   Oct 25, 2026
 */

#include <mp2p_icp/yaml_variables.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>

#include <cstdlib>
#include <fstream>
#include <iostream>

namespace
{
void set_env(const char* name, const char* value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    ::setenv(name, value, 1);
#endif
}

void unset_env(const char* name)
{
#ifdef _WIN32
    _putenv_s(name, "");
#else
    ::unsetenv(name);
#endif
}

bool throws(const std::string& text)
{
    try
    {
        [[maybe_unused]] const auto s = mp2p_icp::expand_yaml_variables(text);
    }
    catch (const std::exception&)
    {
        return true;
    }
    return false;
}

void unit_test()
{
    using mp2p_icp::expand_yaml_variables;

    unset_env("MP2P_TEST_VAR");

    // Defaults:
    ASSERT_EQUAL_(expand_yaml_variables("a: ${MP2P_TEST_VAR|0.5}"), std::string("a: 0.5"));
    ASSERT_EQUAL_(expand_yaml_variables("a: '${MP2P_TEST_VAR|}'"), std::string("a: ''"));
    ASSERT_EQUAL_(
        expand_yaml_variables("a: \"robot_x - ${MP2P_TEST_VAR|12.0}\""),
        std::string("a: \"robot_x - 12.0\""));

    // Undefined without default:
    ASSERT_(throws("a: ${MP2P_TEST_VAR}"));
    ASSERT_(throws("a: ${MP2P_TEST_VAR"));

    // Comments are not expanded:
    ASSERT_EQUAL_(
        expand_yaml_variables("a: 1 # ${MP2P_TEST_VAR}\n# ${MP2P_TEST_VAR}"),
        std::string("a: 1 # ${MP2P_TEST_VAR}\n# ${MP2P_TEST_VAR}"));

    // A `#` within quotes or not after a blank does not start a comment:
    ASSERT_EQUAL_(
        expand_yaml_variables("a: \"map#${MP2P_TEST_VAR|_x}\"\nb: '#${MP2P_TEST_VAR|_y}'"),
        std::string("a: \"map#_x\"\nb: '#_y'"));
    ASSERT_EQUAL_(expand_yaml_variables("a: x#${MP2P_TEST_VAR|1}"), std::string("a: x#1"));
    ASSERT_EQUAL_(
        expand_yaml_variables("a: \"x # y\" # ${MP2P_TEST_VAR}"),
        std::string("a: \"x # y\" # ${MP2P_TEST_VAR}"));

    // An unclosed token in a comment does not reach the next lines:
    ASSERT_EQUAL_(
        expand_yaml_variables("# see ${MP2P_TEST_VAR\na: ${MP2P_TEST_VAR|2} # }"),
        std::string("# see ${MP2P_TEST_VAR\na: 2 # }"));

    // Custom variables:
    ASSERT_EQUAL_(
        expand_yaml_variables("a: ${MP2P_TEST_VAR|1}", {{"MP2P_TEST_VAR", "2"}}),
        std::string("a: 2"));

    // Environment variables have the highest priority:
    set_env("MP2P_TEST_VAR", "3");
    ASSERT_EQUAL_(
        expand_yaml_variables("a: ${MP2P_TEST_VAR|1}", {{"MP2P_TEST_VAR", "2"}}),
        std::string("a: 3"));
    ASSERT_EQUAL_(expand_yaml_variables("a: ${MP2P_TEST_VAR}"), std::string("a: 3"));

    // CURRENT_YAML_FILE_PATH from `variables` wins over the environment:
    set_env("CURRENT_YAML_FILE_PATH", "/wrong");
    ASSERT_EQUAL_(
        expand_yaml_variables("a: ${CURRENT_YAML_FILE_PATH}", {{"CURRENT_YAML_FILE_PATH", "/ok"}}),
        std::string("a: /ok"));

    // Values are not expanded again:
    set_env("MP2P_TEST_VAR", "${OTHER}");
    ASSERT_EQUAL_(expand_yaml_variables("a: '${MP2P_TEST_VAR}'"), std::string("a: '${OTHER}'"));
    unset_env("MP2P_TEST_VAR");

    // Whole file:
    const auto fileName = mrpt::system::getTempFileName() + ".yaml";
    {
        std::ofstream f(fileName);
        f << "# comment with ${UNDEFINED_VAR}\n"
             "resolution: ${MP2P_TEST_VAR|0.20}\n"
             "dir: '${CURRENT_YAML_FILE_PATH}'\n";
    }
    const auto yaml = mp2p_icp::load_yaml_file(fileName, {{"CURRENT_YAML_FILE_PATH", "/wrong"}});
    ASSERT_NEAR_(yaml["resolution"].as<double>(), 0.20, 1e-9);
    ASSERT_EQUAL_(
        yaml["dir"].as<std::string>(),
        mrpt::system::extractFileDirectory(mrpt::system::toAbsolutePath(fileName)));
    unset_env("CURRENT_YAML_FILE_PATH");
    mrpt::system::deleteFile(fileName);
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
