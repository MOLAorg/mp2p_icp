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
 * @file   py_common.h
 * @brief  Helpers shared by the Python bindings of mp2p_icp
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>
#include <pybind11/pybind11.h>

#include <map>
#include <optional>
#include <string>

namespace mp2p_icp_py
{
namespace py = pybind11;

/** Raises a Python FileNotFoundError with the given message */
[[noreturn]] void raise_file_not_found(const std::string& msg);

/** Converts a YAML node into Python dicts, lists and scalars */
py::object yaml_to_python(const mrpt::containers::yaml& y);

/** Converts Python dicts, lists and scalars into a YAML node */
mrpt::containers::yaml python_to_yaml(const py::object& obj);

/** Converts a Python dict {name: value} into strings, for `${VAR}` expansion */
std::map<std::string, std::string> to_string_variables(const py::object& variables);

/** Loads a YAML configuration from: a file path (str or os.PathLike), a dict,
 *  or an mrpt.containers.YAML node. `${VAR}` and `${VAR|default}` tokens are
 *  expanded with `variables` and the environment.
 */
mrpt::containers::yaml load_config(const py::object& config, const py::object& variables);

/** Returns true if the object is a str or os.PathLike */
bool is_path_like(const py::object& obj);

/** Converts a str or os.PathLike into a string */
std::string to_path(const py::object& obj);

/** Accepts: None (identity), mrpt.math.TPose3D, mrpt.poses.CPose3D, a 4x4
 *  homogeneous matrix (array-like), or a sequence of 6 values
 *  [x, y, z, yaw, pitch, roll] (meters, radians). */
mrpt::math::TPose3D to_tpose3d(const py::object& obj);

/** Like to_tpose3d(), for optional arguments */
std::optional<mrpt::math::TPose3D> to_optional_tpose3d(const py::object& obj);

/** Parses a verbosity level name: "DEBUG", "INFO", "WARN" or "ERROR" */
mrpt::system::VerbosityLevel to_verbosity(const std::string& name);

/** Returns a 4x4 NumPy array with the homogeneous matrix of the pose */
py::object pose_to_matrix(const mrpt::poses::CPose3D& p);

}  // namespace mp2p_icp_py
