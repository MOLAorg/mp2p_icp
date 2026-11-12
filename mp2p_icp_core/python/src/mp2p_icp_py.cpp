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
 * @file   mp2p_icp_py.cpp
 * @brief  Python extension module `mp2p_icp._core`
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <pybind11/pybind11.h>

#include "py_common.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace mp2p_icp_py
{
void init_map(py::module_& m);
}  // namespace mp2p_icp_py

PYBIND11_MODULE(_core, m)
{
    m.doc() = "Python bindings of the mp2p_icp C++ libraries (use the `mp2p_icp` package)";
    m.attr("__version__") = MP2P_ICP_VERSION_STRING;

    // MRPT types used in signatures must be registered first, by their own
    // Python modules: this module shares their type registry, and the MRPT
    // libraries they wrap.
    for (const char* mod :
         {"mrpt.rtti", "mrpt.serialization", "mrpt.containers", "mrpt.math", "mrpt.img",
          "mrpt.poses", "mrpt.topography", "mrpt.viz", "mrpt.obs", "mrpt.maps"})
    {
        py::module_::import(mod);
    }

    m.def(
        "load_yaml",
        [](const py::object& config, const py::object& variables)
        { return mp2p_icp_py::yaml_to_python(mp2p_icp_py::load_config(config, variables)); },
        "config"_a, "variables"_a = py::none(),
        "Loads a YAML configuration (file path, dict or mrpt.containers.YAML node) into Python "
        "dicts and lists, expanding `${VAR}` and `${VAR|default}` tokens from `variables` (dict) "
        "and the environment, as all mp2p_icp configuration loaders do.");

    mp2p_icp_py::init_map(m);
}
