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
// Python module built against the C++ SDK of the mp2p-icp wheel, sharing the
// Python types bound by mp2p_icp._core: `import mp2p_icp` before it.

#include <pybind11/pybind11.h>
#include <sdk_consumer_lib.h>

namespace py = pybind11;

PYBIND11_MODULE(sdk_consumer_py, m)
{
    // The mp2p_icp types must be registered first, by their own module:
    py::module_::import("mp2p_icp");

    m.def("make_map", &sdk_consumer::make_map, py::arg("n"), "Returns a mp2p_icp.MetricMap");
    m.def(
        "count_points", &sdk_consumer::count_points, py::arg("map"), "Takes a mp2p_icp.MetricMap");
}
