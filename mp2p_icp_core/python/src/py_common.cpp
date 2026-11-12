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
 * @file   py_common.cpp
 * @brief  Helpers shared by the Python bindings of mp2p_icp
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include "py_common.h"

#include <mp2p_icp/yaml_variables.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <pybind11/numpy.h>

#include <cctype>
#include <sstream>
#include <typeinfo>

namespace mp2p_icp_py
{
void raise_file_not_found(const std::string& msg)
{
    PyErr_SetString(PyExc_FileNotFoundError, msg.c_str());
    throw py::error_already_set();
}

namespace
{
bool is_backtrace_frame(const std::string& line)
{
    // e.g. "[3 ]     0x7c6c88d2e81e " or "[12] file.cpp:10 func()"
    const auto close = line.find(']');
    if (line.empty() || line[0] != '[' || close == std::string::npos || close < 2)
    {
        return false;
    }
    for (size_t i = 1; i < close; i++)
    {
        if (!std::isdigit(static_cast<unsigned char>(line[i])) && line[i] != ' ')
        {
            return false;
        }
    }
    return true;
}

std::string trim(const std::string& s)
{
    const auto b = s.find_first_not_of(" \t\n");
    if (b == std::string::npos)
    {
        return {};
    }
    return s.substr(b, s.find_last_not_of(" \t\n") - b + 1);
}

bool is_mrpt_exception_text(const std::string& s)
{
    return s.find("==== MRPT exception ====") != std::string::npos;
}
}  // namespace

std::string clean_error_message(const std::string& what)
{
    if (!is_mrpt_exception_text(what))
    {
        return what;
    }
    std::string        out;
    std::istringstream ss(what);
    std::string        line;
    bool               inBacktrace = false;
    while (std::getline(ss, line))
    {
        if (inBacktrace && is_backtrace_frame(line))
        {
            continue;
        }
        inBacktrace = false;
        if (line == "==== MRPT exception ====" || line == "==== non-MRPT exception ====" ||
            line.rfind("Location: ", 0) == 0)
        {
            continue;
        }
        if (line == "Call stack backtrace:")
        {
            inBacktrace = true;
            continue;
        }
        if (line.rfind("Message: ", 0) == 0)
        {
            line = line.substr(std::string("Message: ").size());
        }
        line = trim(line);
        if (!line.empty())
        {
            out += (out.empty() ? "" : "\n") + line;
        }
    }
    return out.empty() ? what : out;
}

void register_exception_translator()
{
    py::register_local_exception_translator(
        [](std::exception_ptr p)
        {
            try
            {
                if (p)
                {
                    std::rethrow_exception(p);
                }
            }
            catch (const std::exception& e)
            {
                const std::string full = e.what();
                if (!is_mrpt_exception_text(full))
                {
                    throw;  // to the default translators
                }
                const std::string msg = clean_error_message(full);
                // (no exception may escape from here: use the C API)
                PyObject* txt = PyUnicode_DecodeUTF8(
                    msg.data(), static_cast<Py_ssize_t>(msg.size()), "replace");
                PyObject* det = PyUnicode_DecodeUTF8(
                    full.data(), static_cast<Py_ssize_t>(full.size()), "replace");
                PyObject* exc = txt ? PyObject_CallOneArg(PyExc_RuntimeError, txt) : nullptr;
                if (exc && det && PyObject_SetAttrString(exc, "cpp_traceback", det) == 0)
                {
                    PyErr_SetObject(PyExc_RuntimeError, exc);
                }
                else
                {
                    PyErr_Clear();
                    PyErr_SetString(PyExc_RuntimeError, msg.c_str());
                }
                Py_XDECREF(exc);
                Py_XDECREF(txt);
                Py_XDECREF(det);
            }
        });
}

namespace
{
/** Short description of a YAML node, for error messages */
std::string describe(const mrpt::containers::yaml& y)
{
    if (y.isNullNode())
    {
        return "null";
    }
    if (y.isScalar())
    {
        return "'" + y.as<std::string>() + "'";
    }
    return y.isMap() ? "a map" : "a list";
}

[[noreturn]] void invalid_config(const std::string& what, const std::string& msg)
{
    throw py::value_error("Invalid " + what + " configuration: " + msg);
}

/** Checks a list of pipeline modules (maps with `classKey` and `params`) */
void validate_module_list(
    const mrpt::containers::yaml& list, const std::string& what, const std::string& classKey,
    bool allowNull)
{
    if (list.isNullNode() && allowNull)
    {
        return;
    }
    if (!list.isSequence())
    {
        invalid_config(what, "expected a list of entries, got " + describe(list));
    }
    size_t i = 0;
    for (const auto& node : list.asSequence())
    {
        const mrpt::containers::yaml e(node);
        const std::string            entry = "entry #" + std::to_string(++i);
        if (!e.isMap())
        {
            invalid_config(
                what,
                entry + " must be a map with '" + classKey + "' and 'params', got " + describe(e));
        }
        if (e.has("enabled") && e["enabled"].isScalar() && !e["enabled"].as<bool>())
        {
            continue;  // (ignored, as in C++)
        }
        if (!e.has(classKey) || !e[classKey].isScalar())
        {
            invalid_config(what, entry + " has no '" + classKey + "' (its C++ class name)");
        }
        if (!e.has("params"))
        {
            invalid_config(what, entry + " has no 'params' (use null or {} for the defaults)");
        }
        if (!e["params"].isMap() && !e["params"].isNullNode())
        {
            invalid_config(
                what, "'params' of " + entry + " must be a map, got " + describe(e["params"]));
        }
    }
}
}  // namespace

void validate_pipeline_list(const mrpt::containers::yaml& list, const std::string& what)
{
    validate_module_list(list, what, "class_name", true);
}

void validate_icp_pipeline(const mrpt::containers::yaml& cfg)
{
    const std::string what = "ICP pipeline";
    if (!cfg.isMap())
    {
        invalid_config(
            what,
            "expected a map with 'class_name', 'params', 'solvers', 'matchers' and "
            "'quality', got " +
                describe(cfg));
    }
    if (!cfg.has("class_name") || !cfg["class_name"].isScalar())
    {
        invalid_config(what, "missing 'class_name' (e.g. 'mp2p_icp::ICP')");
    }
    if (!cfg.has("derived") && (!cfg.has("params") || !cfg["params"].isMap()))
    {
        invalid_config(what, "missing the 'params' map (see mp2p_icp::Parameters)");
    }
    for (const char* key : {"solvers", "matchers", "quality"})
    {
        if (cfg.has(key))
        {
            validate_module_list(cfg[key], what + " '" + key + "'", "class", false);
        }
        else if (std::string(key) == "quality")
        {
            invalid_config(what, "missing the 'quality' list (quality evaluators)");
        }
    }
}

void validate_sm2mm_pipeline(const mrpt::containers::yaml& cfg)
{
    if (cfg.isNullNode())
    {
        return;
    }
    if (!cfg.isMap())
    {
        invalid_config(
            "sm2mm pipeline",
            "expected a map with 'generators', 'filters' and 'final_filters', got " +
                describe(cfg));
    }
    for (const char* key : {"generators", "filters", "final_filters"})
    {
        if (cfg.has(key))
        {
            validate_pipeline_list(cfg[key], std::string("sm2mm pipeline '") + key + "'");
        }
    }
}

namespace
{
// Python module whose import registers the `mola::` classes:
constexpr const char* CLASS_PLUGINS_MODULE = "mola_metric_maps";

// Only tried once per process (protected by the GIL):
bool classPluginsImportTried = false;

/** True for "mola::Name" or "mola::ns::Name" */
bool is_mola_class_name(const std::string& s)
{
    const std::string prefix = "mola::";
    if (s.rfind(prefix, 0) != 0 || s.size() == prefix.size() || s.back() == ':')
    {
        return false;
    }
    for (const char c : s.substr(prefix.size()))
    {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != ':')
        {
            return false;
        }
    }
    return true;
}

/** Returns the first `mola::` class name in the YAML tree that is not
 *  registered, or an empty string */
std::string find_unregistered_class(const mrpt::containers::yaml& y)
{
    if (y.isScalar())
    {
        if (y.scalarType() == typeid(std::string))
        {
            const auto s = y.as<std::string>();
            if (is_mola_class_name(s) && !mrpt::rtti::findRegisteredClass(s))
            {
                return s;
            }
        }
        return {};
    }
    if (y.isMap())
    {
        for (const auto& kv : y.asMap())
        {
            if (auto s = find_unregistered_class(mrpt::containers::yaml(kv.second)); !s.empty())
            {
                return s;
            }
        }
    }
    else if (y.isSequence())
    {
        for (const auto& e : y.asSequence())
        {
            if (auto s = find_unregistered_class(mrpt::containers::yaml(e)); !s.empty())
            {
                return s;
            }
        }
    }
    return {};
}
}  // namespace

bool import_class_plugins()
{
    if (classPluginsImportTried)
    {
        return false;
    }
    classPluginsImportTried = true;
    const auto util         = py::module_::import("importlib.util");
    if (util.attr("find_spec")(CLASS_PLUGINS_MODULE).is_none())
    {
        return false;
    }
    py::module_::import(CLASS_PLUGINS_MODULE);
    return true;
}

std::string class_plugins_hint()
{
    return "\nHint: maps and pipelines with classes from other libraries need them to be "
           "registered first. For MOLA classes (mola::...), install the mola_metric_maps Python "
           "module (pip install mola-metric-maps, imported automatically when needed), or, with "
           "the ROS package mola_metric_maps, call "
           "mp2p_icp.load_plugins('libmola_metric_maps.so').";
}

bool is_unregistered_class_error(const std::string& msg)
{
    return msg.find("is not registered") != std::string::npos;
}

void require_registered_classes(const mrpt::containers::yaml& cfg)
{
    auto missing = find_unregistered_class(cfg);
    if (missing.empty())
    {
        return;
    }
    if (import_class_plugins())
    {
        missing = find_unregistered_class(cfg);
    }
    if (!missing.empty())
    {
        throw std::runtime_error(
            "Class '" + missing + "' is not registered." + class_plugins_hint());
    }
}

py::object yaml_to_python(const mrpt::containers::yaml& y)
{
    if (y.isNullNode())
    {
        return py::none();
    }
    if (y.isScalar())
    {
        const auto& t = y.scalarType();
        if (t == typeid(bool))
        {
            return py::bool_(y.as<bool>());
        }
        if (t == typeid(int64_t))
        {
            return py::int_(y.as<int64_t>());
        }
        if (t == typeid(uint64_t))
        {
            return py::int_(y.as<uint64_t>());
        }
        if (t == typeid(double))
        {
            return py::float_(y.as<double>());
        }
        return py::str(y.as<std::string>());
    }
    if (y.isMap())
    {
        py::dict d;
        for (const auto& kv : y.asMap())
        {
            const std::string k(kv.first.internalAsStr());
            d[py::str(k)] = yaml_to_python(mrpt::containers::yaml(kv.second));
        }
        return d;
    }
    if (y.isSequence())
    {
        py::list lst;
        for (const auto& e : y.asSequence())
        {
            lst.append(yaml_to_python(mrpt::containers::yaml(e)));
        }
        return lst;
    }
    return py::none();
}

namespace
{
// JSON is a subset of YAML: converting through text gives Python input
// exactly the same treatment as YAML files.
std::string python_to_json(const py::object& obj)
{
    const auto json = py::module_::import("json");
    // NumPy scalars and arrays are not JSON serializable: use their Python
    // equivalents.
    const auto fallback = py::cpp_function(
        [](const py::object& o) -> py::object
        {
            if (py::hasattr(o, "tolist"))
            {
                return o.attr("tolist")();
            }
            if (py::hasattr(o, "item"))
            {
                return o.attr("item")();
            }
            if (py::hasattr(o, "__fspath__"))
            {
                return o.attr("__fspath__")();
            }
            throw py::type_error(
                "Object of type '" + py::str(o.get_type().attr("__name__")).cast<std::string>() +
                "' cannot be converted to YAML");
        });
    return json.attr("dumps")(obj, py::arg("default") = fallback).cast<std::string>();
}
}  // namespace

mrpt::containers::yaml python_to_yaml(const py::object& obj)
{
    if (py::isinstance<mrpt::containers::yaml>(obj))
    {
        return obj.cast<mrpt::containers::yaml>();
    }
    if (obj.is_none())
    {
        return {};
    }
    return mrpt::containers::yaml::FromText(python_to_json(obj));
}

std::map<std::string, std::string> to_string_variables(const py::object& variables)
{
    std::map<std::string, std::string> vars;
    if (variables.is_none())
    {
        return vars;
    }
    for (const auto& kv : variables.cast<py::dict>())
    {
        vars[py::str(kv.first).cast<std::string>()] = py::str(kv.second).cast<std::string>();
    }
    return vars;
}

bool is_path_like(const py::object& obj)
{
    return py::isinstance<py::str>(obj) || py::hasattr(obj, "__fspath__");
}

std::string to_path(const py::object& obj)
{
    if (py::isinstance<py::str>(obj))
    {
        return obj.cast<std::string>();
    }
    return py::module_::import("os").attr("fspath")(obj).cast<std::string>();
}

mrpt::containers::yaml load_config(const py::object& config, const py::object& variables)
{
    const auto vars = to_string_variables(variables);

    if (is_path_like(config))
    {
        const auto path = to_path(config);
        if (!mrpt::system::fileExists(path))
        {
            raise_file_not_found("Configuration file not found: '" + path + "'");
        }
        return mp2p_icp::load_yaml_file(path, vars);
    }

    std::string text;
    if (py::isinstance<mrpt::containers::yaml>(config))
    {
        std::stringstream ss;
        config.cast<const mrpt::containers::yaml&>().printAsYAML(ss);
        text = ss.str();
    }
    else if (py::isinstance<py::dict>(config) || py::isinstance<py::list>(config))
    {
        text = python_to_json(config);
    }
    else
    {
        throw py::type_error(
            "Expected a configuration as a file path, a dict, or an mrpt.containers.YAML node");
    }
    return mrpt::containers::yaml::FromText(mp2p_icp::expand_yaml_variables(text, vars));
}

mrpt::math::TPose3D to_tpose3d(const py::object& obj)
{
    if (obj.is_none())
    {
        return {};
    }
    if (py::isinstance<mrpt::math::TPose3D>(obj))
    {
        return obj.cast<mrpt::math::TPose3D>();
    }
    if (py::isinstance<mrpt::poses::CPose3D>(obj))
    {
        return obj.cast<const mrpt::poses::CPose3D&>().asTPose();
    }
    if (py::isinstance<mrpt::poses::CPose2D>(obj))
    {
        return mrpt::poses::CPose3D(obj.cast<const mrpt::poses::CPose2D&>()).asTPose();
    }
    if (py::isinstance<mrpt::math::TPose2D>(obj))
    {
        const auto p = obj.cast<mrpt::math::TPose2D>();
        return {p.x, p.y, 0.0, p.phi, 0.0, 0.0};
    }

    const auto arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
    if (!arr)
    {
        throw py::type_error(
            "Cannot convert into a pose: expected None, TPose3D, CPose3D, a 4x4 matrix, or "
            "[x, y, z, yaw, pitch, roll]");
    }
    if (arr.ndim() == 2 && arr.shape(0) == 4 && arr.shape(1) == 4)
    {
        mrpt::math::CMatrixDouble44 M;
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                M(r, c) = arr.at(r, c);
            }
        }
        return mrpt::poses::CPose3D(M).asTPose();
    }
    if (arr.ndim() == 1 && arr.shape(0) == 6)
    {
        return {arr.at(0), arr.at(1), arr.at(2), arr.at(3), arr.at(4), arr.at(5)};
    }
    throw py::value_error(
        "Cannot convert into a pose: expected a 4x4 homogeneous matrix or a vector [x, y, z, "
        "yaw, pitch, roll]");
}

std::optional<mrpt::math::TPose3D> to_optional_tpose3d(const py::object& obj)
{
    if (obj.is_none())
    {
        return std::nullopt;
    }
    return to_tpose3d(obj);
}

mrpt::system::VerbosityLevel to_verbosity(const std::string& name)
{
    std::string s = mrpt::system::upperCase(name);
    if (s.rfind("LVL_", 0) == 0)
    {
        s = s.substr(4);
    }
    if (s == "DEBUG")
    {
        return mrpt::system::LVL_DEBUG;
    }
    if (s == "INFO")
    {
        return mrpt::system::LVL_INFO;
    }
    if (s == "WARN" || s == "WARNING")
    {
        return mrpt::system::LVL_WARN;
    }
    if (s == "ERROR")
    {
        return mrpt::system::LVL_ERROR;
    }
    throw py::value_error(
        "Invalid verbosity '" + name + "': use 'DEBUG', 'INFO', 'WARN' or 'ERROR'");
}

py::object pose_to_matrix(const mrpt::poses::CPose3D& p)
{
    const auto          M = p.getHomogeneousMatrix();
    py::array_t<double> arr(std::vector<py::ssize_t>{4, 4});
    auto                a = arr.mutable_unchecked<2>();
    for (py::ssize_t r = 0; r < 4; r++)
    {
        for (py::ssize_t c = 0; c < 4; c++)
        {
            a(r, c) = M(r, c);
        }
    }
    return arr;
}

}  // namespace mp2p_icp_py
