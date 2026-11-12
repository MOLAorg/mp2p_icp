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
 * @file   py_filters.cpp
 * @brief  Python bindings: filter pipelines, generators and simplemap_to_metricmap()
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <pybind11/stl.h>

#include "py_common.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace mp2p_icp_py
{
namespace
{
/** A filter pipeline and the source of its dynamic parameters, which must
 *  outlive the filters attached to it (hence they are kept together). */
struct PyFilterPipeline
{
    PyFilterPipeline()                                   = default;
    PyFilterPipeline(const PyFilterPipeline&)            = delete;
    PyFilterPipeline& operator=(const PyFilterPipeline&) = delete;

    // Declared first, so it is destroyed after the filters attached to it:
    mp2p_icp::ParameterSource        paramSource;
    mp2p_icp_filters::FilterPipeline filters;
};

/** Like PyFilterPipeline, for generators */
struct PyGenerators
{
    PyGenerators()                               = default;
    PyGenerators(const PyGenerators&)            = delete;
    PyGenerators& operator=(const PyGenerators&) = delete;

    mp2p_icp::ParameterSource      paramSource;
    mp2p_icp_filters::GeneratorSet generators;
};

/** Returns the sequence under `key` if the config is a map, or the config
 *  itself if it is a sequence */
mrpt::containers::yaml get_sequence(const mrpt::containers::yaml& cfg, const std::string& key)
{
    if (cfg.isMap() && cfg.has(key))
    {
        return cfg[key];
    }
    if (cfg.isSequence() || cfg.isNullNode())
    {
        return cfg;
    }
    throw py::value_error(
        "Expected a list of entries, or a configuration with a '" + key + "' list");
}

std::shared_ptr<PyFilterPipeline> make_filters(
    const py::object& config, const py::object& variables, const std::string& verbosity)
{
    const auto cfg = get_sequence(load_config(config, variables), "filters");
    require_registered_classes(cfg);
    auto ret     = std::make_shared<PyFilterPipeline>();
    ret->filters = mp2p_icp_filters::filter_pipeline_from_yaml(cfg, to_verbosity(verbosity));
    mp2p_icp::AttachToParameterSource(ret->filters, ret->paramSource);
    return ret;
}

std::shared_ptr<PyGenerators> make_generators(
    const py::object& config, const py::object& variables, const std::string& verbosity)
{
    const auto cfg = get_sequence(load_config(config, variables), "generators");
    require_registered_classes(cfg);
    auto ret        = std::make_shared<PyGenerators>();
    ret->generators = mp2p_icp_filters::generators_from_yaml(cfg, to_verbosity(verbosity));
    mp2p_icp::AttachToParameterSource(ret->generators, ret->paramSource);
    return ret;
}

void apply_filters(PyFilterPipeline& p, mp2p_icp::metric_map_t& map)
{
    py::gil_scoped_release nogil;
    p.paramSource.realize();
    mp2p_icp_filters::apply_filter_pipeline(p.filters, map);
}

std::shared_ptr<mp2p_icp::metric_map_t> apply_generators(
    PyGenerators& g, const py::object& obs, const py::object& robotPose,
    const std::shared_ptr<mp2p_icp::metric_map_t>& output)
{
    auto out = output ? output : std::make_shared<mp2p_icp::metric_map_t>();

    std::optional<mrpt::poses::CPose3D> pose;
    if (const auto p = to_optional_tpose3d(robotPose); p)
    {
        pose = mrpt::poses::CPose3D(*p);
    }

    if (py::isinstance<mrpt::obs::CSensoryFrame>(obs))
    {
        const auto&            sf = obs.cast<const mrpt::obs::CSensoryFrame&>();
        py::gil_scoped_release nogil;
        g.paramSource.realize();
        mp2p_icp_filters::apply_generators(g.generators, sf, *out, pose);
    }
    else if (py::isinstance<mrpt::obs::CObservation>(obs))
    {
        const auto&            o = obs.cast<const mrpt::obs::CObservation&>();
        py::gil_scoped_release nogil;
        g.paramSource.realize();
        mp2p_icp_filters::apply_generators(g.generators, o, *out, pose);
    }
    else
    {
        throw py::type_error("Expected an mrpt.obs.CObservation or mrpt.obs.CSensoryFrame");
    }
    return out;
}

std::vector<std::pair<std::string, std::string>> filter_names(const PyFilterPipeline& p)
{
    std::vector<std::pair<std::string, std::string>> ret;
    for (const auto& f : p.filters)
    {
        ret.emplace_back(f->GetRuntimeClass()->className, f->name);
    }
    return ret;
}

}  // namespace

void init_filters(py::module_& m)
{
    py::class_<PyFilterPipeline, std::shared_ptr<PyFilterPipeline>>(
        m, "FilterPipeline",
        "A sequence of mp2p_icp_filters filters, applied in order and in place to a MetricMap. "
        "Create it with filter_pipeline_from_yaml().")
        .def_static(
            "from_yaml", &make_filters, "config"_a, "variables"_a = py::none(),
            "verbosity"_a = "INFO",
            "Creates a filter pipeline from a YAML file path, a dict, or an mrpt.containers.YAML "
            "node: either a list of filters (each with `class_name` and `params`), or a map with "
            "a `filters` list. `${VAR}` tokens are expanded from `variables` and the "
            "environment.")
        .def(
            "apply", &apply_filters, "map"_a,
            "Applies all filters, in order, to the map (in place).")
        .def(
            "update_variables",
            [](PyFilterPipeline& self, const std::map<std::string, double>& vars)
            {
                for (const auto& [name, value] : vars)
                {
                    self.paramSource.updateVariable(name, value);
                }
            },
            "variables"_a,
            "Sets the values of variables used in parameter formulas of the filters, e.g. "
            "`max_range: '2*RANGE'`.")
        .def_property_readonly(
            "variables",
            [](const PyFilterPipeline& self) { return self.paramSource.getVariableValues(); },
            "Current values of the formula variables (dict)")
        .def_property_readonly(
            "filters", &filter_names, "List of (class_name, name) tuples, one per filter")
        .def("__len__", [](const PyFilterPipeline& self) { return self.filters.size(); })
        .def(
            "__repr__",
            [](const PyFilterPipeline& self)
            {
                std::string s;
                for (const auto& [cls, name] : filter_names(self))
                {
                    s += (s.empty() ? "" : ", ") + cls;
                }
                return "FilterPipeline([" + s + "])";
            });

    m.def(
        "filter_pipeline_from_yaml", &make_filters, "config"_a, "variables"_a = py::none(),
        "verbosity"_a = "INFO", "Creates a FilterPipeline, see FilterPipeline.from_yaml().");

    m.def(
        "apply_filter_pipeline", &apply_filters, "pipeline"_a, "map"_a,
        "Applies a FilterPipeline to a MetricMap, in place.");

    py::class_<PyGenerators, std::shared_ptr<PyGenerators>>(
        m, "Generators",
        "A set of mp2p_icp_filters generators, which convert MRPT observations into MetricMap "
        "layers. Create it with generators_from_yaml().")
        .def_static(
            "from_yaml", &make_generators, "config"_a, "variables"_a = py::none(),
            "verbosity"_a = "INFO",
            "Creates generators from a YAML file path, a dict, or an mrpt.containers.YAML node: "
            "either a list of generators (each with `class_name` and `params`), or a map with a "
            "`generators` list.")
        .def(
            "apply", &apply_generators, "observation"_a, "robot_pose"_a = py::none(),
            "output"_a = py::none(),
            "Converts an mrpt.obs.CObservation or CSensoryFrame into map layers, with an "
            "optional robot pose. The layers are added to `output` if given, or to a new "
            "MetricMap. Returns the map.")
        .def(
            "update_variables",
            [](PyGenerators& self, const std::map<std::string, double>& vars)
            {
                for (const auto& [name, value] : vars)
                {
                    self.paramSource.updateVariable(name, value);
                }
            },
            "variables"_a, "Sets the values of variables used in parameter formulas.")
        .def("__len__", [](const PyGenerators& self) { return self.generators.size(); });

    m.def(
        "generators_from_yaml", &make_generators, "config"_a, "variables"_a = py::none(),
        "verbosity"_a = "INFO", "Creates Generators, see Generators.from_yaml().");

    m.def(
        "apply_generators", &apply_generators, "generators"_a, "observation"_a,
        "robot_pose"_a = py::none(), "output"_a = py::none(),
        "Applies Generators to an observation, see Generators.apply().");

    m.def(
        "simplemap_to_metricmap",
        [](const py::object& simplemap, const py::object& pipeline, const py::object& variables,
           const std::map<std::string, double>& custom_variables, const std::string& verbosity,
           bool show_progress_bar, std::optional<size_t> start_index,
           std::optional<size_t> end_index, std::optional<size_t> decimate_every_nth_frame,
           std::optional<size_t> decimate_maximum_frame_count, bool throw_on_missing_external_files,
           const std::optional<mp2p_icp::metric_map_t::Georeferencing>& georeferencing,
           const py::object&                                            lazy_load_base_dir)
        {
            // Input simplemap, from a file or an mrpt.maps.CSimpleMap:
            std::shared_ptr<mrpt::maps::CSimpleMap> smFromFile;
            const mrpt::maps::CSimpleMap*           sm = nullptr;
            std::optional<std::string>              lazyDir;
            if (!lazy_load_base_dir.is_none())
            {
                lazyDir = to_path(lazy_load_base_dir);
            }
            if (is_path_like(simplemap))
            {
                const auto file = to_path(simplemap);
                if (!mrpt::system::fileExists(file))
                {
                    raise_file_not_found("Simplemap file not found: '" + file + "'");
                }
                smFromFile = std::make_shared<mrpt::maps::CSimpleMap>();
                {
                    py::gil_scoped_release nogil;
                    if (!smFromFile->loadFromFile(file))
                    {
                        throw std::runtime_error("Error loading simplemap from: '" + file + "'");
                    }
                }
                sm = smFromFile.get();
                // As sm2mm: external observation files are by default in "<name>_Images"
                const auto defaultDir = mrpt::system::fileNameChangeExtension(file, "") + "_Images";
                if (!lazyDir && mrpt::system::directoryExists(defaultDir))
                {
                    lazyDir = defaultDir;
                }
            }
            else
            {
                sm = &simplemap.cast<const mrpt::maps::CSimpleMap&>();
            }
            if (lazyDir)
            {
                mrpt::io::setLazyLoadPathBase(*lazyDir);
            }

            const mrpt::containers::yaml cfg =
                pipeline.is_none() ? mrpt::containers::yaml() : load_config(pipeline, variables);
            require_registered_classes(cfg);

            mp2p_icp_filters::sm2mm_options_t opts;
            opts.verbosity                       = to_verbosity(verbosity);
            opts.showProgressBar                 = show_progress_bar;
            opts.throw_on_missing_external_files = throw_on_missing_external_files;
            opts.start_index                     = start_index;
            opts.end_index                       = end_index;
            opts.decimate_every_nth_frame        = decimate_every_nth_frame;
            opts.decimate_maximum_frame_count    = decimate_maximum_frame_count;
            opts.georeferencing                  = georeferencing;
            for (const auto& [name, value] : custom_variables)
            {
                opts.customVariables.emplace_back(name, value);
            }

            auto out = std::make_shared<mp2p_icp::metric_map_t>();
            {
                py::gil_scoped_release nogil;
                mp2p_icp_filters::simplemap_to_metricmap(*sm, *out, cfg, opts);
            }
            return out;
        },
        "simplemap"_a, "pipeline"_a = py::none(), py::kw_only(), "variables"_a = py::none(),
        "custom_variables"_a = std::map<std::string, double>(), "verbosity"_a = "INFO",
        "show_progress_bar"_a = false, "start_index"_a = py::none(), "end_index"_a = py::none(),
        "decimate_every_nth_frame"_a = py::none(), "decimate_maximum_frame_count"_a = py::none(),
        "throw_on_missing_external_files"_a = true, "georeferencing"_a = py::none(),
        "lazy_load_base_dir"_a = py::none(),
        "Builds a MetricMap from a simplemap (keyframes with poses and raw observations), as the "
        "`sm2mm` tool does. `simplemap` is an mrpt.maps.CSimpleMap or a file path; `pipeline` "
        "is an sm2mm YAML file path, dict or YAML node, with `generators`, `filters` and "
        "`final_filters` (default: one generator, no filters). `variables` expands `${VAR}` "
        "tokens in the pipeline; `custom_variables` defines extra numeric variables for "
        "parameter formulas. `lazy_load_base_dir` is the directory of externally stored "
        "observations (default: `<simplemap name>_Images`, if it exists).");
}

}  // namespace mp2p_icp_py
