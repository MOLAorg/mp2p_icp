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
 * @file   py_icp.cpp
 * @brief  Python bindings: ICP pipelines, parameters, results and logs
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/LogRecord.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <exception>
#include <sstream>

#include "py_common.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace mp2p_icp_py
{
namespace
{
/** Python-side ICP object: the pipeline, its default parameters, and the
 *  source of its dynamic parameters, which must outlive the pipeline modules
 *  attached to it (hence they are kept together). */
struct PyICP
{
    PyICP()                        = default;
    PyICP(const PyICP&)            = delete;
    PyICP& operator=(const PyICP&) = delete;

    // Declared first, so it is destroyed after the modules attached to it:
    mp2p_icp::ParameterSource             paramSource;
    mp2p_icp::ICP::Ptr                    icp;
    std::shared_ptr<mp2p_icp::Parameters> parameters;
    py::object                            hook = py::none();
};

/** Passed to the Python iteration hook */
struct IterationInfo
{
    uint32_t             iteration = 0;
    mrpt::poses::CPose3D pose;
    size_t               num_pairings       = 0;
    uint64_t             potential_pairings = 0;
};

py::object parameters_as_dict(const mp2p_icp::Parameters& p)
{
    mrpt::containers::yaml y = mrpt::containers::yaml::Map();
    p.save_to(y);
    return yaml_to_python(y);
}

std::shared_ptr<PyICP> make_icp(
    const py::object& config, const py::object& variables, const std::string& verbosity)
{
    const auto cfg = load_config(config, variables);
    require_registered_classes(cfg);

    auto ret           = std::make_shared<PyICP>();
    auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg, to_verbosity(verbosity));
    ret->icp           = icp;
    ret->parameters    = std::make_shared<mp2p_icp::Parameters>(params);
    ret->icp->attachToParameterSource(ret->paramSource);
    return ret;
}

std::string class_names_of(const mp2p_icp::ICP& icp)
{
    std::string s;
    const auto  add = [&s](const auto& o)
    {
        if (!s.empty())
        {
            s += ", ";
        }
        s += o.GetRuntimeClass()->className;
    };
    for (const auto& o : icp.matchers())
    {
        add(*o);
    }
    for (const auto& o : icp.solvers())
    {
        add(*o);
    }
    for (const auto& o : icp.quality_evaluators())
    {
        add(*o.obj);
    }
    return s;
}

mp2p_icp::Results align(
    PyICP& self, const mp2p_icp::metric_map_t& local, const mp2p_icp::metric_map_t& global,
    const py::object& initialGuess, const mp2p_icp::Parameters* parameters,
    const std::optional<mrpt::poses::CPose3DPDFGaussianInf>& prior,
    const std::optional<mp2p_icp::GravityPrior>& gravityPrior, mp2p_icp::LogRecord* log)
{
    const auto  initGuess = to_tpose3d(initialGuess);
    const auto& params    = parameters ? *parameters : *self.parameters;

    // Exceptions raised by the Python hook are re-raised once align() returns:
    std::exception_ptr hookError;

    // The hook refers to this stack frame: always remove it on return.
    struct HookReset
    {
        mp2p_icp::ICP& icp;
        ~HookReset() { icp.setIterationHook({}); }
    } hookReset{*self.icp};

    if (!self.hook.is_none())
    {
        self.icp->setIterationHook(
            [&self, &hookError](const mp2p_icp::ICP::IterationHook_Input& in)
            {
                mp2p_icp::ICP::IterationHook_Output out;
                py::gil_scoped_acquire              gil;
                try
                {
                    IterationInfo info;
                    info.iteration = in.currentIteration;
                    if (in.currentSolution)
                    {
                        info.pose = in.currentSolution->optimalPose;
                    }
                    if (in.currentPairings)
                    {
                        info.num_pairings       = in.currentPairings->size();
                        info.potential_pairings = in.currentPairings->potential_pairings;
                    }
                    const py::object r = self.hook(info);
                    out.request_stop   = !r.is_none() && py::bool_(r);
                }
                catch (...)
                {
                    hookError        = std::current_exception();
                    out.request_stop = true;
                }
                return out;
            });
    }

    mp2p_icp::Results result;
    {
        py::gil_scoped_release nogil;
        if (log)
        {
            self.icp->align(local, global, initGuess, params, result, prior, *log, gravityPrior);
        }
        else
        {
            self.icp->align(
                local, global, initGuess, params, result, prior, std::nullopt, gravityPrior);
        }
    }

    if (hookError)
    {
        std::rethrow_exception(hookError);
    }
    return result;
}

py::dict pt2pt_to_numpy(const mp2p_icp::Pairings& p)
{
    const auto            n = static_cast<py::ssize_t>(p.paired_pt2pt.size());
    py::array_t<float>    global(std::vector<py::ssize_t>{n, 3});
    py::array_t<float>    local(std::vector<py::ssize_t>{n, 3});
    py::array_t<uint32_t> gIdx(std::vector<py::ssize_t>{n});
    py::array_t<uint32_t> lIdx(std::vector<py::ssize_t>{n});
    auto                  g  = global.mutable_unchecked<2>();
    auto                  l  = local.mutable_unchecked<2>();
    auto                  gi = gIdx.mutable_unchecked<1>();
    auto                  li = lIdx.mutable_unchecked<1>();
    for (py::ssize_t i = 0; i < n; i++)
    {
        const auto& pr = p.paired_pt2pt[static_cast<size_t>(i)];
        g(i, 0)        = pr.global.x;
        g(i, 1)        = pr.global.y;
        g(i, 2)        = pr.global.z;
        l(i, 0)        = pr.local.x;
        l(i, 1)        = pr.local.y;
        l(i, 2)        = pr.local.z;
        gi(i)          = pr.globalIdx;
        li(i)          = pr.localIdx;
    }
    py::dict d;
    d["global"]     = global;
    d["local"]      = local;
    d["global_idx"] = gIdx;
    d["local_idx"]  = lIdx;
    return d;
}

std::shared_ptr<mp2p_icp::LogRecord> load_log(const std::string& path)
{
    if (!mrpt::system::fileExists(path))
    {
        raise_file_not_found("ICP log file not found: '" + path + "'");
    }
    return load_with_class_plugins(
        [&]()
        {
            auto                             ret = std::make_shared<mp2p_icp::LogRecord>();
            py::gil_scoped_release           nogil;
            mrpt::io::CCompressedInputStream f(path);
            if (!f.is_open())
            {
                throw std::runtime_error("Cannot open file: '" + path + "'");
            }
            auto arch = mrpt::serialization::archiveFrom(f);
            arch >> *ret;
            return ret;
        });
}

}  // namespace

void init_icp(py::module_& m)
{
    using mp2p_icp::IterTermReason;
    py::enum_<IterTermReason>(m, "IterTermReason", "Reason why ICP iterations ended")
        .value("Undefined", IterTermReason::Undefined)
        .value("NoPairings", IterTermReason::NoPairings)
        .value("SolverError", IterTermReason::SolverError)
        .value("MaxIterations", IterTermReason::MaxIterations)
        .value("Stalled", IterTermReason::Stalled)
        .value("QualityCheckpointFailed", IterTermReason::QualityCheckpointFailed)
        .value("HookRequest", IterTermReason::HookRequest);

    // ---------------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------------
    using mp2p_icp::Parameters;
    py::class_<Parameters, mrpt::serialization::CSerializable, std::shared_ptr<Parameters>>(
        m, "Parameters", "ICP parameters: termination criteria, debug files, quality checkpoints")
        .def(py::init<>(), "Default parameters.")
        .def_static(
            "from_dict",
            [](const py::object& d)
            {
                auto p = std::make_shared<Parameters>();
                p->load_from(python_to_yaml(d));
                return p;
            },
            "d"_a,
            "Builds parameters from a dict, with the format of the `params` block of ICP YAML "
            "files.")
        .def("as_dict", &parameters_as_dict, "Returns the parameters as a dict.")
        .def_readwrite(
            "maxIterations", &Parameters::maxIterations, "Maximum number of ICP iterations")
        .def_readwrite(
            "minAbsStep_trans", &Parameters::minAbsStep_trans,
            "Stop when all translation corrections are below this [m]")
        .def_readwrite(
            "minAbsStep_rot", &Parameters::minAbsStep_rot,
            "Stop when all rotation corrections are below this [rad]")
        .def_readwrite(
            "freezePairingsAfterIteration", &Parameters::freezePairingsAfterIteration,
            "If >0, pairings are not recomputed after this iteration")
        .def_readwrite(
            "generateDebugFiles", &Parameters::generateDebugFiles,
            "Save a .icplog file for each align() call")
        .def_readwrite(
            "saveIterationDetails", &Parameters::saveIterationDetails,
            "Store the pairings and pose of each iteration in the LogRecord")
        .def_readwrite(
            "decimationIterationDetails", &Parameters::decimationIterationDetails,
            "With saveIterationDetails, keep 1 out of N iterations")
        .def_readwrite(
            "decimationDebugFiles", &Parameters::decimationDebugFiles,
            "With generateDebugFiles, save 1 out of N log files")
        .def_readwrite(
            "debugFileNameFormat", &Parameters::debugFileNameFormat,
            "File name pattern of the debug files")
        .def_readwrite(
            "debugPrintIterationProgress", &Parameters::debugPrintIterationProgress,
            "Print the progress of each iteration")
        .def_readwrite(
            "quality_checkpoints", &Parameters::quality_checkpoints,
            "dict {iteration: minimum quality}: abort if the quality is lower at that iteration")
        .def_readwrite(
            "force_final_pairings_for_quality", &Parameters::force_final_pairings_for_quality,
            "Run a last matching pass at the final thresholds before evaluating quality")
        .def("__copy__", [](const Parameters& p) { return std::make_shared<Parameters>(p); })
        .def(
            "__deepcopy__",
            [](const Parameters& p, const py::object&) { return std::make_shared<Parameters>(p); },
            "memo"_a)
        .def(
            "__repr__", [](const Parameters& p)
            { return "Parameters(" + py::repr(parameters_as_dict(p)).cast<std::string>() + ")"; });

    // ---------------------------------------------------------------------
    // Pairings, Results
    // ---------------------------------------------------------------------
    using mp2p_icp::Pairings;
    py::class_<Pairings>(m, "Pairings", "Pairings between geometric entities of two maps")
        .def(py::init<>(), "Empty pairings.")
        .def("size", &Pairings::size, "Overall number of pairings")
        .def("__len__", &Pairings::size)
        .def("empty", &Pairings::empty, "True if there are no pairings")
        .def("contents_summary", &Pairings::contents_summary, "Summary of the pairings")
        .def_readonly(
            "potential_pairings", &Pairings::potential_pairings,
            "Maximum number of pairings the matchers could have found")
        .def_property_readonly(
            "num_pt2pt", [](const Pairings& p) { return p.paired_pt2pt.size(); },
            "Number of point-to-point pairings")
        .def_property_readonly(
            "num_pt2pl", [](const Pairings& p) { return p.paired_pt2pl.size(); },
            "Number of point-to-plane pairings")
        .def_property_readonly(
            "num_pt2ln", [](const Pairings& p) { return p.paired_pt2ln.size(); },
            "Number of point-to-line pairings")
        .def_property_readonly(
            "num_pl2pl", [](const Pairings& p) { return p.paired_pl2pl.size(); },
            "Number of plane-to-plane pairings")
        .def_property_readonly(
            "num_ln2ln", [](const Pairings& p) { return p.paired_ln2ln.size(); },
            "Number of line-to-line pairings")
        .def_property_readonly(
            "num_cov2cov", [](const Pairings& p) { return p.paired_cov2cov.size(); },
            "Number of covariance-to-covariance pairings")
        .def(
            "pt2pt_to_numpy", &pt2pt_to_numpy,
            "Returns the point-to-point pairings as a dict of NumPy arrays: 'global' and 'local' "
            "(N,3) point coordinates, and 'global_idx' and 'local_idx' point indices.")
        .def(
            "get_visualization",
            [](const Pairings& p, const py::object& localWrtGlobal)
            { return p.get_visualization(mrpt::poses::CPose3D(to_tpose3d(localWrtGlobal))); },
            "local_wrt_global"_a = py::none(),
            "Returns an mrpt.viz.CSetOfObjects with the pairings, for the given pose of the "
            "local map")
        .def(
            "__repr__", [](const Pairings& p) { return "Pairings(" + p.contents_summary() + ")"; });

    using mp2p_icp::Results;
    py::class_<Results>(m, "Results", "Results of ICP.align()")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "optimal_tf", &Results::optimal_tf,
            "Pose of the local map with respect to the global one, with its covariance "
            "(mrpt.poses.CPose3DPDFGaussian)")
        .def_property_readonly(
            "optimal_pose_matrix",
            [](const Results& r) { return pose_to_matrix(r.optimal_tf.mean); },
            "The mean of optimal_tf as a 4x4 homogeneous matrix (NumPy array)")
        .def_readwrite("optimalScale", &Results::optimalScale, "Scale, if estimated")
        .def_readwrite("nIterations", &Results::nIterations, "Number of executed iterations")
        .def_readwrite("terminationReason", &Results::terminationReason, "Why iterations ended")
        .def_readwrite("quality", &Results::quality, "Registration quality, in [0,1]")
        .def_readwrite("finalPairings", &Results::finalPairings, "Pairings of the last iteration")
        .def_readwrite(
            "gravity_information_share", &Results::gravity_information_share,
            "Share of the rotational information from the gravity prior, <0 if none")
        .def(
            "__str__",
            [](const Results& r)
            {
                std::stringstream ss;
                r.print(ss);
                return ss.str();
            })
        .def(
            "__repr__",
            [](const Results& r)
            {
                return "Results(pose=" + r.optimal_tf.mean.asString() +
                       ", quality=" + std::to_string(r.quality) +
                       ", nIterations=" + std::to_string(r.nIterations) + ", terminationReason=" +
                       std::string(mrpt::typemeta::TEnumType<mp2p_icp::IterTermReason>::value2name(
                           r.terminationReason)) +
                       ")";
            });

    using mp2p_icp::GravityPrior;
    py::class_<GravityPrior>(
        m, "GravityPrior",
        "Gravity ('verticality') observation for ICP: constrains roll and pitch only")
        .def(py::init<>(), "Default: up=(0,0,1) in both frames, sigma 0.02 rad.")
        .def_readwrite(
            "up_body", &GravityPrior::up_body,
            "Measured 'up' direction in the local (body) frame (mrpt.math.TPoint3D)")
        .def_readwrite("up_map", &GravityPrior::up_map, "'up' direction in the global map frame")
        .def_readwrite("sigma_rad", &GravityPrior::sigma_rad, "Tilt standard deviation [rad]");

    py::class_<IterationInfo>(
        m, "IterationInfo",
        "Information passed to the ICP iteration hook, see ICP.set_iteration_hook()")
        .def_readonly("iteration", &IterationInfo::iteration, "Iteration index, from 0")
        .def_readonly("pose", &IterationInfo::pose, "Current pose estimate (mrpt.poses.CPose3D)")
        .def_readonly(
            "num_pairings", &IterationInfo::num_pairings, "Number of pairings in this iteration")
        .def_readonly(
            "potential_pairings", &IterationInfo::potential_pairings,
            "Maximum number of pairings the matchers could have found")
        .def(
            "__repr__",
            [](const IterationInfo& i)
            {
                return "IterationInfo(iteration=" + std::to_string(i.iteration) +
                       ", pose=" + i.pose.asString() +
                       ", num_pairings=" + std::to_string(i.num_pairings) + ")";
            });

    // ---------------------------------------------------------------------
    // LogRecord
    // ---------------------------------------------------------------------
    using mp2p_icp::LogRecord;
    py::class_<LogRecord::DebugInfoPerIteration>(
        m, "IterationDetails", "Pose and pairings of one ICP iteration, in a LogRecord")
        .def_readwrite(
            "optimalPose", &LogRecord::DebugInfoPerIteration::optimalPose, "mrpt.poses.CPose3D")
        .def_readwrite("pairings", &LogRecord::DebugInfoPerIteration::pairings, "Pairings");

    py::class_<LogRecord, mrpt::serialization::CSerializable, std::shared_ptr<LogRecord>>(
        m, "LogRecord",
        "Inputs and outputs of one ICP run, as saved in `.icplog` files (see "
        "Parameters.generateDebugFiles), or filled by ICP.align(..., log=LogRecord()).")
        .def(py::init<>(), "Empty record.")
        .def_static(
            "from_file", [](const py::object& path) { return load_log(to_path(path)); }, "path"_a,
            "Loads an `.icplog` file. Raises an exception on any error.")
        .def(
            "save",
            [](const LogRecord& self, const py::object& path)
            {
                const auto file = to_path(path);
                bool       ok   = false;
                {
                    py::gil_scoped_release nogil;
                    ok = self.save_to_file(file);
                }
                if (!ok)
                {
                    throw std::runtime_error("Error saving ICP log to: '" + file + "'");
                }
            },
            "path"_a, "Saves the record to an `.icplog` file.")
        .def_property(
            "pcGlobal",
            [](const LogRecord& r)
            { return std::const_pointer_cast<mp2p_icp::metric_map_t>(r.pcGlobal); },
            [](LogRecord& r, const std::shared_ptr<mp2p_icp::metric_map_t>& v) { r.pcGlobal = v; },
            "The global map (MetricMap or None)")
        .def_property(
            "pcLocal",
            [](const LogRecord& r)
            { return std::const_pointer_cast<mp2p_icp::metric_map_t>(r.pcLocal); },
            [](LogRecord& r, const std::shared_ptr<mp2p_icp::metric_map_t>& v) { r.pcLocal = v; },
            "The local map (MetricMap or None)")
        .def_readwrite(
            "initialGuessLocalWrtGlobal", &LogRecord::initialGuessLocalWrtGlobal,
            "Initial guess (mrpt.math.TPose3D)")
        .def_readwrite(
            "prior", &LogRecord::prior, "Optional pose prior (mrpt.poses.CPose3DPDFGaussianInf)")
        .def_readwrite("gravityPrior", &LogRecord::gravityPrior, "Optional GravityPrior")
        .def_readwrite("icpParameters", &LogRecord::icpParameters, "ICP Parameters")
        .def_readwrite("icpResult", &LogRecord::icpResult, "ICP Results")
        .def_readwrite(
            "dynamicVariables", &LogRecord::dynamicVariables,
            "Values of the dynamic variables (dict)")
        .def_readwrite(
            "iterationsDetails", &LogRecord::iterationsDetails,
            "Optional dict {iteration: IterationDetails}, if Parameters.saveIterationDetails")
        .def(
            "__repr__",
            [](const LogRecord& r)
            {
                std::stringstream ss;
                ss << "LogRecord(quality=" << r.icpResult.quality
                   << ", pose=" << r.icpResult.optimal_tf.mean.asString()
                   << ", iterations=" << (r.iterationsDetails ? r.iterationsDetails->size() : 0)
                   << ")";
                return ss.str();
            });

    // ---------------------------------------------------------------------
    // ICP
    // ---------------------------------------------------------------------
    py::class_<PyICP, std::shared_ptr<PyICP>>(
        m, "ICP",
        "An ICP pipeline (matchers, solvers, quality evaluators) with its default parameters. "
        "Create it with ICP.from_yaml() or icp_pipeline_from_yaml(). Not thread-safe: do not "
        "call align() on the same object from several threads at once.")
        .def_static(
            "from_yaml", &make_icp, "config"_a, "variables"_a = py::none(), "verbosity"_a = "INFO",
            "Creates an ICP pipeline from a configuration: a YAML file path, a dict, or an "
            "mrpt.containers.YAML node, with the format of mp2p_icp ICP YAML files. "
            "`${VAR}` and `${VAR|default}` tokens are expanded from `variables` (dict) and the "
            "environment.")
        .def_property(
            "parameters", [](const PyICP& self) { return self.parameters; },
            [](PyICP& self, const std::shared_ptr<mp2p_icp::Parameters>& p)
            { self.parameters = p; },
            "Default Parameters for align(), loaded from the `params` block of the "
            "configuration.")
        .def_property_readonly(
            "class_name",
            [](const PyICP& self) { return std::string(self.icp->GetRuntimeClass()->className); },
            "C++ class of the ICP algorithm")
        .def(
            "align", &align, "local_map"_a, "global_map"_a, "initial_guess"_a = py::none(),
            "parameters"_a = py::none(), "prior"_a = py::none(), "gravity_prior"_a = py::none(),
            "log"_a = py::none(),
            "Aligns two maps: finds the pose of `local_map` with respect to `global_map`, "
            "starting from `initial_guess` (TPose3D, CPose3D, 4x4 matrix, or "
            "[x,y,z,yaw,pitch,roll]; default: identity). `parameters` overrides the default "
            "ones. `prior` is an optional mrpt.poses.CPose3DPDFGaussianInf (information in the "
            "SE(3) tangent space of its mean), `gravity_prior` an optional GravityPrior. If a "
            "LogRecord is passed as `log`, it is filled with the inputs and outputs of this run. "
            "Returns Results.")
        .def(
            "set_iteration_hook",
            [](PyICP& self, const py::object& fn)
            {
                if (!fn.is_none() && !PyCallable_Check(fn.ptr()))
                {
                    throw py::type_error("The iteration hook must be callable or None");
                }
                self.hook = fn;
            },
            "hook"_a,
            "Sets a function called at each ICP iteration with an IterationInfo; if it returns "
            "True, iterations stop (terminationReason=HookRequest). Exceptions raised in it stop "
            "align() and are re-raised. Pass None to remove it.")
        .def(
            "update_variables",
            [](PyICP& self, const std::map<std::string, double>& vars)
            {
                for (const auto& [name, value] : vars)
                {
                    self.paramSource.updateVariable(name, value);
                }
            },
            "variables"_a,
            "Sets the values of variables used in parameter formulas of the pipeline modules "
            "(e.g. `threshold: '2.0*SCALE'`). ICP_ITERATION is always defined by align().")
        .def_property_readonly(
            "variables", [](const PyICP& self) { return self.paramSource.getVariableValues(); },
            "Current values of the formula variables (dict)")
        .def(
            "__repr__",
            [](const PyICP& self)
            {
                return "ICP(" + std::string(self.icp->GetRuntimeClass()->className) + ": " +
                       class_names_of(*self.icp) + ")";
            });

    m.def(
        "icp_pipeline_from_yaml",
        [](const py::object& config, const py::object& variables, const std::string& verbosity)
        {
            auto icp = make_icp(config, variables, verbosity);
            return py::make_tuple(icp, icp->parameters);
        },
        "config"_a, "variables"_a = py::none(), "verbosity"_a = "INFO",
        "Creates an ICP pipeline from a configuration (see ICP.from_yaml()). Returns a tuple "
        "(ICP, Parameters), where Parameters is the same object as ICP.parameters.");
}

}  // namespace mp2p_icp_py
