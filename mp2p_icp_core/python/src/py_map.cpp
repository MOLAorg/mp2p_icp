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
 * @file   py_map.cpp
 * @brief  Python bindings: metric_map_t, point layers to/from NumPy, file I/O
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mp2p_icp/load_xyz_file.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/point_cloud_export.h>
#include <mp2p_icp/render_params.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstring>
#include <sstream>

#include "py_common.h"

namespace py = pybind11;
using namespace pybind11::literals;

using mp2p_icp::metric_map_t;
using mrpt::maps::CGenericPointsMap;
using mrpt::maps::CMetricMap;
using mrpt::maps::CPointsMap;

namespace mp2p_icp_py
{
namespace
{
/** Dict-like view of the layers of a metric map, which keeps the map alive */
struct LayersView
{
    std::shared_ptr<metric_map_t> map;
};

const CPointsMap& points_view_or_throw(const CMetricMap& layer)
{
    const auto* pts = mp2p_icp::MapToPointsMapView(layer);
    if (!pts)
    {
        throw py::type_error(
            std::string("Layer of class '") + layer.GetRuntimeClass()->className +
            "' cannot be converted into a point cloud");
    }
    return *pts;
}

template <typename T>
py::array_t<T> to_numpy(const mrpt::aligned_std_vector<T>* v, size_t n)
{
    py::array_t<T> arr(std::vector<py::ssize_t>{static_cast<py::ssize_t>(n)});
    T*             out = arr.mutable_data();
    std::memset(static_cast<void*>(out), 0, n * sizeof(T));
    if (v)
    {
        std::memcpy(static_cast<void*>(out), v->data(), std::min(n, v->size()) * sizeof(T));
    }
    return arr;
}

py::dict layer_to_numpy(const CMetricMap& layer)
{
    const CPointsMap& pc = points_view_or_throw(layer);
    const size_t      n  = pc.size();

    py::dict d;
    d["x"] = to_numpy(&pc.getPointsBufferRef_x(), n);
    d["y"] = to_numpy(&pc.getPointsBufferRef_y(), n);
    d["z"] = to_numpy(&pc.getPointsBufferRef_z(), n);
    for (const auto& name : pc.getPointFieldNames_float_except_xyz())
    {
        d[py::str(name)] = to_numpy(pc.getPointsBufferRef_float_field(name), n);
    }
    for (const auto& name : pc.getPointFieldNames_double())
    {
        d[py::str(name)] = to_numpy(pc.getPointsBufferRef_double_field(name), n);
    }
    for (const auto& name : pc.getPointFieldNames_uint16())
    {
        d[py::str(name)] = to_numpy(pc.getPointsBufferRef_uint16_field(name), n);
    }
    for (const auto& name : pc.getPointFieldNames_uint8())
    {
        d[py::str(name)] = to_numpy(pc.getPointsBufferRef_uint8_field(name), n);
    }
    for (const auto& name : pc.getPointFieldNames_uint32())
    {
        d[py::str(name)] = to_numpy(pc.getPointsBufferRef_uint32_field(name), n);
    }
    return d;
}

template <typename T>
void copy_into(mrpt::aligned_std_vector<T>* dst, const py::array& src, const std::string& name)
{
    if (!dst)
    {
        throw std::runtime_error("Internal error: missing buffer for field '" + name + "'");
    }
    const auto a = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(src);
    std::memcpy(static_cast<void*>(dst->data()), a.data(), dst->size() * sizeof(T));
}

CGenericPointsMap::Ptr layer_from_numpy(const py::object& data)
{
    const auto np = py::module_::import("numpy");

    // Collect named 1-D columns:
    std::vector<std::pair<std::string, py::array>> columns;
    if (py::isinstance<py::array>(data) || py::isinstance<py::list>(data) ||
        py::isinstance<py::tuple>(data))
    {
        const auto arr = py::array::ensure(np.attr("asarray")(data));
        if (!arr || arr.ndim() != 2 || (arr.shape(1) != 2 && arr.shape(1) != 3))
        {
            throw py::value_error(
                "Expected an array of shape (N,3) or (N,2) with point coordinates");
        }
        const auto  cols     = arr.attr("T");
        const char* names[3] = {"x", "y", "z"};
        for (py::ssize_t i = 0; i < arr.shape(1); i++)
        {
            columns.emplace_back(
                names[i], py::array::ensure(np.attr("ascontiguousarray")(
                              cols.attr("__getitem__")(i), py::arg("dtype") = "float32")));
        }
    }
    else if (py::hasattr(data, "items"))
    {
        for (const auto& kv : data.attr("items")())
        {
            const auto item = kv.cast<py::tuple>();
            columns.emplace_back(
                item[0].cast<std::string>(),
                py::array::ensure(np.attr("ascontiguousarray")(item[1])));
        }
    }
    else
    {
        throw py::type_error(
            "Expected a dict {field_name: 1-D array} or an array of shape (N,3) or (N,2)");
    }

    // Validate:
    std::optional<size_t> n;
    bool                  hasX = false;
    bool                  hasY = false;
    for (const auto& [name, arr] : columns)
    {
        if (!arr || arr.ndim() != 1)
        {
            throw py::value_error("Field '" + name + "' must be a 1-D array");
        }
        const auto len = static_cast<size_t>(arr.shape(0));
        if (n && *n != len)
        {
            throw py::value_error(
                "All fields must have the same length, but field '" + name + "' has " +
                std::to_string(len) + " elements instead of " + std::to_string(*n));
        }
        n    = len;
        hasX = hasX || name == "x";
        hasY = hasY || name == "y";
    }
    if (!hasX || !hasY)
    {
        throw py::value_error("The fields 'x' and 'y' are mandatory");
    }

    auto pc = CGenericPointsMap::Create();

    // Register fields by their dtype:
    enum class Kind
    {
        XYZ,
        FLOAT,
        DOUBLE,
        UINT16,
        UINT8,
        UINT32
    };
    std::vector<Kind> kinds;
    for (const auto& [name, arr] : columns)
    {
        if (name == "x" || name == "y" || name == "z")
        {
            if (arr.dtype().kind() != 'f' && arr.dtype().kind() != 'i' && arr.dtype().kind() != 'u')
            {
                throw py::type_error("Field '" + name + "' must be numeric");
            }
            kinds.push_back(Kind::XYZ);
            continue;
        }
        // By kind and size: equal dtypes may be different objects, or have a
        // non-native byte order (converted when copying).
        const auto dt   = arr.dtype();
        const char kind = dt.kind();
        const auto size = dt.itemsize();
        if (kind == 'f' && size == 4)
        {
            pc->registerField_float(name);
            kinds.push_back(Kind::FLOAT);
        }
        else if (kind == 'f' && size == 8)
        {
            pc->registerField_double(name);
            kinds.push_back(Kind::DOUBLE);
        }
        else if (kind == 'u' && size == 2)
        {
            pc->registerField_uint16(name);
            kinds.push_back(Kind::UINT16);
        }
        else if (kind == 'u' && size == 1)
        {
            pc->registerField_uint8(name);
            kinds.push_back(Kind::UINT8);
        }
        else if (kind == 'u' && size == 4)
        {
            pc->registerField_uint32(name);
            kinds.push_back(Kind::UINT32);
        }
        else
        {
            throw py::type_error(
                "Field '" + name + "' has an unsupported dtype '" +
                py::str(static_cast<const py::handle&>(dt)).cast<std::string>() +
                "'. Supported: float32, float64, uint8, uint16, uint32 (convert it with "
                ".astype())");
        }
    }

    pc->resize(*n);

    for (size_t i = 0; i < columns.size(); i++)
    {
        const auto& [name, arr] = columns[i];
        switch (kinds[i])
        {
            case Kind::XYZ:
            case Kind::FLOAT:
                copy_into(pc->getPointsBufferRef_float_field(name), arr, name);
                break;
            case Kind::DOUBLE:
                copy_into(pc->getPointsBufferRef_double_field(name), arr, name);
                break;
            case Kind::UINT16:
                copy_into(pc->getPointsBufferRef_uint16_field(name), arr, name);
                break;
            case Kind::UINT8:
                copy_into(pc->getPointsBufferRef_uint8_field(name), arr, name);
                break;
            case Kind::UINT32:
                copy_into(pc->getPointsBufferRef_uint32_field(name), arr, name);
                break;
        }
    }
    pc->mark_as_modified();
    return pc;
}

/** Accepts an mrpt.maps.CMetricMap, or anything accepted by layer_from_numpy() */
CMetricMap::Ptr to_layer(const py::object& obj)
{
    if (py::isinstance<CMetricMap>(obj))
    {
        return obj.cast<CMetricMap::Ptr>();
    }
    return layer_from_numpy(obj);
}

std::optional<mrpt::poses::CPose3D> to_optional_pose(const py::object& obj)
{
    const auto p = to_optional_tpose3d(obj);
    if (!p)
    {
        return std::nullopt;
    }
    return mrpt::poses::CPose3D(*p);
}

std::vector<std::string> to_fields(const py::object& obj)
{
    if (obj.is_none())
    {
        return {};
    }
    if (py::isinstance<py::str>(obj))
    {
        throw py::type_error("'fields' must be a list of field names, not a string");
    }
    return obj.cast<std::vector<std::string>>();
}

mrpt::io::CompressionOptions to_compression(const std::string& name, const py::object& level)
{
    mrpt::io::CompressionOptions co;
    if (name == "zstd")
    {
        co.type  = mrpt::io::CompressionType::Zstd;
        co.level = 1;
    }
    else if (name == "gzip")
    {
        co.type  = mrpt::io::CompressionType::Gzip;
        co.level = 1;
    }
    else if (name == "none")
    {
        co.type = mrpt::io::CompressionType::None;
    }
    else
    {
        throw py::value_error("Invalid compression '" + name + "': use 'zstd', 'gzip' or 'none'");
    }
    if (!level.is_none())
    {
        co.level = level.cast<int>();
    }
    return co;
}

void load_map(metric_map_t& m, const std::string& path)
{
    if (!mrpt::system::fileExists(path))
    {
        raise_file_not_found("Metric map file not found: '" + path + "'");
    }
    load_with_class_plugins(
        [&]()
        {
            std::string errMsg;
            bool        ok = false;
            {
                py::gil_scoped_release nogil;
                ok = m.load_from_file(path, errMsg);
            }
            if (!ok)
            {
                throw std::runtime_error(errMsg);
            }
        });
}

py::bytes serialize(const mrpt::serialization::CSerializable& o)
{
    mrpt::io::CMemoryStream buf;
    auto                    arch = mrpt::serialization::archiveFrom(buf);
    arch << o;
    return {
        static_cast<const char*>(buf.getRawBufferData()),
        static_cast<size_t>(buf.getTotalBytesCount())};
}

void deserialize(mrpt::serialization::CSerializable& o, const py::bytes& data)
{
    const std::string_view sv = data;
    load_with_class_plugins(
        [&]()
        {
            mrpt::io::CMemoryStream buf;
            buf.assignMemoryNotOwn(sv.data(), sv.size());
            auto arch = mrpt::serialization::archiveFrom(buf);
            arch >> o;
        });
}

std::shared_ptr<metric_map_t> deep_copy(const metric_map_t& m)
{
    auto ret = std::make_shared<metric_map_t>(m);
    for (auto& [name, layer] : ret->layers)
    {
        if (layer)
        {
            layer = std::dynamic_pointer_cast<CMetricMap>(layer->duplicateGetSmartPtr());
        }
    }
    return ret;
}

}  // namespace

void init_map(py::module_& m)
{
    // ---------------------------------------------------------------------
    // Georeferencing
    // ---------------------------------------------------------------------
    using Georef = metric_map_t::Georeferencing;
    py::class_<Georef>(
        m, "Georeferencing",
        "Placement of a metric map with respect to the Earth: the geodetic coordinates of the "
        "ENU frame origin, and the pose of the map frame in that ENU frame.")
        .def(py::init<>(), "Default constructor.")
        .def(
            py::init(
                [](const mrpt::topography::TGeodeticCoords& geo,
                   const mrpt::poses::CPose3DPDFGaussian&   T)
                {
                    Georef g;
                    g.geo_coord    = geo;
                    g.T_enu_to_map = T;
                    return g;
                }),
            "geo_coord"_a, "T_enu_to_map"_a, "Constructor from its two fields.")
        .def_readwrite(
            "geo_coord", &Georef::geo_coord,
            "Geodetic coordinates (WGS-84) of the ENU frame origin "
            "(mrpt.topography.TGeodeticCoords)")
        .def_readwrite(
            "T_enu_to_map", &Georef::T_enu_to_map,
            "Pose of the map frame in the ENU frame (mrpt.poses.CPose3DPDFGaussian)")
        .def(
            "as_dict", [](const Georef& g) { return yaml_to_python(mp2p_icp::ToYAML(g)); },
            "Returns the georeferencing as a dict, in the YAML format used by mp2p_icp tools.")
        .def_static(
            "from_dict",
            [](const py::object& d) -> std::optional<Georef>
            { return mp2p_icp::FromYAML(python_to_yaml(d)); },
            "d"_a, "Builds a georeferencing from a dict, as returned by as_dict().")
        .def(
            "__repr__",
            [](const Georef& g)
            {
                std::stringstream ss;
                ss << "Georeferencing(lat=" << g.geo_coord.lat.getDecimalValue()
                   << ", lon=" << g.geo_coord.lon.getDecimalValue() << ", h=" << g.geo_coord.height
                   << ", T_enu_to_map=" << g.T_enu_to_map.mean.asString() << ")";
                return ss.str();
            });

    // ---------------------------------------------------------------------
    // plane_patch_t
    // ---------------------------------------------------------------------
    py::class_<mp2p_icp::plane_patch_t>(
        m, "PlanePatch", "A plane patch: an infinite plane, plus the centroid of its points.")
        .def(py::init<>(), "Default constructor.")
        .def(
            py::init<const mrpt::math::TPlane3D&, const mrpt::math::TPoint3D&>(), "plane"_a,
            "centroid"_a, "Constructor from a plane and its centroid.")
        .def_readwrite("plane", &mp2p_icp::plane_patch_t::plane, "The plane (mrpt.math.TPlane)")
        .def_readwrite(
            "centroid", &mp2p_icp::plane_patch_t::centroid, "Centroid (mrpt.math.TPoint3D)")
        .def_readwrite("area", &mp2p_icp::plane_patch_t::area, "Surface area [m2], 0 if unknown")
        .def_readwrite(
            "num_points", &mp2p_icp::plane_patch_t::num_points,
            "Number of points the patch was fitted from, 0 if unknown")
        .def(
            "__repr__",
            [](const mp2p_icp::plane_patch_t& p) {
                return "PlanePatch(plane=" + p.plane.asString() +
                       ", centroid=" + p.centroid.asString() + ")";
            });

    // ---------------------------------------------------------------------
    // Render parameters
    // ---------------------------------------------------------------------
    py::class_<mp2p_icp::color_mode_t>(
        m, "ColorMode", "Point cloud coloring mode, for RenderParamsPointLayer.colorMode")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "keep_original_cloud_color", &mp2p_icp::color_mode_t::keep_original_cloud_color,
            "Use the RGB or intensity channels of the cloud, if present")
        .def_readwrite(
            "colorMap", &mp2p_icp::color_mode_t::colorMap, "Color map (mrpt.img.TColormap)")
        .def_readwrite(
            "recolorizeByField", &mp2p_icp::color_mode_t::recolorizeByField,
            "If set, the point field used to colorize points (e.g. 'z')")
        .def_readwrite(
            "colorMapMinCoord", &mp2p_icp::color_mode_t::colorMapMinCoord,
            "Optional fixed field value mapped to the color map start")
        .def_readwrite(
            "colorMapMaxCoord", &mp2p_icp::color_mode_t::colorMapMaxCoord,
            "Optional fixed field value mapped to the color map end")
        .def_readwrite(
            "autoBoundingBoxOutliersPercentile",
            &mp2p_icp::color_mode_t::autoBoundingBoxOutliersPercentile,
            "Optional percentile of outliers ignored to compute the field limits");

    py::class_<mp2p_icp::render_params_point_layer_t>(
        m, "RenderParamsPointLayer", "Rendering options for one point layer")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "pointSize", &mp2p_icp::render_params_point_layer_t::pointSize, "Point size, in pixels")
        .def_readwrite(
            "color", &mp2p_icp::render_params_point_layer_t::color,
            "Fixed color for all points (mrpt.img.TColor). Ignored if colorMode is set.")
        .def_readwrite(
            "force_alpha_channel", &mp2p_icp::render_params_point_layer_t::force_alpha_channel,
            "Overwrite the alpha channel, even for RGBA clouds")
        .def_readwrite(
            "colorMode", &mp2p_icp::render_params_point_layer_t::colorMode,
            "Optional ColorMode, which overrides `color`")
        .def_readwrite(
            "render_voxelmaps_as_points",
            &mp2p_icp::render_params_point_layer_t::render_voxelmaps_as_points,
            "Render voxel maps as points instead of cubes")
        .def_readwrite(
            "render_voxelmaps_free_space",
            &mp2p_icp::render_params_point_layer_t::render_voxelmaps_free_space,
            "Render the free space of voxel maps")
        .def_readwrite(
            "hideBackFacing", &mp2p_icp::render_params_point_layer_t::hideBackFacing,
            "Hide points seen from behind (needs normal or view-vector fields)")
        .def_readwrite(
            "backFacingMaxAngleNormals_deg",
            &mp2p_icp::render_params_point_layer_t::backFacingMaxAngleNormals_deg,
            "For hideBackFacing, with normals [deg]")
        .def_readwrite(
            "backFacingMaxAngleView_deg",
            &mp2p_icp::render_params_point_layer_t::backFacingMaxAngleView_deg,
            "For hideBackFacing, with view vectors [deg]");

    py::class_<mp2p_icp::render_params_points_t>(
        m, "RenderParamsPoints", "Rendering options for the point layers")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "visible", &mp2p_icp::render_params_points_t::visible, "Render points at all")
        .def_readwrite(
            "allLayers", &mp2p_icp::render_params_points_t::allLayers,
            "Options for all layers, used if perLayer is empty")
        .def_readwrite(
            "perLayer", &mp2p_icp::render_params_points_t::perLayer,
            "If not empty, only these layers are rendered, each with its options. It is "
            "returned as a copy: assign a whole dict to change it.");

    py::class_<mp2p_icp::render_params_planes_t>(
        m, "RenderParamsPlanes", "Rendering options for planes")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("visible", &mp2p_icp::render_params_planes_t::visible, "Render planes")
        .def_readwrite("halfWidth", &mp2p_icp::render_params_planes_t::halfWidth, "Half width [m]")
        .def_readwrite(
            "gridSpacing", &mp2p_icp::render_params_planes_t::gridSpacing, "Grid spacing [m]")
        .def_readwrite("color", &mp2p_icp::render_params_planes_t::color, "mrpt.img.TColor");

    py::class_<mp2p_icp::render_params_lines_t>(
        m, "RenderParamsLines", "Rendering options for lines")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("visible", &mp2p_icp::render_params_lines_t::visible, "Render lines")
        .def_readwrite("color", &mp2p_icp::render_params_lines_t::color, "mrpt.img.TColor")
        .def_readwrite(
            "length", &mp2p_icp::render_params_lines_t::length, "Length of all lines [m]")
        .def_readwrite(
            "lengths", &mp2p_icp::render_params_lines_t::lengths,
            "Optional individual lengths [m]");

    py::class_<mp2p_icp::render_params_t>(
        m, "RenderParams", "Rendering options for MetricMap.get_visualization()")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("planes", &mp2p_icp::render_params_t::planes, "RenderParamsPlanes")
        .def_readwrite("lines", &mp2p_icp::render_params_t::lines, "RenderParamsLines")
        .def_readwrite("points", &mp2p_icp::render_params_t::points, "RenderParamsPoints");

    // ---------------------------------------------------------------------
    // metric_map_t
    // ---------------------------------------------------------------------
    py::class_<metric_map_t, mrpt::serialization::CSerializable, std::shared_ptr<metric_map_t>> mm(
        m, "MetricMap",
        "Container of named layers (point clouds, voxel maps, grid maps...), lines and "
        "planes, with optional id, label, metadata and georeferencing. This is "
        "mp2p_icp::metric_map_t, stored in `.mm` files.");

    py::class_<LayersView>(
        m, "LayersView",
        "Dict-like view of the layers of a MetricMap: names (str) to mrpt.maps.CMetricMap. "
        "Assigning a NumPy array (N,3) or a dict of field arrays creates a CGenericPointsMap.")
        .def(
            "__getitem__",
            [](const LayersView& v, const std::string& name)
            {
                const auto it = v.map->layers.find(name);
                if (it == v.map->layers.end())
                {
                    throw py::key_error(name);
                }
                return it->second;
            },
            "name"_a)
        .def(
            "__setitem__",
            [](LayersView& v, const std::string& name, const py::object& layer)
            { v.map->layers[name] = to_layer(layer); },
            "name"_a, "layer"_a)
        .def(
            "__delitem__",
            [](LayersView& v, const std::string& name)
            {
                if (v.map->layers.erase(name) == 0)
                {
                    throw py::key_error(name);
                }
            },
            "name"_a)
        .def(
            "__contains__",
            [](const LayersView& v, const std::string& name)
            { return v.map->layers.count(name) != 0; },
            "name"_a)
        .def("__len__", [](const LayersView& v) { return v.map->layers.size(); })
        .def(
            "__iter__",
            [](const LayersView& v)
            {
                py::list keys;
                for (const auto& [name, _] : v.map->layers)
                {
                    keys.append(name);
                }
                return keys.attr("__iter__")();
            })
        .def(
            "keys",
            [](const LayersView& v)
            {
                std::vector<std::string> keys;
                for (const auto& [name, _] : v.map->layers)
                {
                    keys.push_back(name);
                }
                return keys;
            },
            "Returns a list with the layer names.")
        .def(
            "values",
            [](const LayersView& v)
            {
                std::vector<CMetricMap::Ptr> values;
                for (const auto& [_, layer] : v.map->layers)
                {
                    values.push_back(layer);
                }
                return values;
            },
            "Returns a list with the layers.")
        .def(
            "items",
            [](const LayersView& v)
            {
                std::vector<std::pair<std::string, CMetricMap::Ptr>> items;
                for (const auto& kv : v.map->layers)
                {
                    items.emplace_back(kv);
                }
                return items;
            },
            "Returns a list of (name, layer) tuples.")
        .def(
            "get",
            [](const LayersView& v, const std::string& name, const py::object& def) -> py::object
            {
                const auto it = v.map->layers.find(name);
                if (it == v.map->layers.end())
                {
                    return def;
                }
                return py::cast(it->second);
            },
            "name"_a, "default"_a = py::none(),
            "Returns a layer, or `default` if it does not exist.")
        .def(
            "__repr__",
            [](const LayersView& v)
            {
                std::string s = "{";
                for (const auto& [name, layer] : v.map->layers)
                {
                    if (s.size() > 1)
                    {
                        s += ", ";
                    }
                    s += "'" + name + "': " +
                         (layer ? std::string(layer->GetRuntimeClass()->className) : "None");
                }
                return s + "}";
            });

    mm.def(py::init<>(), "Creates an empty map.")
        .def_static(
            "from_file",
            [](const py::object& path)
            {
                auto ret = std::make_shared<metric_map_t>();
                load_map(*ret, to_path(path));
                return ret;
            },
            "path"_a, "Loads a map from a `.mm` file. Raises an exception on any error.")
        .def(
            "load",
            [](metric_map_t& self, const py::object& path) { load_map(self, to_path(path)); },
            "path"_a,
            "Loads the map from a `.mm` file, replacing its contents. Raises an exception on any "
            "error.")
        .def(
            "save",
            [](const metric_map_t& self, const py::object& path, const std::string& compression,
               const py::object& level)
            {
                const auto co   = to_compression(compression, level);
                const auto file = to_path(path);
                bool       ok   = false;
                {
                    py::gil_scoped_release nogil;
                    ok = self.save_to_file(file, co);
                }
                if (!ok)
                {
                    throw std::runtime_error("Error saving metric map to: '" + file + "'");
                }
            },
            "path"_a, "compression"_a = "zstd", "level"_a = py::none(),
            "Saves the map to a `.mm` file. `compression`: 'zstd' (default), 'gzip' or 'none'; "
            "`level`: optional compression level.")
        .def_property_readonly(
            "layers", [](const std::shared_ptr<metric_map_t>& self) { return LayersView{self}; },
            "Dict-like view of the map layers (mrpt.maps.CMetricMap), by name.")
        .def_readwrite("id", &metric_map_t::id, "Optional numeric ID (int or None)")
        .def_readwrite("label", &metric_map_t::label, "Optional label (str or None)")
        .def_property(
            "metadata",
            [](const metric_map_t& self) -> py::object
            {
                auto d = yaml_to_python(self.metadata);
                return d.is_none() ? py::dict() : d;
            },
            [](metric_map_t& self, const py::object& d)
            {
                auto y = python_to_yaml(d);
                if (!y.isMap() && !y.isNullNode())
                {
                    throw py::type_error("metadata must be a dict");
                }
                if (y.isNullNode())
                {
                    y = mrpt::containers::yaml::Map();
                }
                self.metadata = y;
            },
            "User metadata, as a dict. It is returned as a copy: assign a whole dict to change "
            "it.")
        .def_readwrite(
            "georeferencing", &metric_map_t::georeferencing,
            "Optional Georeferencing. It is returned as a copy: assign it to change it.")
        .def_readwrite(
            "lines", &metric_map_t::lines,
            "3D lines, as a list of mrpt.math.TLine3D. It is returned as a copy: assign a whole "
            "list to change it.")
        .def_readwrite(
            "planes", &metric_map_t::planes,
            "Plane patches, as a list of PlanePatch. It is returned as a copy: assign a whole "
            "list to change it.")
        .def("empty", &metric_map_t::empty, "True if all layers, lines and planes are empty.")
        .def(
            "size", &metric_map_t::size,
            "Overall number of elements (points, voxels, lines, planes).")
        .def(
            "size_points_only", &metric_map_t::size_points_only,
            "Overall number of points, including all layers.")
        .def("clear", &metric_map_t::clear, "Clears all contents.")
        .def(
            "contents_summary", &metric_map_t::contents_summary,
            "Returns a string summarizing the map contents.")
        .def(
            "merge_with",
            [](metric_map_t& self, const metric_map_t& other, const py::object& pose)
            {
                const auto p = to_optional_tpose3d(pose);
                {
                    py::gil_scoped_release nogil;
                    self.merge_with(other, p);
                }
            },
            "other"_a, "other_relative_pose"_a = py::none(),
            "Merges the contents of another map into this one, with an optional relative pose "
            "of `other` (any pose accepted by the ICP functions). Point layers with the same "
            "name are merged; others are added.")
        .def(
            "point_layer",
            [](const metric_map_t& self, const std::string& name)
            {
                if (self.layers.count(name) == 0)
                {
                    throw py::key_error(name);
                }
                return self.point_layer(name);
            },
            "name"_a,
            "Returns a layer as an mrpt.maps.CPointsMap, or raises if it is not a point cloud.")
        .def(
            "get_visualization",
            [](const metric_map_t& self, const std::optional<mp2p_icp::render_params_t>& p)
            { return self.get_visualization(p.value_or(mp2p_icp::render_params_t())); },
            "params"_a = py::none(),
            "Returns an mrpt.viz.CSetOfObjects with a 3D view of the map, with optional "
            "RenderParams.")
        .def(
            "copy", [](const metric_map_t& self) { return deep_copy(self); },
            "Returns a deep copy of the map, with copies of all its layers.")
        .def(
            "__copy__",
            [](const metric_map_t& self) { return std::make_shared<metric_map_t>(self); },
            "Shallow copy: a new map whose layers are the same objects as in this one.")
        .def(
            "__deepcopy__",
            [](const metric_map_t& self, const py::object&) { return deep_copy(self); }, "memo"_a)
        .def(py::pickle(
            [](const metric_map_t& self) { return serialize(self); },
            [](const py::bytes& data)
            {
                auto ret = std::make_shared<metric_map_t>();
                deserialize(*ret, data);
                return ret;
            }))
        .def("__str__", &metric_map_t::contents_summary)
        .def(
            "__repr__",
            [](const metric_map_t& self) { return "MetricMap(" + self.contents_summary() + ")"; });

    m.attr("PT_LAYER_RAW")             = metric_map_t::PT_LAYER_RAW;
    m.attr("PT_LAYER_PLANE_CENTROIDS") = metric_map_t::PT_LAYER_PLANE_CENTROIDS;

    // ---------------------------------------------------------------------
    // Point layers <-> NumPy
    // ---------------------------------------------------------------------
    m.def(
        "layer_to_numpy", &layer_to_numpy, "layer"_a,
        "Returns a copy of all the per-point fields of a point cloud layer, as a dict "
        "{field_name: 1-D NumPy array}. Keys are 'x', 'y', 'z' (float32), then the float32, "
        "float64, uint16, uint8 and uint32 fields. Layers that are not point clouds, but "
        "provide one (e.g. voxel maps), are converted through that point cloud view.");

    m.def(
        "layer_from_numpy", &layer_from_numpy, "data"_a,
        "Creates an mrpt.maps.CGenericPointsMap from a dict {field_name: 1-D array} or from an "
        "array of shape (N,3) or (N,2). 'x' and 'y' are mandatory; 'z' defaults to zeros. "
        "Coordinates are stored as float32; other fields keep their dtype: float32, float64, "
        "uint8, uint16 or uint32.");

    m.def(
        "layer_points_view",
        [](const CMetricMap::Ptr& layer) -> CPointsMap::Ptr
        {
            if (auto pts = std::dynamic_pointer_cast<CPointsMap>(layer); pts)
            {
                return pts;
            }
            // Copy, so the returned object does not depend on the layer lifetime:
            const CPointsMap& view = points_view_or_throw(*layer);
            auto              ret  = CGenericPointsMap::Create();
            ret->insertAnotherMap(&view, mrpt::poses::CPose3D::Identity());
            return ret;
        },
        "layer"_a,
        "Returns the layer itself if it is an mrpt.maps.CPointsMap, or a copy of the point cloud "
        "it provides otherwise (e.g. voxel maps).");

    // ---------------------------------------------------------------------
    // File I/O
    // ---------------------------------------------------------------------
    m.def(
        "load_xyz_file",
        [](const py::object& path)
        {
            const auto file = to_path(path);
            if (!mrpt::system::fileExists(file))
            {
                raise_file_not_found("File not found: '" + file + "'");
            }
            return mp2p_icp::load_xyz_file(file);
        },
        "path"_a,
        "Loads a point cloud from a text file with one 'x y z' point per row, optionally "
        "gzip-compressed (.gz). Returns an mrpt.maps.CSimplePointsMap.");

    m.def(
        "save_ply",
        [](const CMetricMap& layer, const py::object& path, bool binary, const py::object& fields,
           bool ignore_missing_fields, const py::object& transform)
        {
            mp2p_icp::ply_export_options_t o;
            o.binary                    = binary;
            o.fields                    = to_fields(fields);
            o.ignore_missing_fields     = ignore_missing_fields;
            o.transform                 = to_optional_pose(transform);
            const auto&            pc   = points_view_or_throw(layer);
            const auto             file = to_path(path);
            py::gil_scoped_release nogil;
            mp2p_icp::save_point_cloud_to_ply(pc, file, o);
        },
        "layer"_a, "path"_a, "binary"_a = true, "fields"_a = py::none(),
        "ignore_missing_fields"_a = false, "transform"_a = py::none(),
        "Saves a point cloud layer as PLY, with all its fields (or those in `fields`, in that "
        "order). Colors are written as red, green, blue. `transform` optionally transforms the "
        "points (e.g. georeferencing.T_enu_to_map.mean, to export in the ENU frame).");

    m.def(
        "save_las",
        [](const CMetricMap& layer, const py::object& path, const py::object& fields,
           const std::string& system_id, const std::string& generating_software,
           const py::object& transform, bool geodetic, const std::optional<Georef>& georef)
        {
            mp2p_icp::las_export_options_t o;
            o.fields                    = to_fields(fields);
            o.system_id                 = system_id;
            o.generating_software       = generating_software;
            o.transform                 = to_optional_pose(transform);
            o.geodetic                  = geodetic;
            o.georeferencing            = georef;
            const auto&            pc   = points_view_or_throw(layer);
            const auto             file = to_path(path);
            py::gil_scoped_release nogil;
            mp2p_icp::save_point_cloud_to_las(pc, file, o);
        },
        "layer"_a, "path"_a, "fields"_a = py::none(), "system_id"_a = "mp2p_icp",
        "generating_software"_a = "MOLA mp2p_icp", "transform"_a = py::none(), "geodetic"_a = false,
        "georeferencing"_a = py::none(),
        "Saves a point cloud layer as LAS 1.4 (point format 8, plus extra dimensions for "
        "non-standard fields). With `geodetic=True`, points are written as WGS-84 "
        "longitude/latitude/height using `georeferencing`.");

    m.def(
        "save_txt",
        [](const CMetricMap& layer, const py::object& path, bool header, const py::object& fields,
           const py::object& transform)
        {
            mp2p_icp::txt_export_options_t o;
            o.header                  = header;
            o.fields                  = to_fields(fields);
            o.transform               = to_optional_pose(transform);
            const auto&            pc = points_view_or_throw(layer);
            const auto             f  = to_path(path);
            py::gil_scoped_release nogil;
            mp2p_icp::save_point_cloud_to_txt(pc, f, o);
        },
        "layer"_a, "path"_a, "header"_a = true, "fields"_a = py::none(), "transform"_a = py::none(),
        "Saves a point cloud layer as a space-separated text file, one point per row, with an "
        "optional header line with the field names.");

    m.def(
        "load_plugins",
        [](const std::string& modules)
        {
            std::string errMsg;
            if (!mrpt::system::loadPluginModules(modules, errMsg))
            {
                throw std::runtime_error(errMsg);
            }
        },
        "modules"_a = "libmola_metric_maps.so",
        "Loads one or more (comma separated) shared libraries with additional MRPT classes, "
        "e.g. map layer types or ICP pipeline modules. Default: libmola_metric_maps.so, needed "
        "to load maps built with MOLA.");

    m.def(
        "layer_visualization",
        [](const CMetricMap::Ptr&                                      layer,
           const std::optional<mp2p_icp::render_params_point_layer_t>& p)
        {
            auto o = mrpt::viz::CSetOfObjects::Create();
            metric_map_t::get_visualization_map_layer(
                *o, p.value_or(mp2p_icp::render_params_point_layer_t()), layer);
            return o;
        },
        "layer"_a, "params"_a = py::none(),
        "Returns an mrpt.viz.CSetOfObjects with a 3D view of a single layer, with optional "
        "RenderParamsPointLayer.");
}

}  // namespace mp2p_icp_py
