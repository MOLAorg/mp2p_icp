/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   metric_map.cpp
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <algorithm>
#include <iterator>

IMPLEMENTS_MRPT_OBJECT(
    metric_map_t, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t metric_map_t::serializeGetVersion() const { return 1; }
void    metric_map_t::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << lines;

    out.WriteAs<uint32_t>(planes.size());
    for (const auto& p : planes) out << p.plane << p.centroid;

    out.WriteAs<uint32_t>(lines.size());
    for (const auto& l : lines) out << l;

    out.WriteAs<uint32_t>(layers.size());
    for (const auto& l : layers) out << l.first << *l.second.get();

    out << id << label;  // new in v1

    // Optional user data:
    derivedSerializeTo(out);
}
void metric_map_t::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        case 1:
        {
            in >> lines;
            const auto nPls = in.ReadAs<uint32_t>();
            planes.resize(nPls);
            for (auto& pl : planes) in >> pl.plane >> pl.centroid;

            const auto nLins = in.ReadAs<uint32_t>();
            lines.resize(nLins);
            for (auto& l : lines) in >> l;

            const auto nPts = in.ReadAs<uint32_t>();
            layers.clear();
            for (std::size_t i = 0; i < nPts; i++)
            {
                std::string name;
                in >> name;
                layers[name] = mrpt::ptr_cast<mrpt::maps::CMetricMap>::from(
                    in.ReadObject());
            }

            if (version >= 1) { in >> id >> label; }
            else
            {
                id.reset();
                label.reset();
            }

            // Optional user data:
            derivedSerializeFrom(in);
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

auto metric_map_t::get_visualization(const render_params_t& p) const
    -> std::shared_ptr<mrpt::opengl::CSetOfObjects>
{
    MRPT_START
    auto o = mrpt::opengl::CSetOfObjects::Create();

    get_visualization_planes(*o, p.planes);
    get_visualization_lines(*o, p.lines);
    get_visualization_points(*o, p.points);

    return o;
    MRPT_END
}

void metric_map_t::get_visualization_planes(
    mrpt::opengl::CSetOfObjects& o, const render_params_planes_t& p) const
{
    MRPT_START
    if (!p.visible) return;

    const float pw = p.halfWidth, pf = p.gridSpacing;

    for (const auto& plane : planes)
    {
        auto gl_pl =
            mrpt::opengl::CGridPlaneXY::Create(-pw, pw, -pw, pw, .0, pf);
        gl_pl->setColor_u8(p.color);
        mrpt::math::TPose3D planePose;
        plane.plane.getAsPose3DForcingOrigin(plane.centroid, planePose);
        gl_pl->setPose(planePose);
        o.insert(gl_pl);
    }

    MRPT_END
}

void metric_map_t::get_visualization_lines(
    mrpt::opengl::CSetOfObjects& o, const render_params_lines_t& p) const
{
    MRPT_START

    auto glLin = mrpt::opengl::CSetOfLines::Create();
    glLin->setColor_u8(p.color);

    for (size_t idxLine = 0; idxLine < lines.size(); idxLine++)
    {
        const auto& line    = lines[idxLine];
        double      linLen  = p.length;
        const auto  halfSeg = line.director * (0.5 * linLen);
        glLin->appendLine(line.pBase - halfSeg, line.pBase + halfSeg);
    }
    o.insert(glLin);

    MRPT_END
}

void metric_map_t::get_visualization_points(
    mrpt::opengl::CSetOfObjects& o, const render_params_points_t& p) const
{
    MRPT_START
    // Planes:
    if (!p.visible) return;

    if (!p.perLayer.empty())
    {
        // render only these layers:
        for (const auto& kv : p.perLayer)
        {
            const auto itPts = layers.find(kv.first);
            if (itPts == layers.end())
                THROW_EXCEPTION_FMT(
                    "Rendering parameters given for layer '%s' which does not "
                    "exist in this metric_map_t object",
                    kv.first.c_str());

            get_visualization_map_layer(o, kv.second, itPts->second);
        }
    }
    else
    {
        // render all layers with the same params:
        for (const auto& kv : layers)
        {
            get_visualization_map_layer(o, p.allLayers, kv.second);
        }
    }

    MRPT_END
}

void metric_map_t::get_visualization_map_layer(
    mrpt::opengl::CSetOfObjects& o, const render_params_point_layer_t& p,
    const mrpt::maps::CMetricMap::Ptr& map)
{
    mrpt::maps::CPointsMap::Ptr pts;

    auto voxelMap    = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(map);
    auto voxelRGBMap = std::dynamic_pointer_cast<mrpt::maps::CVoxelMapRGB>(map);
    if (voxelMap || voxelRGBMap)
    {
        if (p.render_voxelmaps_as_points)
        {
            // get occupied voxel XYZ coordinates only:
            if (voxelMap) pts = voxelMap->getOccupiedVoxels();
            if (voxelRGBMap) pts = voxelRGBMap->getOccupiedVoxels();
        }
        else
        {
            // Render as real voxelmaps:
            map->getVisualizationInto(o);
            return;
        }
    }
    else
    {
        pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(map);
    }

    if (!pts ||
        (p.colorMode.has_value() && p.colorMode->keep_original_cloud_color))
    {
        // Not convertible to point maps or user selected their original colors,
        // use its own default renderer:
        map->getVisualizationInto(o);
        return;
    }

    if (pts && pts->empty()) return;  // quick return if empty point cloud

    if (p.colorMode.has_value())
    {
        // color point cloud:
        auto glPts = mrpt::opengl::CPointCloudColoured::Create();
        glPts->loadFromPointsMap(pts.get());

        glPts->setPointSize(p.pointSize);

        mrpt::math::TBoundingBoxf bb;

        if (!p.colorMode->colorMapMinCoord.has_value() ||
            !p.colorMode->colorMapMaxCoord.has_value())
            bb = pts->boundingBox();

        ASSERT_(p.colorMode->recolorizeByCoordinate.has_value());

        const unsigned int coordIdx = static_cast<unsigned int>(
            p.colorMode->recolorizeByCoordinate.value());

        const float coordMin = p.colorMode->colorMapMinCoord.has_value()
                                   ? *p.colorMode->colorMapMinCoord
                                   : bb.min[coordIdx];

        const float coordMax = p.colorMode->colorMapMaxCoord.has_value()
                                   ? *p.colorMode->colorMapMaxCoord
                                   : bb.max[coordIdx];

        glPts->recolorizeByCoordinate(
            coordMin, coordMax, coordIdx, p.colorMode->colorMap);

        o.insert(glPts);
    }
    else
    {
        // uniform color point cloud:
        auto glPts = mrpt::opengl::CPointCloud::Create();
        glPts->loadFromPointsMap(pts.get());

        glPts->setPointSize(p.pointSize);
        glPts->setColor_u8(p.color);

        o.insert(glPts);
    }
}

bool metric_map_t::empty() const
{
    return layers.empty() && lines.empty() && planes.empty();
}
void metric_map_t::clear() { *this = metric_map_t(); }

// TODO(JLBC): Write unit test for mergeWith()

void metric_map_t::merge_with(
    const metric_map_t&                       otherPc,
    const std::optional<mrpt::math::TPose3D>& otherRelativePose)
{
    mrpt::poses::CPose3D pose;
    if (otherRelativePose.has_value())
        pose = mrpt::poses::CPose3D(otherRelativePose.value());

    // Lines:
    if (otherRelativePose.has_value())
    {
        std::transform(
            otherPc.lines.begin(), otherPc.lines.end(),
            std::back_inserter(lines), [&](const mrpt::math::TLine3D& l) {
                return mrpt::math::TLine3D::FromPointAndDirector(
                    pose.composePoint(l.pBase),
                    pose.rotateVector(l.getDirectorVector()));
            });
    }
    else
    {
        std::copy(
            otherPc.lines.begin(), otherPc.lines.end(),
            std::back_inserter(lines));
    }

    // Planes:
    if (otherRelativePose.has_value())
    {
        std::transform(
            otherPc.planes.begin(), otherPc.planes.end(),
            std::back_inserter(planes), [&](const plane_patch_t& l) {
                plane_patch_t g;
                g.centroid = pose.composePoint(l.centroid);

                // TODO(JLBC): Finish rotating planes
                // ...
                THROW_EXCEPTION("To-do");
                return g;
            });
    }
    else
    {
        std::copy(
            otherPc.planes.begin(), otherPc.planes.end(),
            std::back_inserter(planes));
    }

    // Points:
    for (const auto& layer : otherPc.layers)
    {
        const auto& name     = layer.first;
        const auto& otherMap = layer.second;

        // New layer?
        if (layers.count(name) == 0)
        {
            // Make a copy and transform (if applicable):
            layers[name] = std::dynamic_pointer_cast<mrpt::maps::CMetricMap>(
                otherMap->duplicateGetSmartPtr());

            if (otherRelativePose.has_value())
            {
                if (auto pts =
                        std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                            layers[name]);
                    pts)
                {
                    // Transform:
                    pts->changeCoordinatesReference(pose);
                }
                else
                {
                    THROW_EXCEPTION(
                        "Merging with SE(3) transform only available for "
                        "metric maps of point cloud types.");
                }
            }
        }
        else
        {
            // merge with existing layer:
            if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                    layers[name]);
                pts)
            {
                pts->insertAnotherMap(
                    std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(otherMap)
                        .get(),
                    pose);
            }
            else
            {
                THROW_EXCEPTION(
                    "Merging with SE(3) transform only available for "
                    "metric maps of point cloud types.");
            }
        }
    }
}

size_t metric_map_t::size() const
{
    size_t n = 0;

    n += lines.size();
    n += planes.size();
    n += size_points_only();

    return n;
}
size_t metric_map_t::size_points_only() const
{
    size_t n = 0;
    for (const auto& layer : layers)
    {
        if (auto pts =
                std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer.second);
            pts)
        {
            n += pts->size();
        }
    }
    return n;
}

std::string metric_map_t::contents_summary() const
{
    using namespace std::string_literals;

    std::string ret;

    if (id) ret += "id="s + std::to_string(*id) + " "s;
    if (label) ret += "label='"s + *label + "' "s;

    if (empty()) return {ret + "empty"s};

    const auto retAppend = [&ret](const std::string& s) {
        if (!ret.empty()) ret += ", "s;
        ret += s;
    };

    if (!lines.empty()) retAppend(std::to_string(lines.size()) + " lines"s);
    if (!planes.empty()) retAppend(std::to_string(planes.size()) + " planes"s);

    size_t nPts = 0, nVoxels = 0;
    for (const auto& layer : layers)
    {
        ASSERT_(layer.second);
        if (auto pts =
                std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer.second);
            pts)
        {
            nPts += pts->size();
        }
        else if (auto vxs = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(
                     layer.second);
                 vxs)
        {
            nVoxels += vxs->grid().activeCellsCount();
        }
    }

    if (nPts != 0 || nVoxels != 0)
    {
        retAppend(
            mrpt::system::unitsFormat(static_cast<double>(nPts), 2, false) +
            " points, "s +
            mrpt::system::unitsFormat(static_cast<double>(nVoxels), 2, false) +
            " voxels in "s + std::to_string(layers.size()) + " layers ("s);

        for (const auto& layer : layers)
        {
            ret +=
                "\""s + layer.first + "\":"s + layer.second->asString() + " "s;
        }
        ret += ")";
    }

    return ret;
}

bool metric_map_t::save_to_file(const std::string& fileName) const
{
    auto f = mrpt::io::CFileGZOutputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch << *this;

    return true;
}

bool metric_map_t::load_from_file(const std::string& fileName)
{
    auto f = mrpt::io::CFileGZInputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch >> *this;

    return true;
}

metric_map_t::Ptr metric_map_t::get_shared_from_this()
{
    try
    {
        return shared_from_this();
    }
    catch (const std::bad_weak_ptr&)
    {
        // Not created as a shared_ptr.
        return {};
    }
}

metric_map_t::Ptr metric_map_t::get_shared_from_this_or_clone()
{
    Ptr ret = get_shared_from_this();
    if (!ret) ret = std::make_shared<metric_map_t>(*this);
    return ret;
}

metric_map_t::ConstPtr metric_map_t::get_shared_from_this() const
{
    try
    {
        return shared_from_this();
    }
    catch (const std::bad_weak_ptr&)
    {
        // Not created as a shared_ptr.
        return {};
    }
}

metric_map_t::ConstPtr metric_map_t::get_shared_from_this_or_clone() const
{
    ConstPtr ret = get_shared_from_this();
    if (!ret) ret = std::make_shared<metric_map_t>(*this);
    return ret;
}

mrpt::maps::CPointsMap::Ptr metric_map_t::point_layer(
    const layer_name_t& name) const
{
    auto it = layers.find(name);
    if (it == layers.end())
        THROW_EXCEPTION_FMT("Layer '%s' does not exist.", name.c_str());

    const auto& ptr = it->second;
    if (!ptr) return {};  // empty shared_ptr.

    auto ret = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(ptr);
    if (!ret)
        THROW_EXCEPTION_FMT(
            "Layer '%s' is not a point cloud (actual class:'%s').",
            name.c_str(), ptr->GetRuntimeClass()->className);

    return ret;
}

const mrpt::maps::CPointsMap* mp2p_icp::MapToPointsMap(
    const mrpt::maps::CMetricMap& map)
{
    if (auto ptsMap = dynamic_cast<const mrpt::maps::CPointsMap*>(&map); ptsMap)
    {
        return ptsMap;
    }
    if (auto voxelMap = dynamic_cast<const mrpt::maps::CVoxelMap*>(&map);
        voxelMap)
    {
        return voxelMap->getOccupiedVoxels().get();
    }
    if (auto voxelRGBMap = dynamic_cast<const mrpt::maps::CVoxelMapRGB*>(&map);
        voxelRGBMap)
    {
        return voxelRGBMap->getOccupiedVoxels().get();
    }
    return {};
}

mrpt::maps::CPointsMap* mp2p_icp::MapToPointsMap(mrpt::maps::CMetricMap& map)
{
    if (auto ptsMap = dynamic_cast<mrpt::maps::CPointsMap*>(&map); ptsMap)
    {
        return ptsMap;
    }
    if (auto voxelMap = dynamic_cast<mrpt::maps::CVoxelMap*>(&map); voxelMap)
    {
        return voxelMap->getOccupiedVoxels().get();
    }
    if (auto voxelRGBMap = dynamic_cast<mrpt::maps::CVoxelMapRGB*>(&map);
        voxelRGBMap)
    {
        return voxelRGBMap->getOccupiedVoxels().get();
    }
    return {};
}

const mrpt::maps::NearestNeighborsCapable* mp2p_icp::MapToNN(
    const mrpt::maps::CMetricMap& map, bool throwIfNotImplemented)
{
    const auto* ptr =
        dynamic_cast<const mrpt::maps::NearestNeighborsCapable*>(&map);

    if (ptr) return ptr;
    if (!throwIfNotImplemented) return nullptr;

    THROW_EXCEPTION_FMT(
        "The map of type '%s' does not implement the expected interface "
        "mrpt::maps::NearestNeighborsCapable",
        map.GetRuntimeClass()->className);
}
