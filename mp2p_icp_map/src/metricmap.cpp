/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   metric_map.cpp
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/distributions.h>  // confidenceIntervals()
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
#include <sstream>

IMPLEMENTS_MRPT_OBJECT(metric_map_t, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t metric_map_t::serializeGetVersion() const { return 5; }
void    metric_map_t::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << lines;

    out.WriteAs<uint32_t>(planes.size());
    for (const auto& p : planes)
    {
        out << p.plane << p.centroid;
    }

    out.WriteAs<uint32_t>(lines.size());
    for (const auto& l : lines)
    {
        out << l;
    }

    out.WriteAs<uint32_t>(layers.size());
    for (const auto& l : layers)
    {
        out << l.first << *l.second.get();
    }

    out << id << label;  // new in v1

    // new in v4: delegate to external function:
    out << georeferencing;

    // new in v5:
    bool hasMetadata = !metadata.isNullNode() && !metadata.empty();
    out << hasMetadata;
    if (hasMetadata)
    {
        std::stringstream ss;
        metadata.printAsYAML(ss);
        out << ss.str();
    }

    // Optional user data:
    derivedSerializeTo(out);
}
void metric_map_t::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        {
            in >> lines;
            const auto nPls = in.ReadAs<uint32_t>();
            planes.resize(nPls);
            for (auto& pl : planes)
            {
                in >> pl.plane >> pl.centroid;
            }

            const auto nLins = in.ReadAs<uint32_t>();
            lines.resize(nLins);
            for (auto& l : lines)
            {
                in >> l;
            }

            const auto nPts = in.ReadAs<uint32_t>();
            layers.clear();
            for (std::size_t i = 0; i < nPts; i++)
            {
                std::string name;
                in >> name;
                layers[name] = mrpt::ptr_cast<mrpt::maps::CMetricMap>::from(in.ReadObject());
            }

            if (version >= 1)
            {
                in >> id >> label;
            }
            else
            {
                id.reset();
                label.reset();
            }

            georeferencing.reset();

            if ((version >= 2 && version < 4) && in.ReadAs<bool>())
            {
                auto& g = georeferencing.emplace();
                in >> g.geo_coord.lat.decimal_value >> g.geo_coord.lon.decimal_value >>
                    g.geo_coord.height;
                if (version >= 3)
                {
                    in >> g.T_enu_to_map;
                }
                else
                {
                    in >> g.T_enu_to_map.mean;
                }
            }

            // delegated function:
            if (version >= 4)
            {
                in >> georeferencing;
            }

            metadata.clear();
            if (version >= 5)
            {
                // new in v5:
                const bool hasMetadata = in.ReadAs<bool>();
                if (hasMetadata)
                {
                    std::string metadata_str;
                    in >> metadata_str;
                    std::stringstream ss(metadata_str);
                    metadata.loadFromStream(ss);
                }
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
    if (!p.visible)
    {
        return;
    }

    const auto pw = static_cast<float>(p.halfWidth);
    const auto pf = static_cast<float>(p.gridSpacing);

    for (const auto& plane : planes)
    {
        auto gl_pl = mrpt::opengl::CGridPlaneXY::Create(-pw, pw, -pw, pw, .0, pf);
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
    if (!p.visible)
    {
        return;
    }

    if (!p.perLayer.empty())
    {
        // render only these layers:
        for (const auto& kv : p.perLayer)
        {
            const auto itPts = layers.find(kv.first);
            if (itPts == layers.end())
            {
                THROW_EXCEPTION_FMT(
                    "Rendering parameters given for layer '%s' which does not "
                    "exist in this metric_map_t object",
                    kv.first.c_str());
            }

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
            if (voxelMap)
            {
                voxelMap->renderingOptions.generateFreeVoxels = p.render_voxelmaps_free_space;
            }
            else if (voxelRGBMap)
            {
                voxelRGBMap->renderingOptions.generateFreeVoxels = p.render_voxelmaps_free_space;
            }

            // regular render method:
            map->getVisualizationInto(o);
            return;
        }
    }
    else
    {
        pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(map);
    }

    if (!pts || (p.colorMode.has_value() && p.colorMode->keep_original_cloud_color))
    {
        // Not convertible to point maps or user selected their original colors,
        // use its own default renderer:
        map->getVisualizationInto(o);

        // apply the point size, if possible:
        if (auto glPtsCol = o.getByClass<mrpt::opengl::CPointCloudColoured>(); glPtsCol)
        {
            glPtsCol->setPointSize(p.pointSize);
        }
        else if (auto glPts = o.getByClass<mrpt::opengl::CPointCloud>(); glPts)
        {
            glPts->setPointSize(p.pointSize);
        }

        return;
    }

    if (pts && pts->empty())
    {
        // quick return if empty point cloud
        return;
    }

    if (p.colorMode.has_value())
    {
        // color point cloud:
        auto glPts = mrpt::opengl::CPointCloudColoured::Create();
        glPts->loadFromPointsMap(pts.get());

        glPts->setPointSize(p.pointSize);

        mrpt::math::TBoundingBoxf bb;

        const bool hasToAutoFindBB =
            (!p.colorMode->colorMapMinCoord.has_value() ||
             !p.colorMode->colorMapMaxCoord.has_value());

        if (hasToAutoFindBB)
        {
            bb = pts->boundingBox();
        }

        ASSERT_(p.colorMode->recolorizeByCoordinate.has_value());

        const unsigned int coordIdx =
            static_cast<unsigned int>(p.colorMode->recolorizeByCoordinate.value());
        ASSERT_(coordIdx < 3);

        float min = bb.min[coordIdx], max = bb.max[coordIdx];

        if (hasToAutoFindBB && p.colorMode->autoBoundingBoxOutliersPercentile.has_value())
        {
            const float confidenceInterval = *p.colorMode->autoBoundingBoxOutliersPercentile;

            // handle planar maps (avoids error in histogram below):
            for (int i = 0; i < 3; i++)
            {
                if (bb.max[i] == bb.min[i])
                {
                    bb.max[i] = bb.min[i] + 0.1f;
                }
            }

            // Use a histogram to discard outliers from the colormap extremes:
            constexpr size_t nBins = 100;
            // for x,y,z
            std::array<mrpt::math::CHistogram, 3> hists = {
                mrpt::math::CHistogram(bb.min.x, bb.max.x, nBins),
                mrpt::math::CHistogram(bb.min.y, bb.max.y, nBins),
                mrpt::math::CHistogram(bb.min.z, bb.max.z, nBins)};

            const auto lambdaVisitPoints = [&hists](const mrpt::math::TPoint3Df& pt)
            {
                for (int i = 0; i < 3; i++)
                {
                    hists[i].add(pt[i]);
                }
            };

            for (size_t i = 0; i < pts->size(); i++)
            {
                mrpt::math::TPoint3Df pt;
                pts->getPoint(i, pt.x, pt.y, pt.z);
                lambdaVisitPoints(pt);
            }

            // Analyze the histograms and get confidence intervals:
            std::vector<double> coords;
            std::vector<double> hits;

            hists[coordIdx].getHistogramNormalized(coords, hits);
            mrpt::math::confidenceIntervalsFromHistogram(
                coords, hits, min, max, confidenceInterval);

        }  // end compute outlier limits

        const float coordMin =
            p.colorMode->colorMapMinCoord.has_value() ? *p.colorMode->colorMapMinCoord : min;

        const float coordMax =
            p.colorMode->colorMapMaxCoord.has_value() ? *p.colorMode->colorMapMaxCoord : max;

        glPts->recolorizeByCoordinate(coordMin, coordMax, coordIdx, p.colorMode->colorMap);

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

bool metric_map_t::empty() const { return layers.empty() && lines.empty() && planes.empty(); }
void metric_map_t::clear() { *this = metric_map_t(); }

// TODO(JLBC): Write unit test for mergeWith()

void metric_map_t::merge_with(
    const metric_map_t& otherPc, const std::optional<mrpt::math::TPose3D>& otherRelativePose)
{
    mrpt::poses::CPose3D pose;
    if (otherRelativePose.has_value())
    {
        pose = mrpt::poses::CPose3D(otherRelativePose.value());
    }

    // Lines:
    if (otherRelativePose.has_value())
    {
        std::transform(
            otherPc.lines.begin(), otherPc.lines.end(), std::back_inserter(lines),
            [&](const mrpt::math::TLine3D& l)
            {
                return mrpt::math::TLine3D::FromPointAndDirector(
                    pose.composePoint(l.pBase), pose.rotateVector(l.getDirectorVector()));
            });
    }
    else
    {
        std::copy(otherPc.lines.begin(), otherPc.lines.end(), std::back_inserter(lines));
    }

    // Planes:
    if (otherRelativePose.has_value())
    {
        std::transform(
            otherPc.planes.begin(), otherPc.planes.end(), std::back_inserter(planes),
            [&](const plane_patch_t& l)
            {
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
        std::copy(otherPc.planes.begin(), otherPc.planes.end(), std::back_inserter(planes));
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
            layers[name] =
                std::dynamic_pointer_cast<mrpt::maps::CMetricMap>(otherMap->duplicateGetSmartPtr());

            if (otherRelativePose.has_value())
            {
                if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layers[name]); pts)
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
            if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layers[name]); pts)
            {
                pts->insertAnotherMap(
                    std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(otherMap).get(), pose);
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
        if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer.second); pts)
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

    if (id)
    {
        ret += "id="s + std::to_string(*id) + " "s;
    }
    if (label)
    {
        ret += "label='"s + *label + "' "s;
    }

    if (georeferencing)
    {
        const auto& gc = georeferencing->geo_coord;
        ret +=
            "georeferenced: "s + "lat="s + gc.lat.getAsString() + " lon="s + gc.lon.getAsString() +
            mrpt::format(" (%.09f  %.09f) ", gc.lat.getDecimalValue(), gc.lon.getDecimalValue()) +
            " h="s + std::to_string(gc.height) + " T_enu_map="s +
            georeferencing->T_enu_to_map.asString();
    }

    if (empty())
    {
        return {ret + "empty"s};
    }

    const auto retAppend = [&ret](const std::string& s)
    {
        if (!ret.empty())
        {
            ret += ", "s;
        }
        ret += s;
    };

    if (!lines.empty())
    {
        retAppend(std::to_string(lines.size()) + " lines"s);
    }
    if (!planes.empty())
    {
        retAppend(std::to_string(planes.size()) + " planes"s);
    }

    size_t nPts = 0, nVoxels = 0;
    bool   otherLayers = false;
    for (const auto& layer : layers)
    {
        ASSERT_(layer.second);
        if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer.second); pts)
        {
            nPts += pts->size();
        }
        else if (auto vxs = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(layer.second); vxs)
        {
            nVoxels += vxs->grid().activeCellsCount();
        }
        else
        {
            otherLayers = true;
        }
    }

    if (nPts != 0 || nVoxels != 0 || otherLayers)
    {
        retAppend(
            mrpt::system::unitsFormat(static_cast<double>(nPts), 2, false) + " points, "s +
            mrpt::system::unitsFormat(static_cast<double>(nVoxels), 2, false) + " voxels in "s +
            std::to_string(layers.size()) + " layers ("s);

        for (const auto& layer : layers)
        {
            ret += "\""s + layer.first + "\":"s + layer.second->asString() + " "s;
        }
        ret += ")";
    }

    if (!metadata.isNullNode())
    {
        std::stringstream                 ss;
        mrpt::containers::YamlEmitOptions eo;
        eo.emitHeader   = false;
        eo.emitComments = false;
        metadata.printAsYAML(ss, eo);
        retAppend("metadata: '"s + ss.str() + "' "s);
    }

    return ret;
}

bool metric_map_t::save_to_file(const std::string& fileName) const
{
    auto f = mrpt::io::CFileGZOutputStream(fileName);
    if (!f.is_open())
    {
        return false;
    }

    auto arch = mrpt::serialization::archiveFrom(f);
    arch << *this;

    return true;
}

bool metric_map_t::load_from_file(const std::string& fileName)
{
    auto f = mrpt::io::CFileGZInputStream(fileName);
    if (!f.is_open())
    {
        return false;
    }

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
    if (!ret)
    {
        ret = std::make_shared<metric_map_t>(*this);
    }
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
    if (!ret)
    {
        ret = std::make_shared<metric_map_t>(*this);
    }
    return ret;
}

mrpt::maps::CPointsMap::Ptr metric_map_t::point_layer(const layer_name_t& name) const
{
    auto it = layers.find(name);
    if (it == layers.end())
    {
        THROW_EXCEPTION_FMT("Layer '%s' does not exist.", name.c_str());
    }

    const auto& ptr = it->second;
    if (!ptr)
    {
        return {};  // empty shared_ptr.
    }

    auto ret = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(ptr);
    if (!ret)
    {
        THROW_EXCEPTION_FMT(
            "Layer '%s' is not a point cloud (actual class:'%s').", name.c_str(),
            ptr->GetRuntimeClass()->className);
    }

    return ret;
}

const mrpt::maps::CPointsMap* mp2p_icp::MapToPointsMap(const mrpt::maps::CMetricMap& map)
{
    if (auto ptsMap = dynamic_cast<const mrpt::maps::CPointsMap*>(&map); ptsMap)
    {
        return ptsMap;
    }
    if (auto voxelMap = dynamic_cast<const mrpt::maps::CVoxelMap*>(&map); voxelMap)
    {
        return voxelMap->getOccupiedVoxels().get();
    }
    if (auto voxelRGBMap = dynamic_cast<const mrpt::maps::CVoxelMapRGB*>(&map); voxelRGBMap)
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
    if (auto voxelRGBMap = dynamic_cast<mrpt::maps::CVoxelMapRGB*>(&map); voxelRGBMap)
    {
        return voxelRGBMap->getOccupiedVoxels().get();
    }
    return {};
}

const mrpt::maps::NearestNeighborsCapable* mp2p_icp::MapToNN(
    const mrpt::maps::CMetricMap& map, bool throwIfNotImplemented)
{
    const auto* ptr = dynamic_cast<const mrpt::maps::NearestNeighborsCapable*>(&map);

    if (ptr)
    {
        return ptr;
    }
    if (!throwIfNotImplemented)
    {
        return nullptr;
    }

    THROW_EXCEPTION_FMT(
        "The map of type '%s' does not implement the expected interface "
        "mrpt::maps::NearestNeighborsCapable",
        map.GetRuntimeClass()->className);
}

const mp2p_icp::NearestPlaneCapable* mp2p_icp::MapToNP(
    const mrpt::maps::CMetricMap& map, bool throwIfNotImplemented)
{
    const auto* ptr = dynamic_cast<const mp2p_icp::NearestPlaneCapable*>(&map);

    if (ptr)
    {
        return ptr;
    }
    if (!throwIfNotImplemented)
    {
        return nullptr;
    }

    THROW_EXCEPTION_FMT(
        "The map of type '%s' does not implement the expected interface "
        "mp2p_icp::NearestPlaneCapable",
        map.GetRuntimeClass()->className);
}

// Serialization of geo-reference information:
constexpr const char* GEOREF_MAGIC_STR = "mp2p_icp::Georeferencing";

mrpt::serialization::CArchive& mp2p_icp::operator>>(
    mrpt::serialization::CArchive& in, std::optional<metric_map_t::Georeferencing>& g)
{
    std::string georef_stream_signature;
    in >> georef_stream_signature;
    ASSERT_EQUAL_(georef_stream_signature, std::string(GEOREF_MAGIC_STR));
    const uint8_t version = in.ReadAs<uint8_t>();
    switch (version)
    {
        case 0:
            if (in.ReadAs<bool>())
            {
                auto& gg = g.emplace();
                in >> gg.geo_coord.lat.decimal_value >> gg.geo_coord.lon.decimal_value >>
                    gg.geo_coord.height;
                in >> gg.T_enu_to_map;
            }
            break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };

    return in;
}

mrpt::serialization::CArchive& mp2p_icp::operator<<(
    mrpt::serialization::CArchive& out, const std::optional<metric_map_t::Georeferencing>& g)
{
    out << std::string(GEOREF_MAGIC_STR);
    constexpr uint8_t serial_version = 0;
    out.WriteAs<uint8_t>(serial_version);

    out.WriteAs<bool>(g.has_value());
    if (g)
    {
        out << g->geo_coord.lat.decimal_value << g->geo_coord.lon.decimal_value
            << g->geo_coord.height;
        out << g->T_enu_to_map;
    }
    return out;
}

std::optional<metric_map_t::Georeferencing> mp2p_icp::FromYAML(
    const mrpt::containers::yaml& yaml_data)
{
    ASSERT_(yaml_data.isMap());
    ASSERT_(yaml_data.has("type"));
    ASSERT_(yaml_data.has("defined"));

    ASSERT_EQUAL_(yaml_data["type"].as<std::string>(), GEOREF_MAGIC_STR);
    const bool defined = yaml_data["defined"].as<bool>();
    if (!defined)
    {  // empty:
        return {};
    }

    std::optional<metric_map_t::Georeferencing> georef;

    auto& g                       = georef.emplace();
    g.geo_coord.lon.decimal_value = yaml_data["geo_coord"]["lon"].as<double>();
    g.geo_coord.lat.decimal_value = yaml_data["geo_coord"]["lat"].as<double>();
    g.geo_coord.height            = yaml_data["geo_coord"]["altitude"].as<double>();

    const auto& ym = yaml_data["T_enu_to_map"]["mean"];
    g.T_enu_to_map.mean.x(ym["x"].as<double>());
    g.T_enu_to_map.mean.y(ym["y"].as<double>());
    g.T_enu_to_map.mean.z(ym["z"].as<double>());

    yaml_data["T_enu_to_map"]["cov"].toMatrix(g.T_enu_to_map.cov);

    return georef;
}

mrpt::containers::yaml mp2p_icp::ToYAML(const std::optional<metric_map_t::Georeferencing>& gref)
{
    mrpt::containers::yaml data = mrpt::containers::yaml::Map();

    data["type"]    = GEOREF_MAGIC_STR;
    data["defined"] = gref.has_value();
    if (gref)
    {
        {
            mrpt::containers::yaml gcoord = mrpt::containers::yaml::Map();
            gcoord["lon"]                 = gref->geo_coord.lon;
            gcoord["lat"]                 = gref->geo_coord.lat;
            gcoord["altitude"]            = gref->geo_coord.height;

            data["geo_coord"] = gcoord;
        }

        mrpt::containers::yaml pose_mean = mrpt::containers::yaml::Map();

        pose_mean["x"] = gref->T_enu_to_map.mean.x();
        pose_mean["y"] = gref->T_enu_to_map.mean.y();
        pose_mean["z"] = gref->T_enu_to_map.mean.z();

        mrpt::containers::yaml pose = mrpt::containers::yaml::Map();
        pose["mean"]                = pose_mean;
        pose["cov"]                 = mrpt::containers::yaml::FromMatrix(gref->T_enu_to_map.cov);
        data["T_enu_to_map"]        = pose;
    }
    return data;
}
