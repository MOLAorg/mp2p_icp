/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pointcloud.cpp
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/pointcloud.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>

#include <algorithm>
#include <iterator>

IMPLEMENTS_MRPT_OBJECT(
    pointcloud_t, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t pointcloud_t::serializeGetVersion() const { return 1; }
void    pointcloud_t::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << lines;

    out.WriteAs<uint32_t>(planes.size());
    for (const auto& p : planes) out << p.plane << p.centroid;

    out.WriteAs<uint32_t>(lines.size());
    for (const auto& l : lines) out << l;

    out.WriteAs<uint32_t>(point_layers.size());
    for (const auto& l : point_layers) out << l.first << *l.second.get();

    out << id << label;  // new in v1

    // Optional user data:
    derivedSerializeTo(out);
}
void pointcloud_t::serializeFrom(
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
            point_layers.clear();
            for (std::size_t i = 0; i < nPts; i++)
            {
                std::string name;
                in >> name;
                point_layers[name] =
                    mrpt::ptr_cast<mrpt::maps::CPointsMap>::from(
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

/** Gets a renderizable view of all planes */
auto pointcloud_t::get_visualization(const render_params_t& p) const
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

void pointcloud_t::get_visualization_planes(
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

void pointcloud_t::get_visualization_lines(
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

void pointcloud_t::get_visualization_points(
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
            const auto itPts = point_layers.find(kv.first);
            if (itPts == point_layers.end())
                THROW_EXCEPTION_FMT(
                    "Rendering parameters given for layer '%s' which does not "
                    "exist in this pointcloud_t object",
                    kv.first.c_str());

            get_visualization_point_layer(o, kv.second, itPts->second);
        }
    }
    else
    {
        // render all layers with the same params:
        for (const auto& kv : point_layers)
            get_visualization_point_layer(o, p.allLayers, kv.second);
    }

    MRPT_END
}

void pointcloud_t::get_visualization_point_layer(
    mrpt::opengl::CSetOfObjects& o, const render_params_point_layer_t& p,
    const mrpt::maps::CPointsMap::Ptr& pts)
{
    ASSERT_(pts);
    if (pts->empty()) return;

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

bool pointcloud_t::empty() const
{
    return point_layers.empty() && lines.empty() && planes.empty();
}
void pointcloud_t::clear() { *this = pointcloud_t(); }

MRPT_TODO("Write unit test for mergeWith()")
void pointcloud_t::merge_with(
    const pointcloud_t&                       otherPc,
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
                MRPT_TODO("Finish rotating planes");
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
    for (const auto& layer : otherPc.point_layers)
    {
        const auto& name     = layer.first;
        const auto& otherPts = layer.second;

        // New layer?
        if (point_layers.count(name) == 0)
        {
            // Make a copy and transform (if applicable):
            point_layers[name] =
                std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                    otherPts->duplicateGetSmartPtr());

            if (otherRelativePose.has_value())
                point_layers[name]->changeCoordinatesReference(pose);
        }
        else
        {
            // merge with existing layer:
            point_layers[name]->insertAnotherMap(otherPts.get(), pose);
        }
    }
}

size_t pointcloud_t::size() const
{
    size_t n = 0;

    n += lines.size();
    n += planes.size();
    for (const auto& layer : point_layers) n += layer.second->size();

    return n;
}

std::string pointcloud_t::contents_summary() const
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

    size_t nPts = 0;
    for (const auto& layer : point_layers)
    {
        ASSERT_(layer.second);
        nPts += layer.second->size();
    }

    if (nPts != 0)
    {
        retAppend(
            std::to_string(nPts) + " points in "s +
            std::to_string(point_layers.size()) + " layers ("s);

        for (const auto& layer : point_layers)
            ret += "\""s + layer.first + "\":"s +
                   std::to_string(layer.second->size()) + " "s;
        ret += ")";
    }

    return ret;
}

bool pointcloud_t::save_to_file(const std::string& fileName) const
{
    auto f = mrpt::io::CFileGZOutputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch << *this;

    return true;
}

bool pointcloud_t::load_from_file(const std::string& fileName)
{
    auto f = mrpt::io::CFileGZInputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch >> *this;

    return true;
}
