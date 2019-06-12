/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pointcloud.cpp
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/pointcloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(
    pointcloud_t, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t pointcloud_t::serializeGetVersion() const { return 0; }
void    pointcloud_t::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << lines;

    out.WriteAs<uint32_t>(planes.size());
    for (const auto& p : planes) out << p.plane << p.centroid;

    out.WriteAs<uint32_t>(lines.size());
    for (const auto& l : lines) out << l;

    out.WriteAs<uint32_t>(point_layers.size());
    for (const auto& l : point_layers) out << l.first << *l.second.get();
}
void pointcloud_t::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
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
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

/** Gets a renderizable view of all planes */
void pointcloud_t::planesAsRenderizable(
    mrpt::opengl::CSetOfObjects& o, const render_params_t& p)
{
    MRPT_START

    const float pw = p.plane_half_width, pf = p.plane_grid_spacing;

    for (const auto& plane : planes)
    {
        auto gl_pl =
            mrpt::opengl::CGridPlaneXY::Create(-pw, pw, -pw, pw, .0, pf);
        gl_pl->setColor_u8(p.plane_color);
        mrpt::math::TPose3D planePose;
        plane.plane.getAsPose3DForcingOrigin(plane.centroid, planePose);
        gl_pl->setPose(planePose);
        o.insert(gl_pl);
    }

    MRPT_END
}

bool pointcloud_t::empty() const
{
    return point_layers.empty() && lines.empty() && planes.empty();
}
void pointcloud_t::clear() { *this = pointcloud_t(); }
