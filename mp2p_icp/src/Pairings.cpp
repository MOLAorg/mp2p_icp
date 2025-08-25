/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Pairings.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/Pairings.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#include <iterator>  // std::make_move_iterator

using namespace mp2p_icp;

static const uint8_t SERIALIZATION_VERSION = 1;

Pairings::~Pairings() = default;

void Pairings::serializeTo(mrpt::serialization::CArchive& out) const
{
    out.WriteAs<uint8_t>(SERIALIZATION_VERSION);
    out << paired_pt2pt;
    out << paired_pt2ln << paired_pt2pl << paired_ln2ln << paired_pl2pl << point_weights;
    out << potential_pairings;  // v1
}

void Pairings::serializeFrom(mrpt::serialization::CArchive& in)
{
    const auto readVersion = in.ReadAs<uint8_t>();

    ASSERT_LE_(readVersion, SERIALIZATION_VERSION);
    in >> paired_pt2pt;
    in >> paired_pt2ln >> paired_pt2pl >> paired_ln2ln >> paired_pl2pl >> point_weights;
    if (readVersion >= 1) in >> potential_pairings;
}

mrpt::serialization::CArchive& mp2p_icp::operator<<(
    mrpt::serialization::CArchive& out, const Pairings& obj)
{
    obj.serializeTo(out);
    return out;
}

mrpt::serialization::CArchive& mp2p_icp::operator>>(
    mrpt::serialization::CArchive& in, Pairings& obj)
{
    obj.serializeFrom(in);
    return in;
}

std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D> mp2p_icp::eval_centroids_robust(
    const Pairings& in, const OutlierIndices& outliers)
{
    using mrpt::math::TPoint3D;

    const auto nPt2Pt = in.paired_pt2pt.size();

    // We need more points than outliers (!)
    ASSERT_GT_(nPt2Pt, outliers.point2point.size());

    // Normalized weights for centroids.
    // Discount outliers.
    const double wcPoints = 1.0 / (nPt2Pt - outliers.point2point.size());

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    TPoint3D ct_local(0, 0, 0), ct_global(0, 0, 0);
    {
        std::size_t cnt             = 0;
        auto        it_next_outlier = outliers.point2point.begin();
        for (std::size_t i = 0; i < in.paired_pt2pt.size(); i++)
        {
            // Skip outlier?
            if (it_next_outlier != outliers.point2point.end() && i == *it_next_outlier)
            {
                ++it_next_outlier;
                continue;
            }
            const auto& pair = in.paired_pt2pt[i];

            ct_global += pair.global;
            ct_local += pair.local;
            cnt++;
        }
        // Sanity check:
        ASSERT_EQUAL_(cnt, nPt2Pt - outliers.point2point.size());

        ct_local *= wcPoints;
        ct_global *= wcPoints;
    }

    return {ct_local, ct_global};
}

template <typename T>
static void push_back_copy(const T& o, T& me)
{
    me.insert(me.end(), o.begin(), o.end());
}
template <typename T>
static void push_back_move(T&& o, T& me)
{
    me.insert(me.end(), std::make_move_iterator(o.begin()), std::make_move_iterator(o.end()));
}

void Pairings::push_back(const Pairings& o)
{
    push_back_copy(o.paired_pt2pt, paired_pt2pt);
    push_back_copy(o.paired_pt2ln, paired_pt2ln);
    push_back_copy(o.paired_pt2pl, paired_pt2pl);
    push_back_copy(o.paired_ln2ln, paired_ln2ln);
    push_back_copy(o.paired_pl2pl, paired_pl2pl);
    potential_pairings += o.potential_pairings;
}

void Pairings::push_back(Pairings&& o)
{
    push_back_move(std::move(o.paired_pt2pt), paired_pt2pt);
    push_back_move(std::move(o.paired_pt2ln), paired_pt2ln);
    push_back_move(std::move(o.paired_pt2pl), paired_pt2pl);
    push_back_move(std::move(o.paired_ln2ln), paired_ln2ln);
    push_back_move(std::move(o.paired_pl2pl), paired_pl2pl);
    potential_pairings = o.potential_pairings;
}

size_t Pairings::size() const
{
    return paired_pt2pt.size() + paired_pt2ln.size() + paired_pt2pl.size() + paired_ln2ln.size() +
           paired_pl2pl.size();
}

template <typename CONTAINER>
void append_container_size(const CONTAINER& c, const std::string& name, std::string& ret)
{
    using namespace std::string_literals;

    if (c.empty()) return;
    if (!ret.empty()) ret += ", "s;
    ret += std::to_string(c.size()) + " "s + name;
}

std::string Pairings::contents_summary() const
{
    using namespace std::string_literals;

    if (empty()) return {"none"s};

    std::string ret;
    append_container_size(paired_pt2pt, "point-point", ret);
    append_container_size(paired_pt2ln, "point-line", ret);
    append_container_size(paired_pt2pl, "point-plane", ret);
    append_container_size(paired_ln2ln, "line-line", ret);
    append_container_size(paired_pl2pl, "plane-plane", ret);
    ret += " out of "s + std::to_string(potential_pairings);

    return ret;
}

auto Pairings::get_visualization(
    const mrpt::poses::CPose3D& localWrtGlobal, const pairings_render_params_t& p) const
    -> std::shared_ptr<mrpt::opengl::CSetOfObjects>
{
    MRPT_START
    auto o = mrpt::opengl::CSetOfObjects::Create();

    get_visualization_pt2pt(*o, localWrtGlobal, p.pt2pt);
    get_visualization_pt2pl(*o, localWrtGlobal, p.pt2pl);
    get_visualization_pt2ln(*o, localWrtGlobal, p.pt2ln);

    return o;
    MRPT_END
}

void Pairings::get_visualization_pt2pt(
    mrpt::opengl::CSetOfObjects& o, const mrpt::poses::CPose3D& localWrtGlobal,
    const render_params_pairings_pt2pt_t& p) const
{
    if (!p.visible) return;

    auto lns = mrpt::opengl::CSetOfLines::Create();
    lns->setColor_u8(p.color);

    // this: global, other: local
    for (const auto& pair : paired_pt2pt)
    {
        const auto ptLocalTf = localWrtGlobal.composePoint(pair.local);
        lns->appendLine(ptLocalTf, pair.global);
    }

    o.insert(lns);
}

void Pairings::get_visualization_pt2pl(
    mrpt::opengl::CSetOfObjects& o, const mrpt::poses::CPose3D& localWrtGlobal,
    const render_params_pairings_pt2pl_t& p) const
{
    if (!p.visible) return;

    auto lns = mrpt::opengl::CSetOfLines::Create();
    lns->setColor_u8(p.segmentColor);

    const double L = 0.5 * p.planePatchSize;

    for (const auto& pair : paired_pt2pl)
    {
        const auto globalPlanePose = mrpt::poses::CPose3D(
            pair.pl_global.plane.getAsPose3DForcingOrigin(pair.pl_global.centroid));

        const auto ptLocal   = pair.pt_local;
        const auto ptLocalTf = localWrtGlobal.composePoint(ptLocal);

        // line segment:
        lns->appendLine(ptLocalTf, globalPlanePose.translation());

        // plane patch:
        auto glPlane = mrpt::opengl::CTexturedPlane::Create();
        glPlane->setPlaneCorners(-L, L, -L, L);
        glPlane->setColor_u8(p.planePatchColor);

        glPlane->setPose(globalPlanePose);

        o.insert(glPlane);
    }

    o.insert(lns);
}

void Pairings::get_visualization_pt2ln(
    mrpt::opengl::CSetOfObjects& o, const mrpt::poses::CPose3D& localWrtGlobal,
    const render_params_pairings_pt2ln_t& p) const
{
    if (!p.visible) return;

    auto pairingLines = mrpt::opengl::CSetOfLines::Create();
    pairingLines->setColor_u8(p.segmentColor);

    auto globalLines = mrpt::opengl::CSetOfLines::Create();
    globalLines->setColor_u8(p.lineColor);

    const double L = 0.5 * p.lineLength;

    for (const auto& pair : paired_pt2ln)
    {
        const auto& globalLine = pair.ln_global;

        const auto ptLocal   = pair.pt_local;
        const auto ptLocalTf = localWrtGlobal.composePoint(ptLocal);

        // line segment:
        pairingLines->appendLine(ptLocalTf, globalLine.pBase);

        // line segment:
        globalLines->appendLine(
            globalLine.pBase - globalLine.director * L, globalLine.pBase + globalLine.director * L);
    }

    o.insert(pairingLines);
    o.insert(globalLines);
}

namespace mrpt::serialization
{
CArchive& operator<<(CArchive& out, const mp2p_icp::point_line_pair_t& obj)
{
    out.WriteAs<uint8_t>(0);

    out << obj.ln_global << obj.pt_local;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::point_line_pair_t& obj)
{
    // const auto ver =
    in.ReadAs<uint8_t>();

    in >> obj.ln_global >> obj.pt_local;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::point_plane_pair_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.pl_global.centroid << obj.pl_global.plane << obj.pt_local;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::point_plane_pair_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.pl_global.centroid >> obj.pl_global.plane >> obj.pt_local;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_line_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.ln_local << obj.ln_global;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::matched_line_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.ln_local >> obj.ln_global;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_plane_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.p_local.centroid << obj.p_local.plane;
    out << obj.p_global.centroid << obj.p_global.plane;
    return out;
}
CArchive& operator>>(CArchive& in, mp2p_icp::matched_plane_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.p_local.centroid >> obj.p_local.plane;
    in >> obj.p_global.centroid >> obj.p_global.plane;
    return in;
}

}  // namespace mrpt::serialization
