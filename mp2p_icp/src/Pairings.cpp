/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Pairings.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/Pairings.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#include <iterator>  // std::make_move_iterator

using namespace mp2p_icp;

static const uint8_t SERIALIZATION_VERSION = 0;

void Pairings::serializeTo(mrpt::serialization::CArchive& out) const
{
    out.WriteAs<uint8_t>(SERIALIZATION_VERSION);
    // out << paired_pt2pt;
    out << paired_pt2ln << paired_pt2pl << paired_ln2ln << paired_pl2pl
        << point_weights;
}

void Pairings::serializeFrom(mrpt::serialization::CArchive& in)
{
    const auto readVersion = in.ReadAs<uint8_t>();

    ASSERT_EQUAL_(readVersion, SERIALIZATION_VERSION);
    in >> paired_pt2pt;
    in >> paired_pt2ln >> paired_pt2pl >> paired_ln2ln >> paired_pl2pl >>
        point_weights;
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

std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D>
    mp2p_icp::eval_centroids_robust(
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
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);
    {
        std::size_t cnt             = 0;
        auto        it_next_outlier = outliers.point2point.begin();
        for (std::size_t i = 0; i < in.paired_pt2pt.size(); i++)
        {
            // Skip outlier?
            if (it_next_outlier != outliers.point2point.end() &&
                i == *it_next_outlier)
            {
                ++it_next_outlier;
                continue;
            }
            const auto& pair = in.paired_pt2pt[i];

            ct_this += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
            cnt++;
        }
        // Sanity check:
        ASSERT_EQUAL_(cnt, nPt2Pt - outliers.point2point.size());

        ct_other *= wcPoints;
        ct_this *= wcPoints;
    }

    return {ct_other, ct_this};
}

template <typename T>
static void push_back_copy(const T& o, T& me)
{
    me.insert(me.end(), o.begin(), o.end());
}
template <typename T>
static void push_back_move(T&& o, T& me)
{
    me.insert(
        me.end(), std::make_move_iterator(o.begin()),
        std::make_move_iterator(o.end()));
}

void Pairings::push_back(const Pairings& o)
{
    push_back_copy(o.paired_pt2pt, paired_pt2pt);
    push_back_copy(o.paired_pt2ln, paired_pt2ln);
    push_back_copy(o.paired_pt2pl, paired_pt2pl);
    push_back_copy(o.paired_ln2ln, paired_ln2ln);
    push_back_copy(o.paired_pl2pl, paired_pl2pl);
}

void Pairings::push_back(Pairings&& o)
{
    push_back_move(std::move(o.paired_pt2pt), paired_pt2pt);
    push_back_move(std::move(o.paired_pt2ln), paired_pt2ln);
    push_back_move(std::move(o.paired_pt2pl), paired_pt2pl);
    push_back_move(std::move(o.paired_ln2ln), paired_ln2ln);
    push_back_move(std::move(o.paired_pl2pl), paired_pl2pl);
}

size_t Pairings::size() const
{
    return paired_pt2pt.size() + paired_pt2ln.size() + paired_pt2pl.size() +
           paired_ln2ln.size() + paired_pl2pl.size();
}

template <typename CONTAINER>
void append_container_size(
    const CONTAINER& c, const std::string& name, std::string& ret)
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

    return ret;
}

namespace mrpt::serialization
{
CArchive& operator<<(CArchive& out, const mp2p_icp::point_line_pair_t& obj)
{
    out.WriteAs<uint8_t>(0);

    out << obj.ln_this << obj.pt_other;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::point_line_pair_t& obj)
{
    // const auto ver =
    in.ReadAs<uint8_t>();

    in >> obj.ln_this >> obj.pt_other;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::point_plane_pair_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.pl_this.centroid << obj.pl_this.plane << obj.pt_other;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::point_plane_pair_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.pl_this.centroid >> obj.pl_this.plane >> obj.pt_other;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_line_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.ln_other << obj.ln_this;
    return out;
}

CArchive& operator>>(CArchive& in, mp2p_icp::matched_line_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.ln_other >> obj.ln_this;
    return in;
}

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_plane_t& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.p_other.centroid << obj.p_other.plane;
    out << obj.p_this.centroid << obj.p_this.plane;
    return out;
}
CArchive& operator>>(CArchive& in, mp2p_icp::matched_plane_t& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.p_other.centroid >> obj.p_other.plane;
    in >> obj.p_this.centroid >> obj.p_this.plane;
    return in;
}

}  // namespace mrpt::serialization
