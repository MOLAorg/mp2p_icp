/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Pairings.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/pointcloud.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TTypeName.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct matched_plane_t
{
    /** \note "this"=global, "other"=local, while finding the transformation
     * local wrt global
     */
    plane_patch_t p_this, p_other;

    matched_plane_t() = default;
    matched_plane_t(const plane_patch_t& pl_this, const plane_patch_t& pl_other)
        : p_this(pl_this), p_other(pl_other)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::matched_plane_t)
};
using TMatchedPlaneList = std::vector<matched_plane_t>;

struct matched_line_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    mrpt::math::TLine3D ln_this, ln_other;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::matched_line_t)
};
using TMatchedLineList = std::vector<matched_line_t>;

struct point_plane_pair_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    plane_patch_t         pl_this;
    mrpt::math::TPoint3Df pt_other;

    point_plane_pair_t() = default;
    point_plane_pair_t(
        const plane_patch_t& p_this, const mrpt::math::TPoint3Df& p_other)
        : pl_this(p_this), pt_other(p_other)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::point_plane_pair_t)
};
using TMatchedPointPlaneList = std::vector<point_plane_pair_t>;

struct point_line_pair_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    mrpt::math::TLine3D  ln_this;
    mrpt::math::TPoint3D pt_other;

    point_line_pair_t() = default;
    point_line_pair_t(
        const mrpt::math::TLine3D& l_this, const mrpt::math::TPoint3D& p_other)
        : ln_this(l_this), pt_other(p_other)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::point_line_pair_t)
};

using TMatchedPointLineList = std::vector<point_line_pair_t>;

/** Common pairing input data for OLAE, Horn's, and other solvers.
 * Planes and lines must have unit director and normal vectors, respectively.
 *
 * Pairings are between a "global" (or "this") and a "local" (or "other")
 * pointcloud, while we are searching for the relative pose of "local" wrt
 * "global", such that "relative_pose \oplus localPoint = globalPoint".
 */
struct Pairings
{
    /** @name Data fields
     * @{ */

    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_pt2pt;
    TMatchedPointLineList          paired_pt2ln;
    TMatchedPointPlaneList         paired_pt2pl;
    TMatchedLineList               paired_ln2ln;
    TMatchedPlaneList              paired_pl2pl;

    /** *Individual* weights for paired_pt2pt: each entry specifies how many
     * points have the given (mapped second value) weight, in the same order as
     * stored in paired_pt2pt. If empty, all points will have equal weights.
     */
    std::vector<std::pair<std::size_t, double>> point_weights;

    /** @} */

    /** @name Methods
     * @{ */

    virtual bool empty() const
    {
        return paired_pt2pt.empty() && paired_pl2pl.empty() &&
               paired_ln2ln.empty() && paired_pt2ln.empty() &&
               paired_pt2pl.empty();
    }

    /** Overall number of element-to-element pairings (points, lines, planes) */
    virtual size_t size() const;

    /** Returns a string summarizing all the paired elements */
    virtual std::string contents_summary() const;

    /** Copy and append pairings from another container. */
    virtual void push_back(const Pairings& o);

    /** Move pairings from another container. */
    virtual void push_back(Pairings&& o);

    virtual void serializeTo(mrpt::serialization::CArchive& out) const;
    virtual void serializeFrom(mrpt::serialization::CArchive& in);

    /** @} */
    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::Pairings)
};

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const Pairings& obj);

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, Pairings& obj);

/** Vector of pairings that are considered outliers, from those in the
 * corresponding `Pairings` structure.
 *
 * \note Indices are always assumed to be sorted in these containers.
 */
struct OutlierIndices
{
    std::vector<std::size_t> point2point;
    std::vector<std::size_t> line2line;
    std::vector<std::size_t> plane2plane;

    inline bool empty() const
    {
        return point2point.empty() && line2line.empty() && plane2plane.empty();
    }
    inline std::size_t size() const
    {
        return point2point.size() + line2line.size() + plane2plane.size();
    }
};

/** Evaluates the centroids [ct_other, ct_this] for point-to-point
 * correspondences only, taking into account the current guess for outliers
 */
std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D> eval_centroids_robust(
    const Pairings& in, const OutlierIndices& outliers);

/** @} */

}  // namespace mp2p_icp

namespace mrpt::serialization
{
CArchive& operator<<(CArchive& out, const mp2p_icp::point_line_pair_t& obj);
CArchive& operator>>(CArchive& in, mp2p_icp::point_line_pair_t& obj);

CArchive& operator<<(CArchive& out, const mp2p_icp::point_plane_pair_t& obj);
CArchive& operator>>(CArchive& in, mp2p_icp::point_plane_pair_t& obj);

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_line_t& obj);
CArchive& operator>>(CArchive& in, mp2p_icp::matched_line_t& obj);

CArchive& operator<<(CArchive& out, const mp2p_icp::matched_plane_t& obj);
CArchive& operator>>(CArchive& in, mp2p_icp::matched_plane_t& obj);

}  // namespace mrpt::serialization
