/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Pairings.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/plane_patch.h>
#include <mp2p_icp/point_plane_pair_t.h>
#include <mp2p_icp/render_params.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/TTypeName.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** Plane-to-plane pair */
struct matched_plane_t
{
    plane_patch_t p_global, p_local;

    matched_plane_t() = default;
    matched_plane_t(const plane_patch_t& pl_this, const plane_patch_t& pl_other)
        : p_global(pl_this), p_local(pl_other)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::matched_plane_t)
};
using MatchedPlaneList = std::vector<matched_plane_t>;

/** Line-to-line pair */
struct matched_line_t
{
    mrpt::math::TLine3D ln_global, ln_local;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::matched_line_t)
};
using MatchedLineList = std::vector<matched_line_t>;

/** Point-to-line pair */
struct point_line_pair_t
{
    mrpt::math::TLine3D  ln_global;
    mrpt::math::TPoint3D pt_local;

    point_line_pair_t() = default;
    point_line_pair_t(
        const mrpt::math::TLine3D&  l_global,
        const mrpt::math::TPoint3D& p_local)
        : ln_global(l_global), pt_local(p_local)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::point_line_pair_t)
};

using MatchedPointLineList = std::vector<point_line_pair_t>;

/** Common pairing input data for OLAE, Horn's, and other solvers.
 * Planes and lines must have unit director and normal vectors, respectively.
 *
 * Pairings are between a "global" (or "this") and a "local" (or "other")
 * pointcloud, while we are searching for the relative pose of "local" wrt
 * "global", such that "relative_pose \oplus localPoint = globalPoint".
 */
struct Pairings
{
    Pairings() = default;
    virtual ~Pairings();

    /** @name Data fields
     * @{ */

    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_pt2pt;
    MatchedPointLineList           paired_pt2ln;
    MatchedPointPlaneList          paired_pt2pl;
    MatchedLineList                paired_ln2ln;
    MatchedPlaneList               paired_pl2pl;

    /// Each Matcher will add pairings in the fields above, and will increment
    /// this `potential_pairings` with the maximum number of potential pairings
    /// that it might have found. That is, the ratio of successful pairings
    /// is `this->size() / double(potential_pairings)`.
    uint64_t potential_pairings = 0;

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

    /** Gets a renderizable view of all geometric entities.
     *
     * See render_params_t for options to show/hide the different geometric
     * entities and point layers.
     *
     * \note If deriving user classes inheriting from metric_map_t, remember to
     *  reimplement this method and call this base class method to render
     *  common elements.
     */
    virtual auto get_visualization(
        const mrpt::poses::CPose3D&     localWrtGlobal,
        const pairings_render_params_t& p = pairings_render_params_t()) const
        -> std::shared_ptr<mrpt::opengl::CSetOfObjects>;

    /** Used inside get_visualization(), renders pt-to-pt pairings only. */
    virtual void get_visualization_pt2pt(
        mrpt::opengl::CSetOfObjects&          o,
        const mrpt::poses::CPose3D&           localWrtGlobal,
        const render_params_pairings_pt2pt_t& p) const;

    /** Used inside get_visualization(), renders pt-to-pl pairings only. */
    virtual void get_visualization_pt2pl(
        mrpt::opengl::CSetOfObjects&          o,
        const mrpt::poses::CPose3D&           localWrtGlobal,
        const render_params_pairings_pt2pl_t& p) const;

    /** Used inside get_visualization(), renders pt-to-ln pairings only. */
    virtual void get_visualization_pt2ln(
        mrpt::opengl::CSetOfObjects&          o,
        const mrpt::poses::CPose3D&           localWrtGlobal,
        const render_params_pairings_pt2ln_t& p) const;

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

/** Evaluates the centroids [ct_local, ct_global] for point-to-point
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
