/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pointcloud.h
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mp2p_icp/layer_name_t.h>
#include <mp2p_icp/render_params.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CSerializable.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_grp
 * @{
 */

struct plane_patch_t
{
    mrpt::math::TPlane   plane;
    mrpt::math::TPoint3D centroid;

    plane_patch_t() = default;
    plane_patch_t(
        const mrpt::math::TPlane3D& pl, const mrpt::math::TPoint3D& center)
        : plane(pl), centroid(center)
    {
    }
};

/**
 * @brief Generic container of pointcloud(s), and/or extracted features.
 *
 * Can be derived by users to define custom point cloud features, for use
 * in custom alignment algorithms.
 *
 * The class supports C++11/C++17 std::shared_from_this() via
 * get_shared_from_this();
 *
 */
class pointcloud_t : public mrpt::serialization::CSerializable,
                     public std::enable_shared_from_this<pointcloud_t>
{
    DEFINE_SERIALIZABLE(pointcloud_t, mp2p_icp)

   public:
    /** @name Reserved point-cloud layer names (for use in `point_layers`)
     * @{ */

    constexpr static const char* PT_LAYER_RAW             = "raw";
    constexpr static const char* PT_LAYER_PLANE_CENTROIDS = "plane_centroids";

    /** @} */

    /** @name Data fields
     * @{ */

    /** Different point layers, indexed by a descriptive name.
     * Known layer names: See section above.
     * - PT_LAYER_RAW: reserved to the original, full point cloud (if kept)
     * - PT_LAYER_PLANE_CENTROIDS: a point for each plane in `planes` (same
     * order).
     */
    std::map<layer_name_t, mrpt::maps::CPointsMap::Ptr> point_layers;

    /** 3D lines (infinite lines, not segments) */
    std::vector<mrpt::math::TLine3D> lines;

    /** Plane patches=centroid point + infinite plane */
    std::vector<plane_patch_t> planes;

    /** An optional numerical ID to identify the pointcloud in some higher-level
     * system. Used to build the names of ICP debug files, if so requested.
     * It is not mandatory and even duplicates may exist without problems: just
     * a placeholder for the user of this library to use it.
     */
    std::optional<uint64_t> id;

    /** An optional textual identification/description of the pointcloud in some
     * higher-level system. Used to build the names of ICP debug files, if so
     * requested.
     * It is not mandatory and even duplicates may exist without problems: just
     * a placeholder for the user of this library to use it.
     */
    std::optional<std::string> label;

    /** @} */

    /** @name Methods
     * @{ */

    /** return true if all point cloud layers, feature lists, etc. are empty */
    virtual bool empty() const;

    /** Overall number of elements (points, lines, planes) */
    virtual size_t size() const;

    /** Returns a string summarizing all the elements in the container (points,
     * lines, planes) */
    virtual std::string contents_summary() const;

    /** clear all containers  */
    virtual void clear();

    /** Saves the pointcloud_t object  to file, using MRPT serialization and
     *  using on-the-fly GZIP compression.
     * \return true on success.
     */
    bool save_to_file(const std::string& fileName) const;

    /** Loads the pointcloud_t object from a file. See \save_to_file()
     * \return true on success.
     */
    bool load_from_file(const std::string& fileName);

    /** Gets a renderizable view of all geometric entities.
     *
     * See render_params_t for options to show/hide the different geometric
     * entities and point layers.
     *
     * \note If deriving user classes inheriting from pointcloud_t, remember to
     *  reimplement this method and call this base class method to render
     *  common elements.
     */
    virtual auto get_visualization(const render_params_t& p = render_params_t())
        const -> std::shared_ptr<mrpt::opengl::CSetOfObjects>;

    /** Merges all geometric entities from another point cloud into this one,
     * with an optional relative pose transformation.
     *
     * \note Point layers will be merged for coinciding names, or created if the
     * layer did not exist in `this`.
     * \note This method is virtual for user-extended point clouds can handle
     * other geometric primitives as needed.
     */
    virtual void merge_with(
        const pointcloud_t&                       otherPc,
        const std::optional<mrpt::math::TPose3D>& otherRelativePose =
            std::nullopt);

    /** Used inside get_visualization(), renders planes only. */
    void get_visualization_planes(
        mrpt::opengl::CSetOfObjects& o, const render_params_planes_t& p) const;

    /** Used inside get_visualization(), renders lines only. */
    void get_visualization_lines(
        mrpt::opengl::CSetOfObjects& o, const render_params_lines_t& p) const;

    /** Used inside get_visualization(), renders points only. */
    void get_visualization_points(
        mrpt::opengl::CSetOfObjects& o, const render_params_points_t& p) const;

    /** Used inside get_visualization_points(), renders points only. */
    static void get_visualization_point_layer(
        mrpt::opengl::CSetOfObjects& o, const render_params_point_layer_t& p,
        const mrpt::maps::CPointsMap::Ptr& pts);

    /** Returns a shared_ptr to this object, if it was already created initially
     * as a shared_ptr, or an empty pointer otherwise.
     */
    Ptr get_shared_from_this();

    /** Like get_shared_from_this(), or makes a deep copy if the original object
     * was allocated in the stack, etc.
     */
    Ptr get_shared_from_this_or_clone();

    // const versions:
    ConstPtr get_shared_from_this() const;
    ConstPtr get_shared_from_this_or_clone() const;

    /** @} */

   protected:
    /** Implement in derived classes if new data fields are required */
    virtual void derivedSerializeTo([
        [maybe_unused]] mrpt::serialization::CArchive& out) const
    {
    }

    /** Implement in derived classes if new data fields are required */
    virtual void derivedSerializeFrom([
        [maybe_unused]] mrpt::serialization::CArchive& in)
    {
    }
};

/** @} */

}  // namespace mp2p_icp
