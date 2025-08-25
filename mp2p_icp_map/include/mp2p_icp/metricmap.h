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
 * @file   metricmap.h
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mp2p_icp/NearestPlaneCapable.h>
#include <mp2p_icp/layer_name_t.h>
#include <mp2p_icp/plane_patch.h>
#include <mp2p_icp/render_params.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/topography/data_types.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/// Frwd declarations:
namespace mrpt::containers
{
class yaml;
}

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_map_grp
 * @{
 */

/**
 * @brief Generic container of pointcloud(s), extracted features and other maps
 *
 * This class could be derived by users to define custom point cloud features,
 * for use in custom alignment algorithms.
 *
 * The class supports C++11/C++17 std::shared_from_this() via
 * get_shared_from_this();
 *
 */
class metric_map_t : public mrpt::serialization::CSerializable,
                     public std::enable_shared_from_this<metric_map_t>
{
    DEFINE_SERIALIZABLE(metric_map_t, mp2p_icp)

   public:
    /** @name Reserved point-cloud layer names (for use in `point_layers`)
     * @{ */

    constexpr static const char* PT_LAYER_RAW             = "raw";
    constexpr static const char* PT_LAYER_PLANE_CENTROIDS = "plane_centroids";

    /** @} */

    /** @name Data fields
     * @{ */

    /** Different layers indexed by a descriptive name, with point-clouds,
     *  2D/3D gridmap, etc.
     *
     * Standarized layer names: See section above.
     * - PT_LAYER_RAW: reserved to the original, full point cloud (if kept)
     * - PT_LAYER_PLANE_CENTROIDS: a point for each plane in `planes` (same
     * order).
     *
     * \sa point_layers()
     */
    std::map<layer_name_t, mrpt::maps::CMetricMap::Ptr> layers;

    /** 3D lines (infinite lines, not segments) */
    std::vector<mrpt::math::TLine3D> lines;

    /** Plane patches=centroid point + infinite plane */
    std::vector<plane_patch_t> planes;

    /** An optional numerical ID to identify the pointcloud in some higher-level system. Used to
     * build the names of ICP debug files, if so requested. It is not mandatory and even duplicates
     * may exist without problems: just a placeholder for the user of this library to use it.
     */
    std::optional<uint64_t> id;

    /** An optional textual identification/description of the pointcloud in some higher-level
     * system. Used to build the names of ICP debug files, if so requested. It is not mandatory and
     * even duplicates may exist without problems: just a placeholder for the user of this library
     * to use it.
     */
    std::optional<std::string> label;

    /** Generic metadata, in YAML format, to store any extra information by the user.
     *
     * Example usage:
     * \code
     *  metric_map_t map;
     *  map.metadata["timestamp"] = "2023-10-01T12:00:00Z";
     *  map.metadata["spot"] = "Alabama";
     *
     *  map.metadata["rules"] = mrpt::containers::yaml::Map();
     *  map.metadata["rules"]["first"] = "don't kill humans";
     *  map.metadata["rules"]["second"] = "do drinking";
     *
     *  map.metadata["sequence_example"] = mrpt::containers::yaml::Sequence();
     *  map.metadata["sequence_example"].push_back(1.0);
     *  map.metadata["sequence_example"].push_back("you can mix types");
     *  map.metadata["sequence_example"].push_back(true);
     * \endcode
     *
     * \note You can checkout the metadata of a ".mm" file using "mm-info" or "mm-viewer".
     */
    mrpt::containers::yaml metadata = mrpt::containers::yaml::Map();

    struct Georeferencing
    {
        /** The geodetic coordinates (on WGS-84) of the metric map ENU frame of
         * reference. */
        mrpt::topography::TGeodeticCoords geo_coord;

        /** The SE(3) transformation from the ENU (earth-north-up) frame to the metric map local
         * frame of reference. If this is the identity (default) it means the map is already in ENU
         * coordinates (i.e. +X is East, +Y is North, +Z is up) and the point (0,0,0) is the one
         * having the geodetic coordinates geo_coord
         */
        mrpt::poses::CPose3DPDFGaussian T_enu_to_map;
    };

    /** If provided, gives the placement of the map with respect to the Earth.
     */
    std::optional<Georeferencing> georeferencing;

    /** @} */

    /** @name Methods
     * @{ */

    /** return true if all point cloud layers, feature lists, etc. are empty */
    virtual bool empty() const;

    /** Overall number of elements (points, lines, planes) */
    virtual size_t size() const;

    /** Overall number of points, including all layers. */
    virtual size_t size_points_only() const;

    /** Returns a string summarizing all the elements in the container (points,
     * lines, planes) */
    virtual std::string contents_summary() const;

    /** clear all containers  */
    virtual void clear();

    /** Saves the metric_map_t object  to file, using MRPT serialization and
     *  using on-the-fly GZIP compression.
     * \return true on success.
     */
    bool save_to_file(const std::string& fileName) const;

    /** Loads the metric_map_t object from a file. See \save_to_file()
     * \return true on success.
     */
    bool load_from_file(const std::string& fileName);

    /** Returns a shared_ptr to the given point cloud layer, or throws if
     *  the layer does not exist or it contains a different type of metric map
     * (e.g. if it is a gridmap).
     *
     * Note that this method is only provided by convenience and
     * backwards-compatibility: users can always directly access `layers` and
     * perform a `std::dynamic_pointer_cast<>`.
     *
     */
    mrpt::maps::CPointsMap::Ptr point_layer(const layer_name_t& name) const;

    /** Gets a renderizable view of all geometric entities.
     *
     * See render_params_t for options to show/hide the different geometric
     * entities and point layers.
     *
     * \note If deriving user classes inheriting from metric_map_t, remember to
     *  reimplement this method and call this base class method to render
     *  common elements.
     */
    virtual auto get_visualization(const render_params_t& p = render_params_t()) const
        -> std::shared_ptr<mrpt::opengl::CSetOfObjects>;

    /** Merges all geometric entities from another point cloud into this one,
     * with an optional relative pose transformation.
     *
     * \note Point layers will be merged for coinciding names, or created if the
     * layer did not exist in `this`.
     * \note This method is virtual for user-extended point clouds can handle
     * other geometric primitives as needed.
     */
    virtual void merge_with(
        const metric_map_t&                       otherPc,
        const std::optional<mrpt::math::TPose3D>& otherRelativePose = std::nullopt);

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
    static void get_visualization_map_layer(
        mrpt::opengl::CSetOfObjects& o, const render_params_point_layer_t& p,
        const mrpt::maps::CMetricMap::Ptr& map);

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
    virtual void derivedSerializeTo([[maybe_unused]] mrpt::serialization::CArchive& out) const {}

    /** Implement in derived classes if new data fields are required */
    virtual void derivedSerializeFrom([[maybe_unused]] mrpt::serialization::CArchive& in) {}
};

/** Function to extract the CPointsMap for any kind of
 * CMetricMap, if there exists a conversion that makes sense for matching
 * against it.
 * Typically:
 *  - Any derived type of mrpt::maps::CPointsMap: just does a dynamic_cast.
 *  - An mrpt::maps::CVoxelMap: gets the equivalent points map with cached
 *    occupied voxels (requires MRPT >=2.11.0).
 *
 * Returns an empty shared_ptr if conversion is not possible.
 *
 * \note The use of raw pointers here imply the lifetime of the input "map"
 *       must be longer than that of its use within the Matcher.
 */
const mrpt::maps::CPointsMap* MapToPointsMap(const mrpt::maps::CMetricMap& map);

/// \overload for non-const input maps.
mrpt::maps::CPointsMap* MapToPointsMap(mrpt::maps::CMetricMap& map);

/** Returns the dynamic_cast<> of the metric map as a
 * mrpt::maps::NearestNeighborsCapable.
 * If the interface is not implemented it returns nullptr, or throws
 * an exception if it `throwIfNotImplemented` is `true`.
 */
const mrpt::maps::NearestNeighborsCapable* MapToNN(
    const mrpt::maps::CMetricMap& map, bool throwIfNotImplemented);

/** Returns the dynamic_cast<> of the metric map as a
 * mp2p_icp::NearestPlaneCapable.
 * If the interface is not implemented it returns nullptr, or throws
 * an exception if it `throwIfNotImplemented` is `true`.
 */
const mp2p_icp::NearestPlaneCapable* MapToNP(
    const mrpt::maps::CMetricMap& map, bool throwIfNotImplemented);

/// Serialization of geo-reference information in binary form:
mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, std::optional<metric_map_t::Georeferencing>& g);
/// Serialization of geo-reference information in binary form:
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const std::optional<metric_map_t::Georeferencing>& g);

/// Serialization of geo-reference information as YAML
std::optional<metric_map_t::Georeferencing> FromYAML(const mrpt::containers::yaml& yaml_data);

/// Serialization of geo-reference information as YAML
mrpt::containers::yaml ToYAML(const std::optional<metric_map_t::Georeferencing>& gref);

/** @} */

}  // namespace mp2p_icp
