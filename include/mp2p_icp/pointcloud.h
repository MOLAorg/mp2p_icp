/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pointcloud.h
 * @brief  Generic representation of pointcloud(s) and/or extracted features.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/serialization/CSerializable.h>
#include <map>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_grp
 * @{
 */

struct render_params_t
{
    render_params_t() = default;

    mrpt::img::TColor plane_color{0xff, 0xff, 0xff, 0xff};
    double            plane_half_width{1.0}, plane_grid_spacing{0.25};
};

struct plane_patch_t
{
    mrpt::math::TPlane3D plane;
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
 */
class pointcloud_t : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(pointcloud_t, mp2p_icp)

   public:
    /** @name Reserved point-cloud layer names (for use in `point_layers`)
     * @{ */

    constexpr static const char* PT_LAYER_RAW{"raw"};
    constexpr static const char* PT_LAYER_PLANE_CENTROIDS{"plane_centroids"};

    /** @} */

    /** Different point layers, indexed by a descriptive name.
     * Known layer names: See section above.
     * - PT_LAYER_RAW: reserved to the original, full point cloud (if kept)
     * - PT_LAYER_PLANE_CENTROIDS: a point for each plane in `planes` (same
     * order).
     */
    std::map<std::string, mrpt::maps::CPointsMap::Ptr> point_layers;
    std::vector<mrpt::math::TLine3D>                   lines;
    std::vector<plane_patch_t>                         planes;

    /** return true if all point cloud layers, feature lists, etc. are empty */
    bool empty() const;
    /** clear all containers  */
    void clear();

    /** Gets a renderizable view of all planes. The target container `o` is not
     * cleared(), clear() it manually if needed before calling. */
    void planesAsRenderizable(
        mrpt::opengl::CSetOfObjects& o,
        const render_params_t&       p = render_params_t());
};

/** @} */

}  // namespace mp2p_icp
