/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   Matcher_Points_Base.h
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */
#pragma once

#include <mp2p_icp/Matcher.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdlib>
#include <limits>  // std::numeric_limits
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** Pointcloud matcher auxiliary class for iterating over point layers.
 *
 * The common to all matchers working on **points** in the `local` input metric
 * map. See derived classes for possible generated pairings.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_Base : public Matcher
{
   public:
    Matcher_Points_Base() = default;

    /** Explicit set of local point layers to match against each global point
     * layer. If empty, each `local` layer is matched against the `global`
     * layer with the identical name.
     * \note This field can be loaded from a configuration file, see
     * initialize().
     *
     * \note Map is: layers["globalLayer"] = {"localLayer1", "localLayer2", ...};
     */
    std::map<std::string, std::set<std::string>> pt2pt_layer_matches;

    /** Whether to allow matching *local* points that have been already matched
     * by a former Matcher instance in the pipeline. */
    bool allowMatchAlreadyMatchedPoints_ = false;

    /** If false, global map points can be paired exclusively against one local
     * point. */
    bool allowMatchAlreadyMatchedGlobalPoints_ = false;

    /** Maximum number of points per tree node. Not set: nanoflann default. */
    std::optional<std::size_t> kdtree_leaf_max_points_;

    /** The additional "margin" in all axes (x,y,z) that bounding box is
     * enlarged for checking the feasibility of pairings to exist. */
    float bounding_box_intersection_check_epsilon_ = 0.20f;

    /** Common parameters to all derived classes:
     *
     * - `pointLayerMatches`: Optional sequence of explicit `{global, local}`
     *  layer-name pairs to match against each other. Refer to example YAML
     *  files. A legacy `weight` key is still accepted for backward
     *  compatibility but is ignored; use the solver's `pair_weights.pt2pt`
     *  instead.
     *
     * - `allowMatchAlreadyMatchedPoints`: Optional (Default=false). Whether to
     * find matches for local points which were already paired by other matcher
     * in an earlier stage in the matchers pipeline.
     *
     * - `bounding_box_intersection_check_epsilon`: Optional (Default=0.20). The
     * additional "margin" in all axes (x,y,z) that bounding box is enlarged for
     * checking the feasibility of pairings to exist.
     */
    void initialize(const mrpt::containers::yaml& params) override;

    /** the output of transform_local_to_global() */
    struct TransformedLocalPointCloud
    {
       public:
        mrpt::math::TPoint3Df localMin{fMax, fMax, fMax};
        mrpt::math::TPoint3Df localMax{-fMax, -fMax, -fMax};

        /** Reordering indexes, used only if we had to pick random indexes */
        std::optional<std::vector<std::size_t>> idxs;

        /** Transformed local points */
        mrpt::aligned_std_vector<float> x_locals, y_locals, z_locals;

       private:
        static constexpr auto fMax = std::numeric_limits<float>::max();
    };

    static TransformedLocalPointCloud transform_local_to_global(
        const mrpt::maps::CPointsMap& pcLocal, const mrpt::poses::CPose3D& localPose);

   protected:
    bool impl_match(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc, MatchState& ms,
        Pairings& out) const override final;

   private:
    virtual void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const = 0;
};

}  // namespace mp2p_icp
