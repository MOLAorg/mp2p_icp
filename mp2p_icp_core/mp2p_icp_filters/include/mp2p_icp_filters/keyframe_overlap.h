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
 * @file   keyframe_overlap.h
 * @brief  Shared perceptual content between keyframes, as a weighted graph.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#pragma once

#include <mp2p_icp_filters/graph_partition.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Parameters for keyframe_signature()
struct KeyframeSignatureParams
{
    /// Voxel side [m]
    double voxel_size = 0.5;

    /// Add the direction of the surface, as seen from the sensor, to each voxel:
    /// the two faces of a wall or a floor slab are then different content.
    bool oriented = true;

    /// Points farther than this from the sensor are ignored [m]. 0: no limit.
    double max_range = 0;

    /// Voxels with fewer points are not planar; their direction is that of the
    /// viewing ray.
    std::size_t min_planar_points = 5;

    /// A voxel is planar if its smallest covariance eigenvalue is below this
    /// fraction of the middle one.
    double max_planarity_ratio = 0.1;
};

/** The content a keyframe observed: sorted, unique keys of the voxels holding
 *  its points (in the map frame), each one optionally with the direction of
 *  its surface (see KeyframeSignatureParams::oriented).
 */
using KeyframeSignature = std::vector<uint64_t>;

/** Builds the signature of a keyframe from its points in the map frame and the
 *  sensor position (also in the map frame).
 *
 *  With `oriented`, the key of each voxel includes one of six axis directions:
 *  that of the normal of its points, oriented towards the sensor, if they are
 *  planar; otherwise that of the viewing ray (in a separate set of six); and
 *  whether the voxel is above or below the sensor. Two keyframes then share a
 *  voxel only if they see the same side of the same surface from the same
 *  height side: the floor and the ceiling of a slab, or a wall where it crosses
 *  a slab, do not join two storeys. `sensorInMap` must be the position of the
 *  sensor itself, not that of a robot frame below it. Deterministic.
 */
[[nodiscard]] KeyframeSignature keyframe_signature(
    const mrpt::maps::CPointsMap& pointsInMap, const mrpt::math::TPoint3D& sensorInMap,
    const KeyframeSignatureParams& params = {});

/// Jaccard similarity |a & b| / |a | b| of two signatures (0 if both are empty)
[[nodiscard]] double signature_similarity(const KeyframeSignature& a, const KeyframeSignature& b);

/** A graph of keyframes (nodes) weighted by their shared content: the Jaccard
 *  similarity of their signatures. Nodes are added one at a time, each one
 *  compared at once with all previous nodes through an inverted index of
 *  voxels, so the cost of an insertion grows with the number of earlier nodes
 *  that saw each of its voxels, not with the number of nodes.
 */
class KeyframeOverlapGraph
{
   public:
    struct Parameters
    {
        /// Edges below this similarity are not stored
        double min_similarity = 0.02;
    };

    KeyframeOverlapGraph() = default;
    explicit KeyframeOverlapGraph(const Parameters& p) : params_(p) {}

    /// Adds a node and its edges to earlier nodes. Returns its index.
    uint32_t add_node(const KeyframeSignature& sig);

    /** Similarity of a signature with every node it shares content with, at
     *  least Parameters::min_similarity, sorted by node index. Does not
     *  modify the graph. */
    [[nodiscard]] std::vector<std::pair<uint32_t, double>> query(
        const KeyframeSignature& sig) const;

    [[nodiscard]] std::size_t node_count() const { return sizes_.size(); }

    /// Edges (i < j), in order of insertion of node j, then by i
    [[nodiscard]] const std::vector<WeightedEdge>& edges() const { return edges_; }

    /// Number of voxels in each node signature
    [[nodiscard]] const std::vector<std::size_t>& signature_sizes() const { return sizes_; }

   private:
    Parameters                                          params_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> index_;
    std::vector<std::size_t>                            sizes_;
    std::vector<WeightedEdge>                           edges_;

    // Shared voxels of `sig` with each node, as (node, count) sorted by node
    [[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> sharedCounts(
        const KeyframeSignature& sig) const;
};

/** @} */

}  // namespace mp2p_icp_filters
