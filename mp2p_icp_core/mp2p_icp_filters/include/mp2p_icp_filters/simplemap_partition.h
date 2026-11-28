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
 * @file   simplemap_partition.h
 * @brief  Splits a simple-map into submaps of keyframes that observe the same place.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#pragma once

#include <mp2p_icp_filters/graph_partition.h>
#include <mp2p_icp_filters/keyframe_overlap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Parameters for partition_simplemap()
struct SimplemapPartitionParams
{
    /// A keyframe becomes a graph node when it is this far [m], or turned this
    /// much [deg], from the previous node.
    double sample_distance  = 2.0;
    double sample_angle_deg = 30.0;

    /// Voxel size [m] the keyframe points are decimated to before their
    /// signature is computed. 0: no decimation.
    double decimation_voxel = 0.1;

    KeyframeSignatureParams          signature;
    KeyframeOverlapGraph::Parameters graph;
    GraphPartitionParams             partition;

    /// Most similar nodes kept per keyframe (KeyframePartition::keyframe_links)
    std::size_t max_keyframe_links = 8;

    /// sm2mm pipeline turning a keyframe into points. Empty: the raw points of
    /// every observation, decimated with `decimation_voxel`.
    mrpt::containers::yaml pipeline;
};

/** A split of the keyframes of a simple-map into submaps, each one a group of
 *  keyframes that observe the same place, plus the similarity graph it comes
 *  from. Some keyframes (the nodes) are sampled to build the graph; every
 *  keyframe is then linked to the nodes it shares content with.
 */
struct SimplemapPartition
{
    /// Keyframe index of each node
    std::vector<uint32_t> node_keyframe;

    /// Similarity between nodes (node indices, i < j)
    std::vector<WeightedEdge> node_edges;

    /// Submap of each node
    std::vector<uint32_t> node_submap;

    /// Size (voxels) of the signature of each node: with the similarity of an
    /// edge, it gives the content two nodes share.
    std::vector<uint32_t> node_voxels;

    /// For each keyframe: the nodes it shares content with and the similarity,
    /// sorted by node. A node keyframe lists itself with 1.
    std::vector<std::vector<std::pair<uint32_t, double>>> keyframe_links;

    /// Submap of each keyframe: that of its most similar node, or, without
    /// links (no points), that of the closest keyframe in the sequence.
    std::vector<uint32_t> keyframe_submap;

    [[nodiscard]] std::size_t keyframe_count() const { return keyframe_submap.size(); }
    [[nodiscard]] std::size_t submap_count() const;

    /// Keyframes of each submap, in increasing order
    [[nodiscard]] std::vector<std::vector<size_t>> submap_keyframes() const;

    void                                    save_to_file(const std::string& file) const;
    [[nodiscard]] static SimplemapPartition load_from_file(const std::string& file);
};

/// Statistics of partition_simplemap()
struct SimplemapPartitionStats
{
    double      signatures_seconds = 0;  //!< Points and signatures of all keyframes
    double      graph_seconds      = 0;  //!< Node insertion and keyframe queries
    double      partition_seconds  = 0;  //!< Spectral partition of the node graph
    std::size_t signature_voxels   = 0;  //!< Sum of the voxels of all node signatures
};

/** Splits the keyframes of `sm` into submaps of shared content: samples nodes
 *  (SimplemapPartitionParams::sample_distance, sample_angle_deg), builds their
 *  similarity graph from keyframe_signature() in the map frame, partitions it
 *  with spectral_partition(), and links every keyframe to its most similar
 *  nodes. Keyframe poses are used as they are: no registration.
 */
[[nodiscard]] SimplemapPartition partition_simplemap(
    const mrpt::maps::CSimpleMap& sm, const SimplemapPartitionParams& params = {},
    SimplemapPartitionStats* stats = nullptr);

/// The up estimate of one submap, for keyframe_ups_from_submaps()
struct SubmapUp
{
    mrpt::math::TVector3D up{0, 0, 1};  //!< Unit up vector, in the map frame

    /// Covariance [rad^2] of the horizontal components (x, y) of `up`, as in
    /// EstimateUpResult::up_cov: an axis the walls do not constrain counts
    /// little along it, and fully along the other one. UpFieldParams::min_sigma_deg
    /// is always added. Default: zero.
    mrpt::math::CMatrixDouble22 cov;
};

/// Parameters for keyframe_ups_from_submaps()
struct UpFieldParams
{
    /// Stiffness of the up field: standard deviation [deg] of the change of up
    /// between two nodes that share as much content as a typical node sees
    /// (the median signature size), and larger as they share less, with the
    /// inverse square root of the shared content. Two keyframes that see the
    /// same surface with different corrections draw it twice, apart by about
    /// that difference times the range: this keeps differences between nodes
    /// that share content small, unless the submap estimates demand them.
    double step_sigma_deg = 1.0;

    /// Added to the submap sigmas [deg]: the systematic error of the references
    /// of a place (floors that are not quite level, facades that lean), which
    /// the covariance of the fit does not see. Differences between submaps
    /// smaller than this are taken as noise rather than as a tilt that varies.
    double min_sigma_deg = 0.3;
};

/** Up vector of each keyframe, from the up estimate of each submap
 *  (std::nullopt where a submap has none).
 *
 *  The up of the nodes is a smooth field over the node graph: the most
 *  probable one given the submap estimates, each one a measurement of the up
 *  of its nodes with its covariance (counted once per submap, whatever its
 *  size),
 *  and a prior that nodes sharing content have similar ups (see
 *  UpFieldParams::step_sigma_deg). One sparse solve. Nodes without an
 *  estimate of their own (a submap without a usable up) follow their
 *  neighbors; a part of the graph with no estimate at all takes the mean of
 *  all of them. Each keyframe then takes the mean of the ups of the nodes it
 *  is linked to, weighted by similarity, or without links, that of the
 *  closest keyframe in the sequence.
 *
 *  \exception std::exception If no submap has an up.
 */
[[nodiscard]] std::vector<mrpt::math::TVector3D> keyframe_ups_from_submaps(
    const SimplemapPartition& partition, const std::vector<std::optional<SubmapUp>>& submapUp,
    const UpFieldParams& params = {});

/** Edges between keyframes that observe the same place, for
 *  level_poses_by_place(): each keyframe to the keyframes of its linked
 *  nodes, if closer than `maxLength` [m], with weight `weight` times their
 *  similarity. Consecutive keyframes are not included.
 */
[[nodiscard]] std::vector<WeightedEdge> keyframe_place_links(
    const SimplemapPartition& partition, const std::vector<mrpt::math::TPoint3D>& kfPositions,
    double maxLength, double weight);

/** @} */

}  // namespace mp2p_icp_filters
