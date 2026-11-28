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
 * @file   graph_partition.h
 * @brief  Normalized-cut partition of sparse weighted graphs.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// An undirected graph edge between nodes `i` and `j` with weight `w` > 0
struct WeightedEdge
{
    uint32_t i = 0;
    uint32_t j = 0;
    double   w = 0;
};

/// Parameters for spectral_partition()
struct GraphPartitionParams
{
    /// A bisection is accepted if its normalized cut (in [0, 2]) is at most this
    double max_ncut = 0.2;

    /// Neither side of an accepted bisection may have fewer nodes than this.
    /// Disconnected components are always split, whatever their size.
    std::size_t min_part_size = 5;

    /// Krylov subspace size of each Lanczos run for the Fiedler vector
    std::size_t lanczos_size = 60;

    /// Lanczos restarts before taking the current Ritz vector as it is (it is
    /// still a good direction to cut along: the sweep evaluates the true cut)
    std::size_t lanczos_max_restarts = 200;

    /// Convergence threshold on the Ritz residual of the Fiedler vector
    double lanczos_tolerance = 1e-6;
};

/// Statistics of spectral_partition()
struct GraphPartitionStats
{
    std::size_t eigen_solves = 0;  //!< Fiedler vectors computed
    std::size_t unconverged  = 0;  //!< Of them, those that hit lanczos_max_restarts
};

/** Normalized cut (Shi and Malik) of a two-way split of a graph:
 *  cut(A,B)/vol(A) + cut(A,B)/vol(B), with vol() the sum of the degrees.
 *  `inA[k]` tells the side of node k. Returns 0 if there is no cut edge.
 */
[[nodiscard]] double normalized_cut(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges, const std::vector<bool>& inA);

/** Fiedler vector of a connected graph: the eigenvector y of the second smallest
 *  eigenvalue of the generalized problem (D - W) y = lambda D y, whose sign and
 *  ordering give the relaxed minimum normalized cut. Computed with Lanczos on
 *  the normalized adjacency D^-1/2 W D^-1/2 deflated by its known top
 *  eigenvector, so the cost grows with the number of edges, not the square of
 *  the nodes. Deterministic.
 *
 *  `lambda2`, if given, receives that eigenvalue (0 for a disconnected graph),
 *  and `converged` whether the Ritz residual reached lanczos_tolerance.
 */
[[nodiscard]] std::vector<double> fiedler_vector(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges,
    const GraphPartitionParams& params = {}, double* lambda2 = nullptr, bool* converged = nullptr);

/** Splits a graph by recursive normalized-cut bisection: connected components
 *  first, then each part is cut along the ordering of its Fiedler vector at
 *  the position of minimum normalized cut (a sweep), as long as that cut is
 *  at most GraphPartitionParams::max_ncut and both sides keep
 *  GraphPartitionParams::min_part_size nodes.
 *
 *  Returns the part index of each node. Parts are numbered by their lowest
 *  node index. Deterministic. Duplicate edges add up; self loops are ignored.
 */
[[nodiscard]] std::vector<uint32_t> spectral_partition(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges,
    const GraphPartitionParams& params = {}, GraphPartitionStats* stats = nullptr);

/** @} */

}  // namespace mp2p_icp_filters
