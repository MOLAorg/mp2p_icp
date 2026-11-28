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
 * @file   test-mp2p_graph_partition.cpp
 * @brief  Unit tests for the normalized-cut graph partition
 */

#include <mp2p_icp_filters/graph_partition.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <random>
#include <set>

using namespace mp2p_icp_filters;

namespace
{
// Nodes [first, first+n) fully connected with weight w
void addClique(std::vector<WeightedEdge>& e, uint32_t first, uint32_t n, double w)
{
    for (uint32_t i = 0; i < n; i++)
    {
        for (uint32_t j = i + 1; j < n; j++)
        {
            e.push_back({first + i, first + j, w});
        }
    }
}

void test_two_clusters()
{
    std::vector<WeightedEdge> e;
    addClique(e, 0, 20, 1.0);
    addClique(e, 20, 25, 0.8);
    e.push_back({3, 30, 0.05});
    e.push_back({7, 41, 0.05});

    const auto labels = spectral_partition(45, e);
    for (uint32_t k = 0; k < 45; k++)
    {
        ASSERT_EQUAL_(labels[k], k < 20 ? 0U : 1U);
    }

    // A strict threshold keeps them together:
    GraphPartitionParams p;
    p.max_ncut        = 1e-4;
    const auto single = spectral_partition(45, e, p);
    ASSERT_EQUAL_(std::set<uint32_t>(single.begin(), single.end()).size(), 1UL);
}

void test_components()
{
    // Three components, one of them a lone node: always split
    std::vector<WeightedEdge> e;
    addClique(e, 0, 10, 1.0);
    addClique(e, 11, 3, 1.0);
    GraphPartitionParams p;
    p.min_part_size   = 5;
    const auto labels = spectral_partition(14, e, p);
    ASSERT_EQUAL_(labels[0], 0U);
    ASSERT_EQUAL_(labels[9], 0U);
    ASSERT_EQUAL_(labels[10], 1U);
    ASSERT_EQUAL_(labels[11], 2U);
    ASSERT_EQUAL_(labels[13], 2U);
}

// A path where each node overlaps the next `reach` nodes, fading
std::vector<WeightedEdge> chain(uint32_t n, uint32_t reach)
{
    std::vector<WeightedEdge> e;
    for (uint32_t i = 0; i < n; i++)
    {
        for (uint32_t d = 1; d <= reach && i + d < n; d++)
        {
            e.push_back({i, i + d, 1.0 - static_cast<double>(d) / (reach + 1)});
        }
    }
    return e;
}

void test_chain_segments()
{
    const uint32_t       n = 200;
    GraphPartitionParams p;
    p.max_ncut        = 0.2;
    p.min_part_size   = 5;
    const auto labels = spectral_partition(n, chain(n, 8), p);

    // Contiguous segments, none shorter than the minimum:
    std::vector<uint32_t> sizes;
    for (uint32_t k = 0; k < n; k++)
    {
        if (k == 0 || labels[k] != labels[k - 1])
        {
            sizes.push_back(0);
            // Parts are numbered by their first node: in order along a chain
            ASSERT_EQUAL_(labels[k], static_cast<uint32_t>(sizes.size() - 1));
        }
        sizes.back()++;
    }
    ASSERT_GT_(sizes.size(), 3UL);
    for (const auto s : sizes)
    {
        ASSERT_GE_(s, p.min_part_size);
    }

    // Deterministic:
    ASSERT_(labels == spectral_partition(n, chain(n, 8), p));
}

void test_fiedler_vs_dense()
{
    // A random sparse connected graph
    const uint32_t            n = 120;
    std::mt19937              rng(7);
    std::vector<WeightedEdge> e = chain(n, 2);
    for (int k = 0; k < 300; k++)
    {
        const auto i = static_cast<uint32_t>(rng() % n);
        const auto j = static_cast<uint32_t>(rng() % n);
        if (i != j)
        {
            e.push_back({i, j, 0.01 + static_cast<double>(rng() % 100) / 1000.0});
        }
    }
    double     lambda2 = 0;
    const auto f       = fiedler_vector(n, e, {}, &lambda2);

    // Dense reference: D^-1/2 (D - W) D^-1/2
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(n, n);
    for (const auto& x : e)
    {
        W(x.i, x.j) += x.w;
        W(x.j, x.i) += x.w;
    }
    const Eigen::VectorXd d   = W.rowwise().sum();
    const Eigen::VectorXd isd = d.cwiseSqrt().cwiseInverse();
    const Eigen::MatrixXd L =
        Eigen::MatrixXd::Identity(n, n) - isd.asDiagonal() * W * isd.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(L);
    const Eigen::VectorXd ref = isd.asDiagonal() * es.eigenvectors().col(1);

    ASSERT_NEAR_(lambda2, es.eigenvalues()[1], 1e-6);
    Eigen::VectorXd fe(n);
    for (uint32_t k = 0; k < n; k++)
    {
        fe[k] = f[k];
    }
    const double c = std::abs(fe.normalized().dot(ref.normalized()));
    ASSERT_GT_(c, 0.9999);
}

void test_fiedler_disconnected()
{
    // An isolated node makes the graph disconnected: lambda2 is 0
    double     lambda2 = -1;
    const auto f       = fiedler_vector(3, {{0, 1, 1.0}}, {}, &lambda2);
    ASSERT_EQUAL_(f.size(), 3UL);
    ASSERT_NEAR_(lambda2, 0.0, 1e-12);
    ASSERT_NEAR_(f[2], 0.0, 1e-12);
}

void test_normalized_cut()
{
    std::vector<WeightedEdge> e = {{0, 1, 1.0}, {1, 2, 0.5}, {2, 3, 1.0}};
    // cut = 0.5, vol(A) = 1+1.5 = 2.5, vol(B) = 2.5
    const double v = normalized_cut(4, e, {true, true, false, false});
    ASSERT_NEAR_(v, 0.4, 1e-12);
    ASSERT_NEAR_(normalized_cut(4, e, {true, true, true, true}), 0.0, 1e-12);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_two_clusters();
        test_components();
        test_chain_segments();
        test_fiedler_vs_dense();
        test_fiedler_disconnected();
        test_normalized_cut();
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
