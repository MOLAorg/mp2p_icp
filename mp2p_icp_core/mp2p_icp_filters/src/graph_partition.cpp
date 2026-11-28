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
 * @file   graph_partition.cpp
 * @brief  Normalized-cut partition of sparse weighted graphs.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#include <mp2p_icp_filters/graph_partition.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace mp2p_icp_filters
{
namespace
{
// Symmetric adjacency in compressed rows
struct Csr
{
    std::vector<size_t>   start;  // n+1
    std::vector<uint32_t> nbr;
    std::vector<double>   w;

    [[nodiscard]] size_t size() const { return start.empty() ? 0 : start.size() - 1; }
};

// Adjacency of the nodes in `nodes` (global ids), renumbered 0..n-1 in that
// order. `local` is a global-sized scratch lookup, all -1 on entry and exit.
Csr subgraph(const Csr& g, const std::vector<uint32_t>& nodes, std::vector<int64_t>& local)
{
    for (size_t k = 0; k < nodes.size(); k++)
    {
        local[nodes[k]] = static_cast<int64_t>(k);
    }
    Csr s;
    s.start.reserve(nodes.size() + 1);
    s.start.push_back(0);
    for (const uint32_t u : nodes)
    {
        for (size_t e = g.start[u]; e < g.start[u + 1]; e++)
        {
            if (const int64_t v = local[g.nbr[e]]; v >= 0)
            {
                s.nbr.push_back(static_cast<uint32_t>(v));
                s.w.push_back(g.w[e]);
            }
        }
        s.start.push_back(s.nbr.size());
    }
    for (const uint32_t u : nodes)
    {
        local[u] = -1;
    }
    return s;
}

Csr buildCsr(size_t n, const std::vector<WeightedEdge>& edges)
{
    std::vector<size_t> count(n, 0);
    for (const auto& e : edges)
    {
        ASSERTMSG_(e.i < n && e.j < n, "spectral_partition(): edge node index out of range");
        ASSERTMSG_(e.w >= 0 && std::isfinite(e.w), "spectral_partition(): invalid edge weight");
        if (e.i != e.j && e.w > 0)
        {
            count[e.i]++;
            count[e.j]++;
        }
    }
    Csr g;
    g.start.assign(n + 1, 0);
    for (size_t k = 0; k < n; k++)
    {
        g.start[k + 1] = g.start[k] + count[k];
    }
    g.nbr.resize(g.start[n]);
    g.w.resize(g.start[n]);
    std::vector<size_t> pos(g.start.begin(), g.start.end() - 1);
    for (const auto& e : edges)
    {
        if (e.i != e.j && e.w > 0)
        {
            g.nbr[pos[e.i]] = e.j;
            g.w[pos[e.i]++] = e.w;
            g.nbr[pos[e.j]] = e.i;
            g.w[pos[e.j]++] = e.w;
        }
    }
    return g;
}

std::vector<double> degrees(const Csr& g)
{
    std::vector<double> d(g.size(), 0.0);
    for (size_t u = 0; u < g.size(); u++)
    {
        for (size_t e = g.start[u]; e < g.start[u + 1]; e++)
        {
            d[u] += g.w[e];
        }
    }
    return d;
}

// Connected components, as lists of local node indices in increasing order
std::vector<std::vector<uint32_t>> components(const Csr& g)
{
    const size_t                       n = g.size();
    std::vector<int64_t>               comp(n, -1);
    std::vector<std::vector<uint32_t>> out;
    std::vector<uint32_t>              queue;
    for (size_t s = 0; s < n; s++)
    {
        if (comp[s] >= 0)
        {
            continue;
        }
        const auto id = static_cast<int64_t>(out.size());
        out.emplace_back();
        queue.assign(1, static_cast<uint32_t>(s));
        comp[s] = id;
        for (size_t q = 0; q < queue.size(); q++)
        {
            const uint32_t u = queue[q];
            for (size_t e = g.start[u]; e < g.start[u + 1]; e++)
            {
                if (comp[g.nbr[e]] < 0)
                {
                    comp[g.nbr[e]] = id;
                    queue.push_back(g.nbr[e]);
                }
            }
        }
        std::sort(queue.begin(), queue.end());
        out.back() = queue;
    }
    return out;
}

// Fiedler vector of a connected graph with n >= 2 nodes (all degrees > 0):
// Lanczos with full reorthogonalization on A = D^-1/2 W D^-1/2, restricted to
// the complement of its top eigenvector D^1/2 1, restarted from the Ritz vector.
std::vector<double> fiedlerConnected(
    const Csr& g, const std::vector<double>& d, const GraphPartitionParams& p, double* lambda2)
{
    const size_t n = g.size();
    ASSERT_GE_(n, 2UL);

    Eigen::VectorXd sd(n);
    for (size_t k = 0; k < n; k++)
    {
        ASSERT_GT_(d[k], 0.0);
        sd[k] = std::sqrt(d[k]);
    }
    const Eigen::VectorXd v1 = sd.normalized();

    const auto matvec = [&](const Eigen::VectorXd& x, Eigen::VectorXd& y)
    {
        for (size_t u = 0; u < n; u++)
        {
            double acc = 0;
            for (size_t e = g.start[u]; e < g.start[u + 1]; e++)
            {
                acc += g.w[e] * x[g.nbr[e]] / sd[g.nbr[e]];
            }
            y[u] = acc / sd[u];
        }
    };

    // Deterministic start vector
    Eigen::VectorXd q(n);
    {
        std::mt19937 rng(12345);
        for (size_t k = 0; k < n; k++)
        {
            q[k] = static_cast<double>(rng()) / 4294967296.0 - 0.5;
        }
    }

    const size_t    m = std::max<size_t>(1, std::min(p.lanczos_size, n - 1));
    Eigen::MatrixXd Q(n, m);
    Eigen::VectorXd r(n);
    Eigen::VectorXd ritz  = q;
    double          theta = 0;

    for (size_t restart = 0; restart <= p.lanczos_max_restarts; restart++)
    {
        q = ritz;
        q -= v1 * v1.dot(q);
        q -= v1 * v1.dot(q);
        const double qn = q.norm();
        ASSERT_(qn > 0);
        q /= qn;

        std::vector<double> alpha;
        std::vector<double> beta;
        size_t              k = 0;
        for (; k < m; k++)
        {
            Q.col(static_cast<Eigen::Index>(k)) = q;
            matvec(q, r);
            const double a = q.dot(r);
            alpha.push_back(a);
            // Full reorthogonalization, twice, also against v1:
            for (int pass = 0; pass < 2; pass++)
            {
                const auto Qk = Q.leftCols(static_cast<Eigen::Index>(k + 1));
                r -= Qk * (Qk.transpose() * r);
                r -= v1 * v1.dot(r);
            }
            const double b = r.norm();
            if (k + 1 == m || b < 1e-12)
            {
                beta.push_back(b);
                k++;
                break;
            }
            beta.push_back(b);
            q = r / b;
        }

        // Largest Ritz pair of the tridiagonal matrix:
        Eigen::VectorXd diag(static_cast<Eigen::Index>(k));
        Eigen::VectorXd sub(static_cast<Eigen::Index>(std::max<size_t>(1, k) - 1));
        for (size_t i = 0; i < k; i++)
        {
            diag[static_cast<Eigen::Index>(i)] = alpha[i];
            if (i + 1 < k)
            {
                sub[static_cast<Eigen::Index>(i)] = beta[i];
            }
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
        es.computeFromTridiagonal(diag, sub, Eigen::ComputeEigenvectors);
        const auto      last = static_cast<Eigen::Index>(k - 1);
        Eigen::VectorXd s    = es.eigenvectors().col(last);
        theta                = es.eigenvalues()[last];
        ritz                 = Q.leftCols(static_cast<Eigen::Index>(k)) * s;
        ritz.normalize();

        const double residual = std::abs(beta.back() * s[last]);
        if (residual < p.lanczos_tolerance || beta.back() < 1e-12 || k < m)
        {
            break;
        }
    }

    if (lambda2)
    {
        *lambda2 = std::max(0.0, 1.0 - theta);
    }
    std::vector<double> f(n);
    for (size_t k = 0; k < n; k++)
    {
        f[k] = ritz[static_cast<Eigen::Index>(k)] / sd[static_cast<Eigen::Index>(k)];
    }
    return f;
}

struct Split
{
    bool                  found = false;
    double                ncut  = std::numeric_limits<double>::infinity();
    std::vector<uint32_t> a;  // local indices
    std::vector<uint32_t> b;
};

// Minimum normalized cut among the splits along the ordering of `f`
Split sweepCut(
    const Csr& g, const std::vector<double>& d, const std::vector<double>& f, size_t minSize)
{
    const size_t        n = g.size();
    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(
        order.begin(), order.end(),
        [&](size_t x, size_t y) { return f[x] < f[y] || (f[x] == f[y] && x < y); });

    const double      vol  = std::accumulate(d.begin(), d.end(), 0.0);
    double            cut  = 0;
    double            volS = 0;
    std::vector<bool> inS(n, false);
    Split             best;
    size_t            bestK = 0;
    for (size_t k = 0; k + 1 < n; k++)
    {
        const size_t u = order[k];
        for (size_t e = g.start[u]; e < g.start[u + 1]; e++)
        {
            cut += inS[g.nbr[e]] ? -g.w[e] : g.w[e];
        }
        inS[u] = true;
        volS += d[u];
        if (k + 1 < minSize || n - k - 1 < minSize)
        {
            continue;
        }
        const double c    = std::max(0.0, cut);
        const double ncut = c / volS + c / (vol - volS);
        if (ncut < best.ncut)
        {
            best.ncut  = ncut;
            best.found = true;
            bestK      = k;
        }
    }
    if (best.found)
    {
        for (size_t k = 0; k < n; k++)
        {
            (k <= bestK ? best.a : best.b).push_back(static_cast<uint32_t>(order[k]));
        }
        std::sort(best.a.begin(), best.a.end());
        std::sort(best.b.begin(), best.b.end());
    }
    return best;
}

std::vector<uint32_t> toGlobal(const std::vector<uint32_t>& loc, const std::vector<uint32_t>& nodes)
{
    std::vector<uint32_t> out;
    out.reserve(loc.size());
    for (const uint32_t k : loc)
    {
        out.push_back(nodes[k]);
    }
    return out;
}

}  // namespace

double normalized_cut(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges, const std::vector<bool>& inA)
{
    ASSERT_EQUAL_(inA.size(), nodeCount);
    double cut  = 0;
    double volA = 0;
    double volB = 0;
    for (const auto& e : edges)
    {
        ASSERT_(e.i < nodeCount && e.j < nodeCount);
        if (e.i == e.j || !(e.w > 0))
        {
            continue;
        }
        (inA[e.i] ? volA : volB) += e.w;
        (inA[e.j] ? volA : volB) += e.w;
        if (inA[e.i] != inA[e.j])
        {
            cut += e.w;
        }
    }
    if (cut == 0)
    {
        return 0;
    }
    return cut / volA + cut / volB;
}

std::vector<double> fiedler_vector(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges,
    const GraphPartitionParams& params, double* lambda2)
{
    MRPT_START
    const Csr             g = buildCsr(nodeCount, edges);
    const auto            d = degrees(g);
    std::vector<uint32_t> active;
    for (size_t k = 0; k < nodeCount; k++)
    {
        if (d[k] > 0)
        {
            active.push_back(static_cast<uint32_t>(k));
        }
    }
    std::vector<double> f(nodeCount, 0.0);
    if (lambda2)
    {
        *lambda2 = 0;
    }
    if (active.size() < 2)
    {
        return f;
    }
    std::vector<int64_t> local(nodeCount, -1);
    const Csr            s  = subgraph(g, active, local);
    const auto           fs = fiedlerConnected(s, degrees(s), params, lambda2);
    for (size_t k = 0; k < active.size(); k++)
    {
        f[active[k]] = fs[k];
    }
    return f;
    MRPT_END
}

std::vector<uint32_t> spectral_partition(
    std::size_t nodeCount, const std::vector<WeightedEdge>& edges,
    const GraphPartitionParams& params)
{
    MRPT_START
    ASSERT_GE_(params.min_part_size, 1UL);

    std::vector<uint32_t> labels(nodeCount, 0);
    if (nodeCount == 0)
    {
        return labels;
    }

    const Csr                          g = buildCsr(nodeCount, edges);
    std::vector<int64_t>               local(nodeCount, -1);
    std::vector<std::vector<uint32_t>> stack;
    std::vector<std::vector<uint32_t>> leaves;

    stack.emplace_back(nodeCount);
    std::iota(stack.back().begin(), stack.back().end(), 0U);

    while (!stack.empty())
    {
        const std::vector<uint32_t> nodes = std::move(stack.back());
        stack.pop_back();

        const Csr  s     = subgraph(g, nodes, local);
        const auto comps = components(s);
        if (comps.size() > 1)
        {
            for (const auto& c : comps)
            {
                stack.push_back(toGlobal(c, nodes));
            }
            continue;
        }
        if (nodes.size() < 2 * params.min_part_size || nodes.size() < 2)
        {
            leaves.push_back(nodes);
            continue;
        }
        const auto d     = degrees(s);
        const auto f     = fiedlerConnected(s, d, params, nullptr);
        const auto split = sweepCut(s, d, f, params.min_part_size);
        if (!split.found || split.ncut > params.max_ncut)
        {
            leaves.push_back(nodes);
            continue;
        }
        stack.push_back(toGlobal(split.a, nodes));
        stack.push_back(toGlobal(split.b, nodes));
    }

    // Number the parts by their lowest node:
    std::sort(
        leaves.begin(), leaves.end(),
        [](const auto& a, const auto& b) { return a.front() < b.front(); });
    for (size_t p = 0; p < leaves.size(); p++)
    {
        for (const uint32_t u : leaves[p])
        {
            labels[u] = static_cast<uint32_t>(p);
        }
    }
    return labels;
    MRPT_END
}

}  // namespace mp2p_icp_filters
