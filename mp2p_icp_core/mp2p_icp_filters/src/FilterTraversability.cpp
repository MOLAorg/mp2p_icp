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
 * @file   FilterTraversability.cpp
 * @brief  2D obstacles for ground vehicles from a 3D point cloud (2.5D ground model)
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#include <mp2p_icp_filters/FilterTraversability.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>

#include <cmath>
#include <deque>
#include <limits>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(FilterTraversability, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// Margin for sensor noise when comparing ground heights [m]:
constexpr float kGroundNoise = 0.10f;

// Upper limit on the number of cells of the dense grid:
constexpr size_t kMaxCells = 200'000'000;

constexpr int kNeighbors8[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                                   {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

struct Grid
{
    float x0  = 0;
    float y0  = 0;
    float res = 1;
    int   nx  = 0;
    int   ny  = 0;

    size_t size() const { return static_cast<size_t>(nx) * static_cast<size_t>(ny); }
    bool   inside(int i, int j) const { return i >= 0 && j >= 0 && i < nx && j < ny; }
    size_t idx(int i, int j) const
    {
        return static_cast<size_t>(j) * static_cast<size_t>(nx) + static_cast<size_t>(i);
    }
    int   cx(float x) const { return static_cast<int>(std::floor((x - x0) / res)); }
    int   cy(float y) const { return static_cast<int>(std::floor((y - y0) / res)); }
    float centerX(int i) const { return x0 + (static_cast<float>(i) + 0.5f) * res; }
    float centerY(int j) const { return y0 + (static_cast<float>(j) + 0.5f) * res; }
};

// Minimum over a (2*half+1)^2 window (separable), with +inf outside the grid:
std::vector<float> minFilter(const Grid& g, const std::vector<float>& in, int half)
{
    std::vector<float> tmp(in.size(), kInf);
    std::vector<float> out(in.size(), kInf);
    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            float m = kInf;
            for (int d = -half; d <= half; d++)
            {
                if (g.inside(i + d, j))
                {
                    m = std::min(m, in[g.idx(i + d, j)]);
                }
            }
            tmp[g.idx(i, j)] = m;
        }
    }
    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            float m = kInf;
            for (int d = -half; d <= half; d++)
            {
                if (g.inside(i, j + d))
                {
                    m = std::min(m, tmp[g.idx(i, j + d)]);
                }
            }
            out[g.idx(i, j)] = m;
        }
    }
    return out;
}

// Chessboard distance (in cells) from the cells where `source` is true, up to
// `maxSteps` (larger values stay at maxSteps+1). Optionally, it also propagates
// the value of `carry` from the source cells.
std::vector<int> distanceFrom(
    const Grid& g, const std::vector<uint8_t>& source, int maxSteps,
    std::vector<float>* carry = nullptr)
{
    std::vector<int>                dist(g.size(), maxSteps + 1);
    std::deque<std::pair<int, int>> q;
    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            if (source[g.idx(i, j)])
            {
                dist[g.idx(i, j)] = 0;
                q.emplace_back(i, j);
            }
        }
    }
    while (!q.empty())
    {
        const auto [i, j] = q.front();
        q.pop_front();
        const int d = dist[g.idx(i, j)];
        if (d >= maxSteps)
        {
            continue;
        }
        for (const auto& n : kNeighbors8)
        {
            const int ii = i + n[0];
            const int jj = j + n[1];
            if (g.inside(ii, jj) && dist[g.idx(ii, jj)] > d + 1)
            {
                dist[g.idx(ii, jj)] = d + 1;
                if (carry)
                {
                    (*carry)[g.idx(ii, jj)] = (*carry)[g.idx(i, j)];
                }
                q.emplace_back(ii, jj);
            }
        }
    }
    return dist;
}

}  // namespace

void FilterTraversability::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterTraversability& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, seeds_pointcloud_layer);
    MCP_LOAD_OPT(c, output_layer_obstacles);
    MCP_LOAD_OPT(c, output_layer_traversable);
    MCP_LOAD_OPT(c, body_min_points);
    MCP_LOAD_OPT(c, unknown_is_obstacle);

    DECLARE_PARAMETER_IN_OPT(c, resolution, parent);
    DECLARE_PARAMETER_IN_OPT(c, max_slope_deg, parent);
    DECLARE_PARAMETER_IN_OPT(c, slope_window_radius, parent);
    DECLARE_PARAMETER_IN_OPT(c, max_step, parent);
    DECLARE_PARAMETER_IN_OPT(c, body_min_height, parent);
    DECLARE_PARAMETER_IN_OPT(c, body_max_height, parent);
    DECLARE_PARAMETER_IN_OPT(c, ground_envelope_size, parent);
    DECLARE_PARAMETER_IN_OPT(c, ground_envelope_slope_deg, parent);
    DECLARE_PARAMETER_IN_OPT(c, ground_fill_distance, parent);
    DECLARE_PARAMETER_IN_OPT(c, ground_fill_distance_unobserved, parent);
    DECLARE_PARAMETER_IN_OPT(c, seeds_radius, parent);
    DECLARE_PARAMETER_IN_OPT(c, obstacle_distance, parent);

    ASSERTMSG_(
        !output_layer_obstacles.empty() || !output_layer_traversable.empty(),
        "At least one 'output_layer_obstacles' or 'output_layer_traversable' must be "
        "provided.");
}

FilterTraversability::FilterTraversability()
{
    mrpt::system::COutputLogger::setLoggerName("FilterTraversability");
}

void FilterTraversability::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterTraversability::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    const auto& p = params;
    ASSERT_GT_(p.resolution, 0.0f);
    ASSERT_GT_(p.body_max_height, p.body_min_height);

    const auto pcPtr = inOut.point_layer(p.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", p.input_pointcloud_layer.c_str()));
    const auto& xs = pcPtr->getPointsBufferRef_x();
    const auto& ys = pcPtr->getPointsBufferRef_y();
    const auto& zs = pcPtr->getPointsBufferRef_z();

    mrpt::maps::CPointsMap::Ptr seedsPc;
    if (!p.seeds_pointcloud_layer.empty())
    {
        seedsPc = inOut.point_layer(p.seeds_pointcloud_layer);
        ASSERTMSG_(
            seedsPc,
            mrpt::format(
                "Seeds point cloud layer '%s' was not found.", p.seeds_pointcloud_layer.c_str()));
    }

    auto outObstacles   = GetOrCreatePointLayer(inOut, p.output_layer_obstacles);
    auto outTraversable = GetOrCreatePointLayer(inOut, p.output_layer_traversable);

    if (xs.empty())
    {
        return;
    }

    // Grid over the bounding box of the input, with a one cell margin:
    Grid g;
    g.res = p.resolution;
    {
        const auto bb = pcPtr->boundingBox();
        g.x0          = bb.min.x - g.res;
        g.y0          = bb.min.y - g.res;
        // Check the size before converting to int:
        const double nx = std::ceil((bb.max.x - g.x0) / g.res) + 2;
        const double ny = std::ceil((bb.max.y - g.y0) / g.res) + 2;
        ASSERTMSG_(
            nx * ny <= static_cast<double>(kMaxCells),
            mrpt::format("Grid of %.0fx%.0f cells is too large: increase 'resolution'.", nx, ny));
        g.nx = static_cast<int>(nx);
        g.ny = static_cast<int>(ny);
    }

    // 1) Ground model -------------------------------------------------------
    // Lowest point per cell:
    std::vector<float>  lowest(g.size(), kInf);
    std::vector<size_t> cellOfPoint(xs.size());
    for (size_t k = 0; k < xs.size(); k++)
    {
        const size_t c = g.idx(g.cx(xs[k]), g.cy(ys[k]));
        cellOfPoint[k] = c;
        lowest[c]      = std::min(lowest[c], zs[k]);
    }

    // Accept it as ground only if close to the lower envelope around it:
    const int envHalf =
        std::max(0, static_cast<int>(std::round(p.ground_envelope_size / g.res)) / 2);
    const auto  envelope = minFilter(g, lowest, envHalf);
    const float envTol =
        0.5f * p.ground_envelope_size * std::tan(mrpt::DEG2RAD(p.ground_envelope_slope_deg)) +
        kGroundNoise;

    std::vector<float>   ground(g.size(), kNaN);
    std::vector<uint8_t> accepted(g.size(), 0);
    for (size_t c = 0; c < g.size(); c++)
    {
        if (std::isfinite(lowest[c]) && lowest[c] - envelope[c] <= envTol)
        {
            accepted[c] = 1;
            ground[c]   = lowest[c];
        }
    }

    // Observed cells without ground (e.g. under a tree canopy, where the ground
    // hits may be meters apart) get their ground interpolated, by diffusion from
    // the accepted cells, up to ground_fill_distance:
    const int fillStepsObserved = static_cast<int>(std::floor(p.ground_fill_distance / g.res));
    if (fillStepsObserved > 0)
    {
        const auto           distToGround = distanceFrom(g, accepted, fillStepsObserved);
        std::vector<uint8_t> hasValue     = accepted;
        for (int iter = 0; iter < 4 * fillStepsObserved; iter++)
        {
            auto newGround   = ground;
            auto newHasValue = hasValue;
            for (int j = 0; j < g.ny; j++)
            {
                for (int i = 0; i < g.nx; i++)
                {
                    const size_t c = g.idx(i, j);
                    if (accepted[c] || !std::isfinite(lowest[c]) ||
                        distToGround[c] > fillStepsObserved)
                    {
                        continue;
                    }
                    double sum = 0;
                    int    n   = 0;
                    for (int dj = -1; dj <= 1; dj++)
                    {
                        for (int di = -1; di <= 1; di++)
                        {
                            if (g.inside(i + di, j + dj) && hasValue[g.idx(i + di, j + dj)])
                            {
                                sum += ground[g.idx(i + di, j + dj)];
                                n++;
                            }
                        }
                    }
                    if (n > 0)
                    {
                        newGround[c]   = static_cast<float>(sum / n);
                        newHasValue[c] = 1;
                    }
                }
            }
            ground.swap(newGround);
            hasValue.swap(newHasValue);
        }
    }

    // Never observed cells are filled too, by design, so sparse ground stays
    // continuous, up to ground_fill_distance_unobserved (that is how far the
    // ground extends past edges whose other side was never seen). They take the
    // ground of the nearest cell with ground, instead of an average, so a step
    // reaching the unobserved area is not smoothed into a ramp around its end:
    const int fillStepsUnobserved =
        static_cast<int>(std::floor(p.ground_fill_distance_unobserved / g.res));
    if (fillStepsUnobserved > 0)
    {
        std::vector<uint8_t> hasGround(g.size(), 0);
        for (size_t c = 0; c < g.size(); c++)
        {
            hasGround[c] = std::isnan(ground[c]) ? 0 : 1;
        }
        auto       nearestGround = ground;
        const auto dist          = distanceFrom(g, hasGround, fillStepsUnobserved, &nearestGround);
        for (size_t c = 0; c < g.size(); c++)
        {
            if (!hasGround[c] && !std::isfinite(lowest[c]) && dist[c] <= fillStepsUnobserved)
            {
                ground[c] = nearestGround[c];
            }
        }
    }
    const auto known = [&](size_t c) { return !std::isnan(ground[c]); };

    // 2) Per cell checks ------------------------------------------------------
    std::vector<uint8_t> geomObstacle(g.size(), 0);
    std::vector<uint8_t> bodyObstacle(g.size(), 0);

    const int slopeHalf = std::max(1, static_cast<int>(std::round(p.slope_window_radius / g.res)));
    const float maxSlopeTan = std::tan(mrpt::DEG2RAD(p.max_slope_deg));

    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            const size_t c = g.idx(i, j);
            if (!known(c))
            {
                continue;
            }
            // Step to the neighbors:
            float maxStep = 0;
            for (const auto& n : kNeighbors8)
            {
                const int ii = i + n[0];
                const int jj = j + n[1];
                if (g.inside(ii, jj) && known(g.idx(ii, jj)))
                {
                    maxStep = std::max(maxStep, std::abs(ground[g.idx(ii, jj)] - ground[c]));
                }
            }
            // Plane z = a x + b y + d fitted to the window, centered coordinates:
            double n   = 0;
            double sx  = 0;
            double sy  = 0;
            double sz  = 0;
            double sxx = 0;
            double syy = 0;
            double sxy = 0;
            double sxz = 0;
            double syz = 0;
            for (int dj = -slopeHalf; dj <= slopeHalf; dj++)
            {
                for (int di = -slopeHalf; di <= slopeHalf; di++)
                {
                    if (!g.inside(i + di, j + dj) || !known(g.idx(i + di, j + dj)))
                    {
                        continue;
                    }
                    const double x = static_cast<double>(di) * g.res;
                    const double y = static_cast<double>(dj) * g.res;
                    const double z = ground[g.idx(i + di, j + dj)] - ground[c];
                    n += 1;
                    sx += x;
                    sy += y;
                    sz += z;
                    sxx += x * x;
                    syy += y * y;
                    sxy += x * y;
                    sxz += x * z;
                    syz += y * z;
                }
            }
            bool steep = false;
            if (n >= 5)
            {
                const double cxx = sxx - sx * sx / n;
                const double cyy = syy - sy * sy / n;
                const double cxy = sxy - sx * sy / n;
                const double cxz = sxz - sx * sz / n;
                const double cyz = syz - sy * sz / n;
                const double det = cxx * cyy - cxy * cxy;
                if (std::abs(det) > 1e-12)
                {
                    const double a = (cxz * cyy - cyz * cxy) / det;
                    const double b = (cyz * cxx - cxz * cxy) / det;
                    steep          = std::hypot(a, b) > maxSlopeTan;
                }
            }
            // Steps only count where there is observed data: between never observed
            // cells they are artifacts of the ground fill, where the nearest ground of
            // two cells comes from different places (the reachability still does not
            // cross them).
            bool observedNearby = false;
            for (int dj = -1; dj <= 1 && !observedNearby; dj++)
            {
                for (int di = -1; di <= 1 && !observedNearby; di++)
                {
                    observedNearby =
                        g.inside(i + di, j + dj) && std::isfinite(lowest[g.idx(i + di, j + dj)]);
                }
            }
            geomObstacle[c] = (steep || (observedNearby && maxStep > p.max_step)) ? 1 : 0;
        }
    }

    // Points in the body band above the local ground:
    {
        std::vector<uint32_t> bandCount(g.size(), 0);
        for (size_t k = 0; k < xs.size(); k++)
        {
            const size_t c = cellOfPoint[k];
            if (!known(c))
            {
                continue;
            }
            const float h = zs[k] - ground[c];
            if (h > p.body_min_height && h < p.body_max_height)
            {
                bandCount[c]++;
            }
        }
        for (size_t c = 0; c < g.size(); c++)
        {
            bodyObstacle[c] = bandCount[c] >= p.body_min_points ? 1 : 0;
        }
    }

    std::vector<uint8_t> traversable(g.size(), 0);
    for (size_t c = 0; c < g.size(); c++)
    {
        traversable[c] = (known(c) && !geomObstacle[c] && !bodyObstacle[c]) ? 1 : 0;
    }

    const auto outputCell = [&](mrpt::maps::CPointsMap& out, int i, int j, float z)
    { out.insertPoint(g.centerX(i), g.centerY(j), z); };

    // Without seeds: per cell classification only.
    if (!seedsPc || seedsPc->empty())
    {
        for (int j = 0; j < g.ny; j++)
        {
            for (int i = 0; i < g.nx; i++)
            {
                const size_t c = g.idx(i, j);
                if (outObstacles && known(c) && (geomObstacle[c] || bodyObstacle[c]))
                {
                    outputCell(*outObstacles, i, j, ground[c]);
                }
                if (outTraversable && traversable[c])
                {
                    outputCell(*outTraversable, i, j, ground[c]);
                }
            }
        }
        return;
    }

    // 3) Reachability -------------------------------------------------------
    // Seeds are traversable within seeds_radius:
    std::vector<uint8_t> seeded(g.size(), 0);
    {
        const int   r   = static_cast<int>(std::round(p.seeds_radius / g.res));
        const auto& sxs = seedsPc->getPointsBufferRef_x();
        const auto& sys = seedsPc->getPointsBufferRef_y();
        for (size_t k = 0; k < sxs.size(); k++)
        {
            const int si = g.cx(sxs[k]);
            const int sj = g.cy(sys[k]);
            for (int dj = -r; dj <= r; dj++)
            {
                for (int di = -r; di <= r; di++)
                {
                    if (di * di + dj * dj <= r * r && g.inside(si + di, sj + dj))
                    {
                        seeded[g.idx(si + di, sj + dj)] = 1;
                    }
                }
            }
        }
    }
    for (size_t c = 0; c < g.size(); c++)
    {
        if (seeded[c] && known(c))
        {
            traversable[c]  = 1;
            geomObstacle[c] = 0;
            bodyObstacle[c] = 0;
        }
    }

    // Grow the reachable region from the seeds, without crossing steps:
    std::vector<uint8_t>            reach(g.size(), 0);
    std::deque<std::pair<int, int>> q;
    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            if (seeded[g.idx(i, j)] && traversable[g.idx(i, j)])
            {
                reach[g.idx(i, j)] = 1;
                q.emplace_back(i, j);
            }
        }
    }
    while (!q.empty())
    {
        const auto [i, j] = q.front();
        q.pop_front();
        const size_t c = g.idx(i, j);
        for (const auto& n : kNeighbors8)
        {
            const int ii = i + n[0];
            const int jj = j + n[1];
            if (!g.inside(ii, jj))
            {
                continue;
            }
            const size_t cn = g.idx(ii, jj);
            if (reach[cn] || !traversable[cn])
            {
                continue;
            }
            if (std::abs(ground[cn] - ground[c]) <= p.max_step || (seeded[cn] && seeded[c]))
            {
                reach[cn] = 1;
                q.emplace_back(ii, jj);
            }
        }
    }

    // Obstacles around the reachable region. Unknown cells take the ground height
    // of the nearest reachable cell:
    const int obstacleSteps =
        std::max(1, static_cast<int>(std::round(p.obstacle_distance / g.res)));
    auto       zNearest    = ground;
    const auto distToReach = distanceFrom(g, reach, obstacleSteps, &zNearest);

    size_t nObstacles = 0;
    size_t nReachable = 0;
    for (int j = 0; j < g.ny; j++)
    {
        for (int i = 0; i < g.nx; i++)
        {
            const size_t c = g.idx(i, j);
            if (reach[c])
            {
                nReachable++;
                if (outTraversable)
                {
                    outputCell(*outTraversable, i, j, ground[c]);
                }
                continue;
            }
            const int  d        = distToReach[c];
            const bool isBorder = d == 1 && (known(c) || p.unknown_is_obstacle);
            const bool isNonTraversableNear =
                d <= obstacleSteps && known(c) && (geomObstacle[c] || bodyObstacle[c]);
            if (isBorder || isNonTraversableNear)
            {
                nObstacles++;
                if (outObstacles)
                {
                    outputCell(*outObstacles, i, j, known(c) ? ground[c] : zNearest[c]);
                }
            }
        }
    }

    MRPT_LOG_DEBUG_STREAM(
        "Grid " << g.nx << "x" << g.ny << ", reachable cells: " << nReachable
                << ", obstacle cells: " << nObstacles);

    MRPT_END
}
