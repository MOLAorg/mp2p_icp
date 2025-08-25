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
 * @file   FilterPoleDetector.cpp
 * @brief  Leaves or removes points that seem to belong to vertical structures
 * @author Jose Luis Blanco Claraco
 * @date   Jan 27, 2025
 */

#include <mp2p_icp_filters/FilterPoleDetector.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>

#include <unordered_map>

IMPLEMENTS_MRPT_OBJECT(FilterPoleDetector, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

namespace
{
/** Discrete index type for voxel or 2D grid maps, suitable for std::map and
 * std::unordered_map, using mola::index3d_hash as hash type.
 */
template <typename cell_coord_t = int32_t>
struct index2d_t
{
    index2d_t() = default;

    index2d_t(cell_coord_t Cx, cell_coord_t Cy) noexcept : cx(Cx), cy(Cy) {}

    cell_coord_t cx = 0, cy = 0;

    bool operator==(const index2d_t<cell_coord_t>& o) const noexcept
    {
        return cx == o.cx && cy == o.cy;
    }
    bool operator!=(const index2d_t<cell_coord_t>& o) const noexcept { return !operator==(o); }
    index2d_t<cell_coord_t> operator+(const index2d_t<cell_coord_t>& o) const noexcept
    {
        return {cx + o.cx, cy + o.cy};
    }
};

index2d_t<> xy_to_index(float x, float y, float resolution)
{
    return {static_cast<int32_t>(x / resolution), static_cast<int32_t>(y / resolution)};
}

/** This implement the optimized hash from this paper (adapted for 2D)
 *
 *  Teschner, M., Heidelberger, B., Müller, M., Pomerantes, D., & Gross, M. H.
 * (2003, November). Optimized spatial hashing for collision detection of
 * deformable objects. In Vmv (Vol. 3, pp. 47-54).
 *
 */
template <typename cell_coord_t = int32_t>
struct index2d_hash
{
    /// Hash operator for unordered maps:
    std::size_t operator()(const index2d_t<cell_coord_t>& k) const noexcept
    {
        // These are the implicit assumptions of the reinterpret cast below:
        static_assert(sizeof(cell_coord_t) == sizeof(uint32_t));
        static_assert(offsetof(index2d_t<cell_coord_t>, cx) == 0 * sizeof(uint32_t));
        static_assert(offsetof(index2d_t<cell_coord_t>, cy) == 1 * sizeof(uint32_t));

        const uint32_t* vec = reinterpret_cast<const uint32_t*>(&k);
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663);
    }

    /// k1 < k2? for std::map containers
    bool operator()(
        const index2d_t<cell_coord_t>& k1, const index2d_t<cell_coord_t>& k2) const noexcept
    {
        if (k1.cx != k2.cx) return k1.cx < k2.cx;
        return k1.cy < k2.cy;
    }
};

struct GridCellData
{
    GridCellData() = default;

    std::optional<float> minZ, maxZ;
    float                sumZ = 0;
    std::vector<size_t>  point_indices;

    float mean() const { return point_indices.empty() ? .0f : sumZ / point_indices.size(); }
};

using grid2d_map_t = std::unordered_map<index2d_t<int32_t>, GridCellData, index2d_hash<int32_t>>;

}  // namespace

using namespace mp2p_icp_filters;

void FilterPoleDetector::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterPoleDetector& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, output_layer_poles);
    MCP_LOAD_OPT(c, output_layer_no_poles);
    MCP_LOAD_OPT(c, minimum_pole_points);
    MCP_LOAD_OPT(c, minimum_neighbors_checks_to_pass);
    DECLARE_PARAMETER_IN_REQ(c, grid_size, parent);
    DECLARE_PARAMETER_IN_REQ(c, minimum_relative_height, parent);
    DECLARE_PARAMETER_IN_REQ(c, maximum_relative_height, parent);

    ASSERTMSG_(
        !output_layer_poles.empty() || !output_layer_no_poles.empty(),
        "At least one 'output_layer_poles' or "
        "'output_layer_no_poles' must be provided.");
}

FilterPoleDetector::FilterPoleDetector()
{
    mrpt::system::COutputLogger::setLoggerName("FilterPoleDetector");
}

void FilterPoleDetector::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c, *this);

    MRPT_END
}

void FilterPoleDetector::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    const auto pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPoles = GetOrCreatePointLayer(
        inOut, params_.output_layer_poles, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outPoles) outPoles->reserve(outPoles->size() + pc.size() / 10);

    // Optional output layer:
    mrpt::maps::CPointsMap::Ptr outNoPoles = GetOrCreatePointLayer(
        inOut, params_.output_layer_no_poles, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outNoPoles) outNoPoles->reserve(outNoPoles->size() + pc.size() / 10);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    // 1st pass: build grid with stats
    grid2d_map_t grid;

    for (size_t i = 0; i < xs.size(); i++)
    {
        const auto z = zs[i];

        const auto idxs = xy_to_index(xs[i], ys[i], params_.grid_size);
        auto&      cell = grid[idxs];
        cell.sumZ += z;
        cell.point_indices.push_back(i);
        if (!cell.maxZ || z > *cell.maxZ) cell.maxZ = z;
        if (!cell.minZ || z < *cell.minZ) cell.minZ = z;
    }

    // 2nd pass: classify pts
    for (const auto& [idxs, cell] : grid)
    {
        const std::array<index2d_t<>, 8> neighbors = {
            idxs + index2d_t<>(-1, -1), idxs + index2d_t<>(-1, 0), idxs + index2d_t<>(-1, +1),
            idxs + index2d_t<>(0, -1),  idxs + index2d_t<>(0, +1), idxs + index2d_t<>(+1, -1),
            idxs + index2d_t<>(+1, 0),  idxs + index2d_t<>(+1, +1)};

        // Criteria: my mean must be > than most neighbor:
        if (cell.point_indices.size() < params_.minimum_pole_points) continue;
        const float my_mean          = cell.mean();
        size_t      check_pass_count = 0;
        for (const auto& neig_idx : neighbors)
        {
            const auto it = grid.find(neig_idx);
            if (it == grid.end()) continue;
            const auto& c        = it->second;
            const float its_mean = c.mean();
            if (my_mean > its_mean + params_.minimum_relative_height &&
                my_mean < its_mean + params_.maximum_relative_height)
                check_pass_count++;
        }

        const bool isPole = check_pass_count >= params_.minimum_neighbors_checks_to_pass;

        auto* targetPc = isPole ? outPoles.get() : outNoPoles.get();
        if (targetPc)
        {
            for (const auto ptIdx : cell.point_indices) targetPc->insertPointFrom(pc, ptIdx);
        }
    }

    MRPT_END
}
