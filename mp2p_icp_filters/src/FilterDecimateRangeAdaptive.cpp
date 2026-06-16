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
 * @file   FilterDecimateRangeAdaptive.cpp
 * @brief  EllipseLIO-style range-adaptive voxel decimation (arXiv:2605.21150).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 2026
 */

#include <mp2p_icp/pointcloud_field_utils.h>
#include <mp2p_icp_filters/FilterDecimateRangeAdaptive.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TPoint3D.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <mutex>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterDecimateRangeAdaptive, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateRangeAdaptive::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_OPT(c, vertical_fov_rad);
    MCP_LOAD_OPT(c, num_scan_lines);
    MCP_LOAD_OPT(c, bin_width);
    MCP_LOAD_OPT(c, max_range);
    MCP_LOAD_OPT(c, min_voxel_size);
    MCP_LOAD_OPT(c, max_voxel_size);
    MCP_LOAD_OPT(c, min_input_points_per_voxel);
    MCP_LOAD_OPT(c, parallelization_grain_size);
}

// Simple hash for voxel grid indices using a flat 3D key
struct VoxelKey
{
    int32_t cx, cy, cz;
    bool    operator==(const VoxelKey& o) const { return cx == o.cx && cy == o.cy && cz == o.cz; }
};

struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& k) const
    {
        // FNV-inspired mixing
        size_t h = 2166136261ULL;
        h ^= static_cast<size_t>(k.cx);
        h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.cy);
        h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.cz);
        h *= 1099511628211ULL;
        return h;
    }
};

struct FilterDecimateRangeAdaptive::Impl
{
    // nothing needed — the filter is purely algorithmic per-call
};

FilterDecimateRangeAdaptive::FilterDecimateRangeAdaptive() : impl_(mrpt::make_impl<Impl>())
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateRangeAdaptive");
}

void FilterDecimateRangeAdaptive::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);

    ASSERTMSG_(params.bin_width > 0, "FilterDecimateRangeAdaptive: bin_width must be > 0.");
    ASSERTMSG_(
        params.min_voxel_size <= params.max_voxel_size,
        "FilterDecimateRangeAdaptive: min_voxel_size must be <= max_voxel_size.");

    MRPT_END
}

void FilterDecimateRangeAdaptive::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // --- Input layer ---
    const auto& pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    const auto&  pc = *pcPtr;
    const auto&  xs = pc.getPointsBufferRef_x();
    const auto&  ys = pc.getPointsBufferRef_y();
    const auto&  zs = pc.getPointsBufferRef_z();
    const size_t N  = xs.size();

    // --- Auto-derive sensor geometry from ring channel ---
    const auto* ptrRing = pc.getPointsBufferRef_uint16_field("ring");

    double       fov_rad   = params.vertical_fov_rad;
    unsigned int num_lines = params.num_scan_lines;

    if (ptrRing && !ptrRing->empty() && ptrRing->size() == N)
    {
        if (num_lines == 0)
        {
            // Derive beta from max ring id:
            uint16_t max_ring = 0;
            for (size_t i = 0; i < N; i++)
            {
                if ((*ptrRing)[i] > max_ring)
                {
                    max_ring = (*ptrRing)[i];
                }
            }
            num_lines = static_cast<unsigned int>(max_ring) + 1u;
        }

        if (fov_rad <= 0)
        {
            // Estimate FOV from elevation of all points:
            double el_min = std::numeric_limits<double>::max();
            double el_max = std::numeric_limits<double>::lowest();
            for (size_t i = 0; i < N; i++)
            {
                const double rxy = std::sqrt(
                    static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i]);
                const double el = std::atan2(static_cast<double>(zs[i]), rxy);
                if (el < el_min)
                {
                    el_min = el;
                }
                if (el > el_max)
                {
                    el_max = el;
                }
            }
            fov_rad = el_max - el_min;
        }
    }

    ASSERTMSG_(
        fov_rad > 0,
        "FilterDecimateRangeAdaptive: vertical_fov_rad must be > 0 (or auto-derivable from ring "
        "channel).");
    ASSERTMSG_(
        num_lines >= 2,
        "FilterDecimateRangeAdaptive: num_scan_lines must be >= 2 (or auto-derivable from ring "
        "channel).");

    // --- Find max range ---
    double max_r = params.max_range;
    if (max_r <= 0)
    {
        for (size_t i = 0; i < N; i++)
        {
            const double r = std::sqrt(
                static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                static_cast<double>(zs[i]) * zs[i]);
            if (r > max_r)
            {
                max_r = r;
            }
        }
    }

    // --- Output layer ---
    ASSERT_(!params.output_pointcloud_layer.empty());

    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params.output_pointcloud_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    outPc->registerPointFieldsFrom(pc);
    mrpt::maps::CPointsMap::InsertCtx ctx = outPc->prepareForInsertPointsFrom(pc);
    mp2p_icp::warn_on_field_padding_mismatch(pc, *outPc, *this);

    // --- Radial binning ---
    const double bin_w   = params.bin_width;
    const double theta   = fov_rad;
    const double beta_m1 = static_cast<double>(num_lines - 1u);

    // For each bin index, build a voxel hash of first-point indices.
    // Collect bin indices needed:
    const size_t num_bins =
        (max_r <= 0) ? 1u : (static_cast<size_t>(std::ceil(max_r / bin_w)) + 1u);

    // Per-bin voxel maps: bin_idx -> (VoxelKey -> {count, first_point_index})
    struct VoxelEntry
    {
        size_t count     = 0;
        size_t first_idx = 0;
    };
    std::vector<std::unordered_map<VoxelKey, VoxelEntry, VoxelKeyHash>> bin_voxels(num_bins);

    for (size_t i = 0; i < N; i++)
    {
        const double x = static_cast<double>(xs[i]);
        const double y = static_cast<double>(ys[i]);
        const double z = static_cast<double>(zs[i]);
        const double r = std::sqrt(x * x + y * y + z * z);

        if (params.max_range > 0 && r > params.max_range)
        {
            continue;
        }

        const size_t bin_i = static_cast<size_t>(r / bin_w);
        if (bin_i >= num_bins)
        {
            continue;
        }

        // Eq. 1: v_i = (bin_i + 1) * theta / (beta - 1)
        const double v_raw = static_cast<double>(bin_i + 1u) * theta / beta_m1;
        const double v     = std::clamp(v_raw, params.min_voxel_size, params.max_voxel_size);

        const int32_t cx = static_cast<int32_t>(std::floor(x / v));
        const int32_t cy = static_cast<int32_t>(std::floor(y / v));
        const int32_t cz = static_cast<int32_t>(std::floor(z / v));

        auto [it, inserted] = bin_voxels[bin_i].emplace(VoxelKey{cx, cy, cz}, VoxelEntry{1, i});
        if (!inserted)
        {
            it->second.count++;
        }
    }

    // --- Collect survivors into output (Eq. 3: union of bin outputs) ---
    size_t total_voxels = 0;
    for (const auto& vmap : bin_voxels)
    {
        for (const auto& kv : vmap)
        {
            if (kv.second.count < params.min_input_points_per_voxel)
            {
                continue;
            }
            outPc->insertPointFrom(kv.second.first_idx, ctx);
            total_voxels++;
        }
    }

    outPc->mark_as_modified();

    MRPT_LOG_DEBUG_STREAM(
        "Input=" << N << " output_voxels=" << total_voxels << " fov_rad=" << fov_rad
                 << " num_lines=" << num_lines << " num_bins=" << num_bins);

    MRPT_END
}
