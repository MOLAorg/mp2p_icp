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
 * @file   FilterDecimateAdaptive.cpp
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#include <mp2p_icp/pointcloud_field_utils.h>
#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/version.h>

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(MP2P_HAS_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterDecimateAdaptive, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateAdaptive::OutputTarget::load_from_yaml(
    const mrpt::containers::yaml& c, FilterDecimateAdaptive& parent)
{
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    DECLARE_PARAMETER_IN_REQ(c, desired_output_point_count, parent);
    DECLARE_PARAMETER_IN_OPT(c, maximum_voxel_stride, parent);
}

void FilterDecimateAdaptive::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterDecimateAdaptive& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    outputs.clear();

    if (c.has("outputs"))
    {
        ASSERTMSG_(
            !c.has("output_pointcloud_layer") && !c.has("desired_output_point_count"),
            "Use either the 'outputs' sequence, or the single-output keys "
            "'output_pointcloud_layer'/'desired_output_point_count', but not both.");

        auto cfgOutputs = c["outputs"];
        ASSERTMSG_(cfgOutputs.isSequence(), "'outputs' must be a YAML sequence.");
        ASSERTMSG_(!cfgOutputs.asSequence().empty(), "'outputs' cannot be an empty sequence.");

        // Sized up front, never grown: declaring a dynamic parameter stores a
        // pointer to the target field, which a reallocation would dangle.
        outputs.resize(cfgOutputs.asSequence().size());

        size_t idx = 0;
        for (const auto& entry : cfgOutputs.asSequence())
        {
            outputs.at(idx++).load_from_yaml(entry, parent);
        }
    }
    else
    {
        outputs.resize(1);
        outputs.front().load_from_yaml(c, parent);
    }

    DECLARE_PARAMETER_IN_OPT(c, voxel_size, parent);
    MCP_LOAD_OPT(c, minimum_input_points_per_voxel);
    MCP_LOAD_OPT(c, parallelization_grain_size);
    MCP_LOAD_OPT(c, decimate_method);
}

struct FilterDecimateAdaptive::Impl
{
#if defined(MP2P_HAS_TBB)
    tbb::enumerable_thread_specific<PointCloudToVoxelGrid> tls;

    /// Where the per-thread grids of `tls` are reassembled. Owning the merged
    /// voxels here is what keeps voxel_t a plain non-owning span.
    PointCloudToVoxelGrid merged_grid;

    /// Reusable list of the `tls` grids to merge, to avoid reallocating it.
    std::vector<const PointCloudToVoxelGrid*> parts;
#else
    PointCloudToVoxelGrid filter_grid;
#endif
};

FilterDecimateAdaptive::FilterDecimateAdaptive() : impl_(mrpt::make_impl<Impl>())
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateAdaptive");
}

void FilterDecimateAdaptive::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterDecimateAdaptive::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // sanity checks:
    ASSERT_(!params.outputs.empty());
    for (const auto& o : params.outputs)
    {
        ASSERT_GT_(o.desired_output_point_count, 0);
    }
    ASSERT_GT_(params.voxel_size, 0);

    // In:
    ASSERTMSG_(
        inOut.layers.count(params.input_pointcloud_layer) != 0,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    auto pcPtr = mp2p_icp::MapToPointsMap(*inOut.layers.at(params.input_pointcloud_layer));
    if (!pcPtr)
    {
        THROW_EXCEPTION_FMT(
            "Layer '%s' must be of point cloud type.", params.input_pointcloud_layer.c_str());
    }

    const auto& pc = *pcPtr;

    const auto& _ = params;  // shortcut

    struct DataPerVoxel
    {
        // voxel_t is a non-owning span; copy by value so we don't hold a
        // dangling pointer to the temporary built inside visit_voxels.
        PointCloudToVoxelGrid::voxel_t voxel;
        uint32_t                       nextIdx   = 0;
        bool                           exhausted = false;

        /// Where to start taking points from within the voxel (!=0 only for
        /// DecimateMethod::RandomPoint).
        uint32_t startOffset = 0;

        /// Representative of the whole voxel, precomputed once since it does
        /// not depend on the output target. Only for the two average-based
        /// decimation methods: an index into the input cloud for
        /// ClosestToAverage, the average point itself for VoxelAverage.
        uint32_t              representativeIdx = 0;
        mrpt::math::TPoint3Df average           = {0, 0, 0};
    };

    // A list of all "valid" voxels:
    std::vector<DataPerVoxel> voxels;

    std::size_t nTotalVoxels = 0;

    const auto lambdaVisitVoxel =
        [&](const PointCloudToVoxelGrid::indices_t&, const PointCloudToVoxelGrid::voxel_t& data)
    {
        if (!data.empty())
        {
            nTotalVoxels++;
        }
        if (data.size() < _.minimum_input_points_per_voxel)
        {
            return;
        }

        voxels.emplace_back().voxel = data;
    };

    // Parse input cloud through subsampling:
#if defined(MP2P_HAS_TBB)

    // Clear from past runs:
    for (auto& grid : impl_->tls)
    {
        grid.setConfiguration(params.voxel_size, true);
    }

    const auto pointCount = pc.size();

    const size_t grainsize = params.parallelization_grain_size;  // minimum block size hint

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, pointCount, grainsize),
        [&](const tbb::blocked_range<size_t>& r)
        {
            const auto start  = r.begin();
            const auto length = r.end() - r.begin();

            bool  tls_exists;
            auto& grid = impl_->tls.local(tls_exists);
            if (!tls_exists)
            {
                // Configure for first time:
                grid.setConfiguration(params.voxel_size, true);
            }
            grid.processPointCloud(pc, start, length);
        });

    // Reassemble the per-thread grids: a voxel key is global, so the same
    // spatial voxel is present in every grid whose block saw points in it.
    // Visiting the grids separately would treat each fragment as an
    // independent voxel, which both breaks the one-representative-per-voxel
    // contract and makes the result depend on how work was split.
    impl_->merged_grid.setConfiguration(params.voxel_size, true);

    impl_->parts.clear();
    for (const auto& grid : impl_->tls)
    {
        impl_->parts.push_back(&grid);
    }
    impl_->merged_grid.mergeFrom(impl_->parts);
    impl_->merged_grid.sortVoxelPointIndices();

    voxels.reserve(impl_->merged_grid.size());

    impl_->merged_grid.visit_voxels(lambdaVisitVoxel);

#else
    impl_->filter_grid.clear();
    impl_->filter_grid.setConfiguration(params.voxel_size, true);
    impl_->filter_grid.processPointCloud(pc);

    voxels.reserve(impl_->filter_grid.size());

    impl_->filter_grid.visit_voxels(lambdaVisitVoxel);

#endif

    // Canonical voxel order. Hash map traversal order is an implementation
    // detail, and the resampling below walks this list with a stride, so which
    // voxels make it to the output would otherwise depend on it. Ordering by
    // the first input point of each voxel gives the same list for any map type
    // and any number of threads.
    std::sort(
        voxels.begin(), voxels.end(),
        [](const DataPerVoxel& a, const DataPerVoxel& b) { return a.voxel[0] < b.voxel[0]; });

    const size_t nVoxels = voxels.size();

    // Precompute, once for all output targets, whatever the selected
    // decimation method needs per voxel:
    // ----------------------------------------------------------------
    const bool usesAverage = _.decimate_method == DecimateMethod::ClosestToAverage ||
                             _.decimate_method == DecimateMethod::VoxelAverage;

    if (_.decimate_method == DecimateMethod::RandomPoint)
    {
        auto rng = mrpt::random::CRandomGenerator();

        for (auto& v : voxels)
        {
            v.startOffset = static_cast<uint32_t>(rng.drawUniform64bit() % v.voxel.size());
        }
    }
    else if (usesAverage)
    {
        const auto& xs = pc.getPointsBufferRef_x();
        const auto& ys = pc.getPointsBufferRef_y();
        const auto& zs = pc.getPointsBufferRef_z();

        for (auto& v : voxels)
        {
            const float inv_n = 1.0f / static_cast<float>(v.voxel.size());

            auto mean = mrpt::math::TPoint3Df(0, 0, 0);
            for (size_t i = 0; i < v.voxel.size(); i++)
            {
                const auto ptIdx = v.voxel[i];
                mean.x += xs[ptIdx];
                mean.y += ys[ptIdx];
                mean.z += zs[ptIdx];
            }
            mean *= inv_n;
            v.average = mean;

            if (_.decimate_method != DecimateMethod::ClosestToAverage)
            {
                continue;
            }

            std::optional<float> minSqrErr;
            for (size_t i = 0; i < v.voxel.size(); i++)
            {
                const auto  ptIdx  = v.voxel[i];
                const float sqrErr = mrpt::square(xs[ptIdx] - mean.x) +
                                     mrpt::square(ys[ptIdx] - mean.y) +
                                     mrpt::square(zs[ptIdx] - mean.z);

                if (!minSqrErr.has_value() || sqrErr < *minSqrErr)
                {
                    minSqrErr           = sqrErr;
                    v.representativeIdx = ptIdx;
                }
            }
        }
    }

    // Perform resampling:
    // -------------------
    // Note that all requested output layers are sampled from this same, single
    // voxelization pass above (the dominant cost), each one walking the voxel
    // list independently with its own stride.
    constexpr int FRACTIONARY_BIT_COUNT = 12;

    if (nVoxels == 0)
    {
        MRPT_LOG_WARN("FilterDecimateAdaptive: No occupied voxels. Outputs will be empty.");
    }

    for (const auto& target : _.outputs)
    {
        // Bound the sampling ratio, if asked to: the absolute count then acts
        // as a floor, so this can only add coverage, never remove it.
        uint32_t pointCountTarget = target.desired_output_point_count;
        if (target.maximum_voxel_stride > 0)
        {
            // The input is the hard ceiling: no method can emit more points
            // than it was given. Clamping there also keeps an arbitrarily
            // small stride from overflowing the conversion below. Strides
            // under 1 are legitimate for the methods that can take several
            // points out of one voxel, so they are not rejected.
            const double cap = static_cast<double>(
                std::min<std::size_t>(pc.size(), std::numeric_limits<uint32_t>::max()));
            const double wanted =
                std::ceil(static_cast<double>(nVoxels) / target.maximum_voxel_stride);

            pointCountTarget =
                std::max(pointCountTarget, static_cast<uint32_t>(std::min(wanted, cap)));
        }

        // Create if new: Append to existing layer, if already existed.
        mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
            inOut, target.output_pointcloud_layer,
            /*do not allow empty*/
            false,
            /* create cloud of the same type */
            pcPtr->GetRuntimeClass()->className);

        const size_t sizeBefore = outPc->size();

        outPc->reserve(sizeBefore + pointCountTarget);
        outPc->registerPointFieldsFrom(pc);

        if (nVoxels == 0)
        {
            continue;  // Avoid division by zero below; layer is left as created.
        }

        mrpt::maps::CPointsMap::InsertCtx ctx = outPc->prepareForInsertPointsFrom(pc);
        mp2p_icp::warn_on_field_padding_mismatch(pc, *outPc, *this);

        // Each output starts its own walk over the shared voxel list:
        for (auto& v : voxels)
        {
            v.nextIdx   = 0;
            v.exhausted = false;
        }

        float voxelIdxIncrement = 1.0f;
        if (nVoxels > pointCountTarget)
        {
            voxelIdxIncrement = static_cast<float>(nVoxels) / static_cast<float>(pointCountTarget);
        }

        const auto voxelIdxIncrement_frac =
            static_cast<std::size_t>(voxelIdxIncrement * (1 << FRACTIONARY_BIT_COUNT));

        bool anyInsertInTheRound = false;

        std::size_t i_frac = 0;
        while (outPc->size() < pointCountTarget)
        {
            std::size_t i = i_frac >> FRACTIONARY_BIT_COUNT;

            if (i >= nVoxels)
            {
                i_frac = i_frac % (nVoxels << FRACTIONARY_BIT_COUNT);
                i      = i_frac >> FRACTIONARY_BIT_COUNT;

                if (!anyInsertInTheRound)
                {
                    // This means there is no more points and we must end
                    // despite we didn't reached the user's desired number of
                    // points:
                    break;
                }

                anyInsertInTheRound = false;
            }

            auto& ith = voxels[i];
            if (!ith.exhausted)
            {
                if (usesAverage)
                {
                    // These two methods summarize the whole voxel, so each one
                    // can only ever emit one point:
                    if (_.decimate_method == DecimateMethod::VoxelAverage)
                    {
                        outPc->insertPointFast(ith.average.x, ith.average.y, ith.average.z);
                    }
                    else
                    {
                        outPc->insertPointFrom(ith.representativeIdx, ctx);
                    }
                    ith.exhausted = true;
                }
                else
                {
                    // Take successive points out of the voxel, starting at
                    // startOffset (0 unless picking at random):
                    const auto k = (ith.startOffset + ith.nextIdx++) % ith.voxel.size();

                    outPc->insertPointFrom(ith.voxel[k], ctx);

                    if (ith.nextIdx >= ith.voxel.size())
                    {
                        ith.exhausted = true;
                    }
                }
                anyInsertInTheRound = true;
            }

            i_frac += voxelIdxIncrement_frac;
        }

        // A stride above 1 means the budget is binding: only one in
        // `voxelIdxIncrement` occupied cells is ever visited, so coverage is a
        // stride-shaped subsample of the scene rather than the whole of it.
        // Reported so a configuration can be checked against its input instead
        // of assumed.
        MRPT_LOG_DEBUG_FMT(
            "output '%s': nVoxels=%zu budget=%u stride=%.2f emitted=%zu",
            target.output_pointcloud_layer.c_str(), nVoxels, pointCountTarget, voxelIdxIncrement,
            outPc->size() - sizeBefore);
    }

    MRPT_LOG_DEBUG_STREAM("used voxels=" << nTotalVoxels);

    MRPT_END
}
