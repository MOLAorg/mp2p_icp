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
 * @file   PointCloudToVoxelGridAverage.h
 * @brief  Voxel grid that summarizes each voxel without keeping its points.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2026
 */

#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

#include <cstdint>
#include <functional>
#include <vector>

/** \ingroup mp2p_icp_filters_grp */
namespace mp2p_icp_filters
{
/** Like PointCloudToVoxelGrid, but for the decimation methods that only need a
 *  summary of each voxel: its point count, the average of its points, and
 *  optionally the point closest to that average.
 *
 *  PointCloudToVoxelGrid has to keep the index list of every voxel, which costs
 *  a second hashed pass over the cloud, a relayout of the whole index array,
 *  and a random-access gather of x/y/z when those lists are walked. None of
 *  that is needed here: one hashed pass accumulates the sums, and a second
 *  linear pass (hash-free, since the first pass records the voxel of each
 *  point) finds the closest point to the resulting average. Both passes read
 *  the coordinate buffers sequentially.
 *
 *  Results are identical to summarizing PointCloudToVoxelGrid's index lists,
 *  including floating-point rounding: the per-voxel sum is accumulated in the
 *  same ascending point order.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class PointCloudToVoxelGridAverage
{
   public:
    PointCloudToVoxelGridAverage();

    /** Changes the voxel settings, clearing past contents */
    void setConfiguration(const float voxel_size, bool use_tsl_robin_map = true);

    /** Bins one cloud and summarizes each voxel.
     *
     *  \param findClosestToAverage If false, voxel_t::closestToAverageIdx is
     *         left undefined and the second pass is skipped altogether.
     */
    void processPointCloud(const mrpt::maps::CPointsMap& p, bool findClosestToAverage);

    /** Remove all points and internal data. */
    void clear();

    /** The summary of one voxel. */
    struct voxel_t
    {
        /** Average of all the points that fell into this voxel. */
        mrpt::math::TPoint3Df average = {0, 0, 0};

        /** Index, within the processed cloud, of the point closest to
         *  `average`. Only valid if processPointCloud() was asked for it. */
        uint32_t closestToAverageIdx = 0;

        /** How many points fell into this voxel. */
        uint32_t pointCount = 0;
    };

    struct indices_t
    {
        indices_t(int32_t cx, int32_t cy, int32_t cz) : cx_(cx), cy_(cy), cz_(cz) {}

        int32_t cx_ = 0, cy_ = 0, cz_ = 0;

        bool operator==(const indices_t& o) const
        {
            return cx_ == o.cx_ && cy_ == o.cy_ && cz_ == o.cz_;
        }
    };

    /** This implements the optimized hash from this paper:
     *
     *  Teschner, M., Heidelberger, B., Müller, M., Pomerantes, D., & Gross, M.
     * H. (2003, November). Optimized spatial hashing for collision detection of
     * deformable objects. In Vmv (Vol. 3, pp. 47-54).
     *
     */
    struct IndicesHash
    {
        /// Hash operator for unordered maps:
        std::size_t operator()(const indices_t& k) const noexcept
        {
            // These are the implicit assumptions of the reinterpret cast below:
            static_assert(sizeof(indices_t::cx_) == sizeof(uint32_t));
            static_assert(offsetof(indices_t, cx_) == 0 * sizeof(uint32_t));
            static_assert(offsetof(indices_t, cy_) == 1 * sizeof(uint32_t));
            static_assert(offsetof(indices_t, cz_) == 2 * sizeof(uint32_t));

            const uint32_t* vec = reinterpret_cast<const uint32_t*>(&k);
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        }

        // k1 < k2?
        bool operator()(const indices_t& k1, const indices_t& k2) const noexcept
        {
            if (k1.cx_ != k2.cx_)
            {
                return k1.cx_ < k2.cx_;
            }
            if (k1.cy_ != k2.cy_)
            {
                return k1.cy_ < k2.cy_;
            }
            return k1.cz_ < k2.cz_;
        }
    };

    inline int32_t coord2idx(float xyz) const { return static_cast<int32_t>(xyz / resolution_); }

    void visit_voxels(
        const std::function<void(const indices_t idx, const voxel_t& vxl)>& userCode) const;

    /// Returns the number of occupied voxels.
    size_t size() const;

   private:
    /** Voxel size (meters) or resolution. */
    float resolution_ = 0.20f;

    bool use_tsl_robin_map_ = true;

    /** The actual hash map. Hidden inside a PIMP to prevent problems with
     * duplicated TSL library copies in the user space */
    struct Impl;
    mrpt::pimpl<Impl> impl_;
};

}  // namespace mp2p_icp_filters
