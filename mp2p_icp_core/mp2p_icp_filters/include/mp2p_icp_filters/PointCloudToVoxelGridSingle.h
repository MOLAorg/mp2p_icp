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
 * @file   PointCloudToVoxelGridSingle.h
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

#include <cstdint>
#include <vector>

/** \ingroup mp2p_icp_filters_grp */
namespace mp2p_icp_filters
{
/** Like PointCloudToVoxelGrid, but hardcoded to only store one single point per
 * voxel.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class PointCloudToVoxelGridSingle
{
   public:
    PointCloudToVoxelGridSingle();

    /** Changes the voxel settings, clearing past contents */
    void setConfiguration(const float voxel_size, bool use_tsl_robin_map = true);

    void processPointCloud(
        const mrpt::maps::CPointsMap& p, const std::size_t first_pt_idx = 0,
        const std::size_t points_to_process = 0);

    /** Remove all points and internal data.
     */
    void clear();

    /** The single point kept for a voxel.
     *
     *  The fields are stored bare, rather than wrapped in std::optional, since
     *  this struct is the value type of the hash map and is therefore touched
     *  once per input point: the optionals made it 56 bytes, which is what the
     *  per-point cost of this grid is dominated by. `pointCount == 0` marks an
     *  empty voxel, so no separate "has value" flag is needed.
     */
    struct voxel_t
    {
        mrpt::math::TPoint3Df point = {0, 0, 0};

        /** Index of `point` within the source cloud. */
        uint32_t pointIdx = 0;

        /** Even if we keep the first point only, count them all.
         *  Zero means the voxel holds no point yet. */
        uint32_t pointCount = 0;

        /** Which of the clouds passed to processPointCloud() `pointIdx`
         *  refers to; resolve it with sourceCloud(). */
        uint16_t sourceIdx = 0;
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

            // Keep all 32 bits: masking to fewer bits bounds the number of
            // distinct buckets, which makes open-addressing maps grow without
            // limit once they hold that many keys.
            const uint32_t* vec = reinterpret_cast<const uint32_t*>(&k);
            return vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791;
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

    /** Resolves voxel_t::sourceIdx into the cloud it refers to. */
    const mrpt::maps::CPointsMap* sourceCloud(uint16_t sourceIdx) const;

   private:
    /** Voxel size (meters) or resolution. */
    float resolution_ = 0.20f;

    bool use_tsl_robin_map_ = true;

    /** The clouds seen by processPointCloud() since the last clear(), in call
     *  order. Voxels store an index into this list instead of a pointer, which
     *  keeps voxel_t small. */
    std::vector<const mrpt::maps::CPointsMap*> sources_;

    /** The actual hash map. Hidden inside a PIMP to prevent problems with
     * duplicated TSL library copies in the user space */
    struct Impl;
    mrpt::pimpl<Impl> impl_;
};

}  // namespace mp2p_icp_filters
