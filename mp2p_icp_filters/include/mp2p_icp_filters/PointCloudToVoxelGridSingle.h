/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGridSingle.h
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

#include <optional>

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
    ~PointCloudToVoxelGridSingle() {}

    /** Changes the voxel resolution, clearing past contents */
    void setResolution(const float voxel_size);

    void processPointCloud(const mrpt::maps::CPointsMap& p);

    /** Remove all points and internal data.
     */
    void clear();

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::optional<mrpt::math::TPoint3Df>         point;
        std::optional<size_t>                        pointIdx;  // Index in the source
        std::optional<const mrpt::maps::CPointsMap*> source;

        /** Even if we keep the first point only, count them all. */
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
            if (k1.cx_ != k2.cx_) return k1.cx_ < k2.cx_;
            if (k1.cy_ != k2.cy_) return k1.cy_ < k2.cy_;
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

    /** The actual hash map. Hidden inside a PIMP to prevent problems with
     * duplicated TSL library copies in the user space */
    struct Impl;
    mrpt::pimpl<Impl> impl_;
};

}  // namespace mp2p_icp_filters
