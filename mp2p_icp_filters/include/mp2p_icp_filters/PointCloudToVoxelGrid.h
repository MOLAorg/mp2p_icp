/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGrid.h
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <tsl/robin_map.h>

#include <vector>

/** \ingroup mp2p_icp_filters_grp */
namespace mp2p_icp_filters
{
/** Auxiliary data structure: an index of points in a point cloud, organized by
 *  their 3D position according to a predefined regular-sized voxel grid.
 * \ingroup mp2p_icp_filters_grp
 */
class PointCloudToVoxelGrid
{
   public:
    PointCloudToVoxelGrid() = default;
    ~PointCloudToVoxelGrid() {}

    /** Changes the voxel resolution, clearing past contents */
    void setResolution(const float voxel_size);

    void processPointCloud(const mrpt::maps::CPointsMap& p);

    /** Remove all points and internal data.
     */
    void clear();

    struct Parameters
    {
        /** Minimum distance (infinity norm) between **consecutive** points to
         * be accepted in a voxel. By looking at points in order, this allows
         * for a very fast discrimination of too-close consecutive points
         * without the need to query any KD-tree.
         *
         * (Default=0, i.e. disabled).
         */
        float min_consecutive_distance{.0f};
    };

    Parameters params_;

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::vector<std::size_t> indices;
    };

    struct indices_t
    {
        indices_t(int32_t cx, int32_t cy, int32_t cz)
            : cx_(cx), cy_(cy), cz_(cz)
        {
        }

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
            return ((1 << 20) - 1) &
                   (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        }

        // k1 < k2?
        bool operator()(const indices_t& k1, const indices_t& k2) const noexcept
        {
            if (k1.cx_ != k2.cx_) return k1.cx_ < k2.cx_;
            if (k1.cy_ != k2.cy_) return k1.cy_ < k2.cy_;
            return k1.cz_ < k2.cz_;
        }
    };

    /** The point indices in each voxel. Directly access to each desired cell,
     * use its iterator, etc. */
    tsl::robin_map<indices_t, voxel_t, IndicesHash> pts_voxels;

    inline int32_t coord2idx(float xyz) const
    {
        return static_cast<int32_t>(xyz / resolution_);
    }

   private:
    /** Voxel size (meters) or resolution. */
    float resolution_ = 0.20f;
};

}  // namespace mp2p_icp_filters
