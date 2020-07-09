/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_Base.h
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */
#pragma once

#include <mp2p_icp/Matcher.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdlib>
#include <limits>  // std::numeric_limits
#include <optional>
#include <vector>

namespace mp2p_icp
{
/** Pointcloud matcher auxiliary class for iterating over point layers.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_Base : public Matcher
{
   public:
    Matcher_Points_Base() = default;

    /** Weights for each layer. If empty, the output Pairings::point_weights
     * will left empty (=all points have equal weight).
     * \note Note: this field can be loaded from a configuration file via
     * initializeLayerWeights().
     */
    std::map<std::string, double> weight_pt2pt_layers;

    uint64_t maxLocalPointsPerLayer_ = 0;
    uint64_t localPointsSampleSeed_  = 0;

    /** Common parameters to all derived classes:
     *
     * - `maxLocalPointsPerLayer`: Maximum number of local points to consider
     * for the "local" point cloud, per point layer. "0" means "all" (no
     * decimation) [Default=0].
     *
     * - `localPointsSampleSeed`: Only if `maxLocalPointsPerLayer`!=0, and the
     * number of points in the local map is larger than that number, a seed for
     * the RNG used to pick random point indices. `0` (default) means to use a
     * time-based seed.
     *
     * - `pointLayerWeights`: Optional map of layer names to relative weights.
     */
    void initialize(const mrpt::containers::Parameters& params) override;

    /** the output of transform_local_to_global() */
    struct TransformedLocalPointCloud
    {
       public:
        mrpt::math::TPoint3Df localMin{fMax, fMax, fMax};
        mrpt::math::TPoint3Df localMax{-fMax, -fMax, -fMax};

        /** Reordering indexes, used only if we had to pick random indexes */
        std::optional<std::vector<std::size_t>> idxs;

        /** Transformed local points: all, or a random subset */
        mrpt::aligned_std_vector<float> x_locals, y_locals, z_locals;

       private:
        static constexpr auto fMax = std::numeric_limits<float>::max();
    };

    static TransformedLocalPointCloud transform_local_to_global(
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D&   localPose,
        const std::size_t             maxLocalPoints        = 0,
        const uint64_t                localPointsSampleSeed = 0);

   protected:
    void impl_match(
        const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
        const mrpt::poses::CPose3D& localPose, const MatchContext& mc,
        Pairings& out) const override final;

   private:
    virtual void implMatchOneLayer(
        const mrpt::maps::CPointsMap& pcGlobal,
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, Pairings& out) const = 0;
};

}  // namespace mp2p_icp
