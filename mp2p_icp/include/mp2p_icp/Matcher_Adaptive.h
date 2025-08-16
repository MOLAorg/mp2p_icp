/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Adaptive.h
 * @brief  Pointcloud matcher: smart adaptive matcher
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2023
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>
#include <mrpt/containers/vector_with_small_size_optimization.h>

namespace mp2p_icp
{
/** Pointcloud matcher: adaptive algorithm automatically discarding outliers,
 *  and creating point-to-plane, point-to-line, or point-to-point pairings.
 *
 * Finds pairings between the `local` and `global` input metric maps.
 *
 * By default, each `local` point layer is matched against the layer with the
 * same name in the `global` map, unless specified otherwise in the base class
 * member `weight_pt2pt_layers`. Refer to example configuration YAML files for
 * example configurations.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Adaptive : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Adaptive, mp2p_icp)

   public:
    Matcher_Adaptive() = default;

    Matcher_Adaptive(
        double ConfidenceInterval, double FirstToSecondDistanceMax,
        double AbsoluteMaxSearchDistance)
        : Matcher_Points_Base(),
          confidenceInterval(ConfidenceInterval),
          firstToSecondDistanceMax(FirstToSecondDistanceMax),
          absoluteMaxSearchDistance(AbsoluteMaxSearchDistance)
    {
    }

    /** Parameters:
     * - `confidenceInterval`: Inliers top-confidence interval. (0-1)
     * - `firstToSecondDistanceMax`:
     * - `absoluteMaxSearchDistance`:
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    double         confidenceInterval        = 0.80;
    double         firstToSecondDistanceMax  = 1.2;
    mutable double absoluteMaxSearchDistance = 5.0;  // m
    bool           enableDetectPlanes        = false;
    uint32_t       maxPt2PtCorrespondences   = 1;
    uint32_t       planeSearchPoints         = 8;
    uint32_t       planeMinimumFoundPoints   = 4;
    double         planeMinimumDistance      = 0.10;
    double         planeEigenThreshold       = 0.01;
    double         minimumCorrDist           = 0.1;  // m

    // Declared here to avoid memory reallocations:
    mutable std::vector<uint64_t>              neighborIndices_;
    mutable std::vector<float>                 neighborSqrDists_;
    mutable std::vector<mrpt::math::TPoint3Df> neighborPts_;
    mutable std::vector<float>                 kddXs, kddYs, kddZs;
    mutable std::vector<double>                histXs_, histValues_;

    constexpr static size_t MAX_CORRS_PER_LOCAL = 10;

    mutable std::vector<mrpt::containers::vector_with_small_size_optimization<
        mrpt::tfest::TMatchingPair, MAX_CORRS_PER_LOCAL>>
        matchesPerLocal_;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
