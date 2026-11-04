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
 * @file   IcpCovFeatures.h
 * @brief  Framework-independent evaluation of a registration result, for
 *         learning a calibrated covariance of it.
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mp2p_icp
{
/** Parameters of CovFeatureEvaluator. They define the common cost function
 * every registration result is evaluated with, so changing any of them changes
 * the meaning of the features: they are part of the feature version.
 *
 * \ingroup mp2p_icp_grp
 */
struct CovFeatureParameters
{
    double   map_voxel       = 0.10;  //!< Voxel size of the reference cloud [m]
    double   scan_voxel      = 0.10;  //!< Voxel size of the query scan [m]
    size_t   max_scan_points = 4096;  //!< Query points kept after voxelization
    size_t   probe_points    = 1024;  //!< Prefix of the (shuffled) query used for probing
    double   max_assoc_dist  = 1.0;  //!< Nearest-neighbor association gate [m]
    double   cauchy_scale    = 0.05;  //!< Scale of the Cauchy robust kernel [m]
    size_t   normal_k        = 12;  //!< Neighbors of a map point for its normal
    double   normal_max_dist = 1.0;  //!< Farthest neighbor allowed in a normal fit [m]
    double   max_condition   = 1e9;  //!< Eigenvalue floor of H, relative to the largest
    uint64_t seed            = 1;  //!< Seed of the query subsampling

    /** Probing step sizes, in units of the analytic standard deviation along
     *  each whitened axis. Exactly kNumProbeScales of them. */
    std::vector<double> probe_scales = {1.0, 4.0, 16.0};

    static constexpr size_t kNumProbeScales = 3;
};

/** What the evaluator is told about how a registration result was obtained. */
struct CovFeatureContext
{
    /** Initial guess of the registration (vehicle pose in the map frame). */
    mrpt::poses::CPose3D initialGuess;

    /** Covariance of the initial guess, in its right SE(3) tangent
     *  `[x y z rx ry rz]`, if known. */
    std::optional<Eigen::Matrix<double, 6, 6>> priorCovariance;

    bool   priorInSolver = false;  //!< Whether the solver used it as a prior
    double iterations    = 0;
    bool   converged     = true;
    double quality       = std::numeric_limits<double>::quiet_NaN();
};

/** Output of CovFeatureEvaluator::evaluate(). */
struct CovFeatureResult
{
    /** False when the solution has too few associations to define the cost
     *  locally; then only the residual, scene and context features are
     *  meaningful and the analytic covariance is undefined. */
    bool ok = false;

    /** Gauss-Newton Hessian and gradient of the common cost at the solution,
     *  in the right tangent `[x y z rx ry rz]` of the solution. */
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();

    /** A-posteriori unit-weight variance of the common cost [m^2]. */
    double sigma2 = 0;

    /** Common analytic covariance `sigma2 * H^-1` (condition-limited) and its
     *  lower Cholesky factor, same tangent as H. */
    Eigen::Matrix<double, 6, 6> Sigma_a = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> L_a     = Eigen::Matrix<double, 6, 6>::Zero();

    /** Feature vector, in the order of CovFeatureEvaluator::featureNames(). */
    std::vector<float> features;
};

/** Evaluates a registration result with a cost function of its own, so that
 * results from different ICP implementations and configurations are described
 * by the same numbers.
 *
 * The evaluator owns its reference cloud (voxelized), its query scan
 * (voxelized and subsampled) and a point-to-plane cost with a Cauchy kernel and
 * lazily fitted map normals. At the given solution it computes the analytic
 * covariance of that cost and a feature vector describing the problem's
 * structure, the residuals, the cost's real shape at finite steps along each
 * whitened axis (with correspondences re-searched), the scene geometry, and
 * how the solution relates to its initial guess.
 *
 * Typical use: setReference() once per local map, setScan() once per query
 * scan, then evaluate() once per registration result.
 *
 * \ingroup mp2p_icp_grp
 */
class CovFeatureEvaluator
{
   public:
    explicit CovFeatureEvaluator(const CovFeatureParameters& p = {});

    /** Sets the reference cloud from point clouds and the poses that bring
     *  each of them into the map frame. */
    void setReference(
        const std::vector<std::pair<const mrpt::maps::CPointsMap*, mrpt::poses::CPose3D>>& clouds);

    /** Incremental alternative to setReference(), for a sliding window of
     *  clouds: adds one cloud's points, or removes them (pass the same cloud
     *  and pose it was added with). Call finalizeReference() before the next
     *  evaluate(). Cheaper than setReference() when only the ends of the
     *  window change; the result is the same up to floating-point rounding
     *  and the order of the reference points. */
    void addReferenceCloud(const mrpt::maps::CPointsMap& cloud, const mrpt::poses::CPose3D& pose);
    void removeReferenceCloud(
        const mrpt::maps::CPointsMap& cloud, const mrpt::poses::CPose3D& pose);
    void finalizeReference();

    /** Sets the query scan, in the vehicle frame. */
    void setScan(const mrpt::maps::CPointsMap& scan);

    /** Evaluates the registration result `solution` (vehicle pose in the map
     *  frame). Requires setReference() and setScan(). */
    CovFeatureResult evaluate(const mrpt::poses::CPose3D& solution, const CovFeatureContext& ctx);

    size_t referenceSize() const;
    size_t scanSize() const { return scan_.size(); }

    const CovFeatureParameters& parameters() const { return params_; }

    /** Names of the entries of CovFeatureResult::features. */
    static const std::vector<std::string>& featureNames();

    /** Increased whenever the meaning or order of the features changes. */
    static constexpr int kVersion = 1;

   private:
    CovFeatureParameters params_;

    struct RefVoxel
    {
        double   x = 0;
        double   y = 0;
        double   z = 0;
        uint32_t n = 0;
    };
    std::unordered_map<uint64_t, RefVoxel> refGrid_;  //!< reference voxel sums

    void accumulateReference(
        const mrpt::maps::CPointsMap& cloud, const mrpt::poses::CPose3D& pose, bool add);

    mrpt::maps::CSimplePointsMap::Ptr ref_;
    std::vector<Eigen::Vector3f>      normals_;
    std::vector<Eigen::Vector3f>      shapes_;  //!< planarity, linearity, scattering
    std::vector<uint8_t>              normalState_;  //!< 0 unknown, 1 valid, 2 invalid

    std::vector<Eigen::Vector3d> scan_;  //!< shuffled, vehicle frame

    struct Assoc
    {
        bool            ok     = false;
        double          r      = 0;  //!< point-to-plane residual [m]
        double          d      = 0;  //!< point-to-point distance [m]
        size_t          mapIdx = 0;
        Eigen::Vector3d n      = Eigen::Vector3d::Zero();  //!< map-frame normal
    };

    bool   mapNormal(size_t idx);
    Assoc  associate(const Eigen::Vector3d& q);
    double cost(const mrpt::poses::CPose3D& pose, size_t count);
};

}  // namespace mp2p_icp
