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
 * @file   IncrementCovarianceModel.h
 * @brief  Learned covariance of the pose increments of a LiDAR odometry.
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** Learned covariance of each pose increment of a scan-to-map LiDAR
 * odometry, i.e. of the motion from the previous registered scan to the
 * current one, as a state estimator chaining one relative factor per scan
 * needs it.
 *
 * The axes of the covariance are the eigenvectors of the pairing information
 * of the registration (Results::measurement_information, priors excluded),
 * with rotations scaled by a length L so that meters and radians are
 * commensurate. A small network, shared by the six axes, predicts how fast
 * the variance along each axis grows with the motion of the increment
 * (distance plus L times the rotated angle), from that axis' information and
 * the registration and motion of the scan. Floors are added for the
 * translation and rotation. Everything (features, normalization, layers,
 * activations, floors) comes from a YAML model file written by the training
 * scripts, so models can be retrained and replaced without recompiling.
 *
 * A model is valid only for the platform, sensor and odometry settings it
 * was trained with, which its file records.
 *
 * \ingroup mp2p_icp_grp
 */
class IncrementCovarianceModel
{
   public:
    /** What the model needs from one registration and its increment. */
    struct Input
    {
        /** Results::measurement_information of the registration: SE(3)
         *  right tangent of the solution, translation first. */
        mrpt::math::CMatrixDouble66 measurement_information;
        size_t                      iterations = 0;  //!< Results::nIterations
        double                      quality    = 0;  //!< Results::quality
        /** Motion from the previous registered scan to this one, in the
         *  frame of the previous one. */
        mrpt::poses::CPose3D increment;
        double               dt = 0;  //!< Time between both scans [s]
    };

    /** Loads a model file. Throws if it is malformed or not a model of this
     *  class; then the previously loaded model, if any, is kept. */
    void load(const std::string& yamlFile);

    /** Same as load(), from an already parsed YAML document. */
    void load(const mrpt::containers::yaml& doc);

    bool loaded() const { return !layers_.empty(); }

    /** The model's own description line. */
    const std::string& description() const { return description_; }

    /** Covariance of the increment in the SE(3) right tangent of the
     *  current pose, translation first [m^2, rad^2]. The model must be
     *  loaded. */
    mrpt::math::CMatrixDouble66 predict(const Input& in) const;

    /** Feature names, in the order the model file lists them. */
    const std::vector<std::string>& featureNames() const { return featureNames_; }

   private:
    enum class Activation
    {
        Linear,
        Tanh
    };
    struct Layer
    {
        Eigen::MatrixXd W;  //!< n_out x n_in
        Eigen::VectorXd b;
        Activation      act = Activation::Linear;
    };
    enum class Feature
    {
        LogLambda,
        TranslationalShare,
        ZShare,
        YawShare,
        Iterations,
        Quality,
        Speed,
        TurnRate
    };

    std::string              description_;
    std::vector<std::string> featureNames_;
    std::vector<Feature>     features_;
    Eigen::VectorXd          mean_;
    Eigen::VectorXd          std_;
    std::vector<Layer>       layers_;
    double                   rotLength_    = 1.0;
    double                   floorTrans_   = 0;
    double                   floorRot_     = 0;
    double                   rotVarPerRad_ = 0;
    double                   zMin_         = -30;
    double                   zMax_         = 5;
};

}  // namespace mp2p_icp
