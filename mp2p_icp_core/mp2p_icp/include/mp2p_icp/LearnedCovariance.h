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
 * @file   LearnedCovariance.h
 * @brief  Learned correction of the analytic registration covariance.
 */
#pragma once

#include <mp2p_icp/IcpCovFeatures.h>

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** A small ensemble of MLPs that corrects the analytic covariance of
 * CovFeatureEvaluator in its whitened space:
 *
 *   Sigma = L_a M L_a^T,   M = L_m L_m^T,
 *
 * with L_m lower triangular (log-diagonal and off-diagonal entries are the
 * network outputs). The ensemble's covariance is the mean of its members',
 * and it also outputs a probability that the registration failed.
 *
 * Inference only, plain Eigen; the model file is produced by the training
 * scripts of the learned-covariance study.
 *
 * \ingroup mp2p_icp_grp
 */
class LearnedCovariance
{
   public:
    /** Loads a model file. Throws on a format error or a feature mismatch
     *  with CovFeatureEvaluator::featureNames(). */
    void load(const std::string& file);

    bool loaded() const { return !members_.empty(); }

    struct Output
    {
        Eigen::Matrix<double, 6, 6> Sigma = Eigen::Matrix<double, 6, 6>::Zero();
        double                      pFail = 0;
    };

    /** Predicts the covariance of a registration from the evaluator's
     *  result and the context labels the model was trained with (e.g.
     *  {"framework_name": "mp2p_gicp", "sensor": "...", "mode": "s2m"}).
     *  Unknown context values contribute all-zero one-hots. Requires
     *  `r.ok`. */
    Output predict(
        const CovFeatureResult& r, const std::map<std::string, std::string>& context) const;

    size_t numMembers() const { return members_.size(); }

   private:
    struct Layer
    {
        bool            linear = true;
        Eigen::MatrixXd W;  //!< linear: rows x cols; layernorm: unused
        Eigen::VectorXd w;  //!< layernorm gain
        Eigen::VectorXd b;  //!< bias
    };
    using Member = std::vector<Layer>;

    std::vector<std::string> featNames_;
    Eigen::VectorXd          median_;
    Eigen::VectorXd          mean_;
    Eigen::VectorXd          std_;
    std::vector<size_t>      nanIdx_;  //!< features with a missing-value indicator
    std::vector<std::pair<std::string, std::vector<std::string>>> context_;
    double                                                        clip_  = 8.0;
    double                                                        lnEps_ = 1e-5;
    std::vector<Member>                                           members_;
    std::vector<int> featMap_;  //!< model feature -> evaluator feature index

    Eigen::VectorXd input(
        const CovFeatureResult& r, const std::map<std::string, std::string>& context) const;
};

}  // namespace mp2p_icp
