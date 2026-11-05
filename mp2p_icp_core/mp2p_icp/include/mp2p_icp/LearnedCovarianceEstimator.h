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
 * @file   LearnedCovarianceEstimator.h
 * @brief  Learned covariance of the registrations of a sliding-window
 *         odometry, computed off the caller's thread.
 */
#pragma once

#include <mp2p_icp/IcpCovFeatures.h>
#include <mp2p_icp/LearnedCovariance.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Core>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mp2p_icp
{
/** Learned covariance (LearnedCovariance + CovFeatureEvaluator) of every
 * registration of a scan-to-map odometry, computed by a worker thread so the
 * caller only pays for queueing a request.
 *
 * The evaluator's reference is a sliding window with the last
 * `window_scans` accepted scans, each at its estimated pose. Requests and
 * window updates are served strictly in call order, so each registration is
 * evaluated against exactly the window that preceded it: results do not
 * depend on when, or from which thread, they are collected.
 *
 * Typical use, per registration of scan k:
 * \code
 *  auto cov = est.submit({scan_k, solution_k, ctx_k}); // cheap
 *  // ... hand `cov` to its consumer (e.g. a state estimator) ...
 *  est.pushToWindow(scan_k, pose_k); // cheap, once scan k is accepted
 * \endcode
 *
 * If the worker falls behind, the oldest evaluations still waiting are
 * answered without a learned covariance (Result::ok false), and queued scans
 * that would leave the window before any evaluation sees them are dropped,
 * which bounds the latency and the queue. If a window update fails, the
 * window restarts empty.
 *
 * \ingroup mp2p_icp_grp
 */
class LearnedCovarianceEstimator
{
   public:
    struct Parameters
    {
        std::string model_file;  //!< Written by the training scripts
        size_t      window_scans = 25;  //!< Must match the model's training

        /** Context labels the model was trained with, e.g.
         *  {"framework_name": "mp2p_gicp", "sensor": "...", "mode": "s2m"}. */
        std::map<std::string, std::string> context;

        /** Evaluations allowed to wait for the worker, besides the running
         *  one. At least 1. */
        size_t max_waiting = 1;
    };

    struct Request
    {
        /** Query scan in the vehicle frame: full resolution, de-skewed.
         *  Shared, not copied: do not modify it before the result is ready. */
        mrpt::maps::CPointsMap::Ptr scan;

        /** Registration result: vehicle pose in the map frame. */
        mrpt::poses::CPose3D solution;

        /** How the result was obtained (initial guess, prior, iterations...). */
        CovFeatureContext context;
    };

    struct Result
    {
        /** False if no learned covariance is available for this request:
         *  empty window, too few associations, or the worker fell behind. */
        bool ok = false;

        /** Covariance in the right tangent `[x y z rx ry rz]` of the solution. */
        Eigen::Matrix<double, 6, 6> Sigma = Eigen::Matrix<double, 6, 6>::Zero();

        double pFail = 0;  //!< Probability that the registration failed
    };

    /** Loads the model and starts the worker. Throws on a model error. */
    explicit LearnedCovarianceEstimator(const Parameters& p);

    /** Stops the worker; requests still waiting get `ok=false`. */
    ~LearnedCovarianceEstimator();

    LearnedCovarianceEstimator(const LearnedCovarianceEstimator&)            = delete;
    LearnedCovarianceEstimator& operator=(const LearnedCovarianceEstimator&) = delete;

    /** Queues the evaluation of one registration against the current window. */
    std::shared_future<Result> submit(Request r);

    /** Appends an accepted scan (vehicle frame) at its estimated pose. The
     *  scan is shared, not copied, until it leaves the window: do not modify
     *  it after this call. */
    void pushToWindow(mrpt::maps::CPointsMap::Ptr scan, const mrpt::poses::CPose3D& pose);

    /** Empties the window, e.g. after the odometry restarts. */
    void clearWindow();

    const Parameters&        parameters() const { return params_; }
    const LearnedCovariance& model() const { return model_; }

   private:
    struct Task
    {
        enum class Kind
        {
            Evaluate,
            Push,
            Clear
        };
        Kind                                  kind = Kind::Evaluate;
        Request                               request;  //!< Evaluate; Push uses scan+solution
        std::shared_ptr<std::promise<Result>> promise;
    };

    Parameters        params_;
    LearnedCovariance model_;

    // Shared with the worker:
    std::mutex              mtx_;
    std::condition_variable cv_;
    std::deque<Task>        queue_;
    bool                    stop_ = false;

    // Worker-owned:
    std::unique_ptr<CovFeatureEvaluator>                                     evaluator_;
    std::deque<std::pair<mrpt::maps::CPointsMap::Ptr, mrpt::poses::CPose3D>> window_;
    bool                                                                     dirty_ = false;

    std::thread worker_;

    void   enqueue(Task t);
    void   dropRedundantPushes();  //!< With mtx_ held
    void   run();
    Result evaluate(const Request& r);
};

}  // namespace mp2p_icp
