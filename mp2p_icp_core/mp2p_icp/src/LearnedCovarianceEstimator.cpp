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
 * @file   LearnedCovarianceEstimator.cpp
 * @brief  Learned covariance of the registrations of a sliding-window
 *         odometry, computed off the caller's thread.
 */

#include <mp2p_icp/LearnedCovarianceEstimator.h>
#include <mrpt/core/exceptions.h>

using namespace mp2p_icp;

LearnedCovarianceEstimator::LearnedCovarianceEstimator(const Parameters& p) : params_(p)
{
    ASSERTMSG_(params_.window_scans > 0, "LearnedCovarianceEstimator: window_scans must be > 0");
    model_.load(params_.model_file);
    evaluator_ = std::make_unique<CovFeatureEvaluator>();
    worker_    = std::thread([this]() { run(); });
}

LearnedCovarianceEstimator::~LearnedCovarianceEstimator()
{
    std::deque<Task> left;
    {
        std::lock_guard<std::mutex> lck(mtx_);
        stop_ = true;
        left.swap(queue_);
    }
    cv_.notify_all();
    worker_.join();
    for (auto& t : left)
    {
        if (t.promise)
        {
            t.promise->set_value(Result{});
        }
    }
}

std::shared_future<LearnedCovarianceEstimator::Result> LearnedCovarianceEstimator::submit(Request r)
{
    Task t;
    t.kind                       = Task::Kind::Evaluate;
    t.request                    = std::move(r);
    t.promise                    = std::make_shared<std::promise<Result>>();
    std::shared_future<Result> f = t.promise->get_future().share();
    enqueue(std::move(t));
    return f;
}

void LearnedCovarianceEstimator::pushToWindow(
    mrpt::maps::CPointsMap::Ptr scan, const mrpt::poses::CPose3D& pose)
{
    ASSERT_(scan);
    Task t;
    t.kind             = Task::Kind::Push;
    t.request.scan     = std::move(scan);
    t.request.solution = pose;
    enqueue(std::move(t));
}

void LearnedCovarianceEstimator::clearWindow()
{
    Task t;
    t.kind = Task::Kind::Clear;
    enqueue(std::move(t));
}

void LearnedCovarianceEstimator::enqueue(Task t)
{
    std::shared_ptr<std::promise<Result>> dropped;
    {
        std::lock_guard<std::mutex> lck(mtx_);
        if (t.kind == Task::Kind::Evaluate)
        {
            // Bound the latency: answer the oldest waiting evaluation now.
            size_t waiting = 0;
            for (const auto& q : queue_)
            {
                waiting += q.kind == Task::Kind::Evaluate ? 1 : 0;
            }
            if (waiting >= params_.max_waiting)
            {
                for (auto it = queue_.begin(); it != queue_.end(); ++it)
                {
                    if (it->kind == Task::Kind::Evaluate)
                    {
                        dropped = it->promise;
                        queue_.erase(it);
                        break;
                    }
                }
            }
        }
        queue_.push_back(std::move(t));
    }
    cv_.notify_one();
    if (dropped)
    {
        dropped->set_value(Result{});
    }
}

void LearnedCovarianceEstimator::run()
{
    for (;;)
    {
        Task t;
        {
            std::unique_lock<std::mutex> lck(mtx_);
            cv_.wait(lck, [this]() { return stop_ || !queue_.empty(); });
            if (stop_)
            {
                return;
            }
            t = std::move(queue_.front());
            queue_.pop_front();
        }

        // An exception must not escape the worker: a failed evaluation is
        // answered without a learned covariance.
        try
        {
            switch (t.kind)
            {
                case Task::Kind::Push:
                    // The window slides one scan per step: update the reference
                    // incrementally instead of rebuilding it.
                    window_.emplace_back(t.request.scan, t.request.solution);
                    evaluator_->addReferenceCloud(*t.request.scan, t.request.solution);
                    while (window_.size() > params_.window_scans)
                    {
                        const auto& [oldScan, oldPose] = window_.front();
                        evaluator_->removeReferenceCloud(*oldScan, oldPose);
                        window_.pop_front();
                    }
                    dirty_ = true;
                    break;

                case Task::Kind::Clear:
                    window_.clear();
                    evaluator_ = std::make_unique<CovFeatureEvaluator>();
                    dirty_     = false;
                    break;

                case Task::Kind::Evaluate:
                    t.promise->set_value(evaluate(t.request));
                    t.promise.reset();
                    break;
            }
        }
        catch (const std::exception&)
        {
            if (t.promise)
            {
                t.promise->set_value(Result{});
            }
        }
    }
}

LearnedCovarianceEstimator::Result LearnedCovarianceEstimator::evaluate(const Request& req)
{
    Result out;
    if (window_.empty() || !req.scan || req.scan->empty())
    {
        return out;
    }
    if (dirty_)
    {
        evaluator_->finalizeReference();
        dirty_ = false;
    }
    evaluator_->setScan(*req.scan);
    const auto f = evaluator_->evaluate(req.solution, req.context);
    if (!f.ok)
    {
        return out;
    }
    const auto pred = model_.predict(f, params_.context);
    out.ok          = true;
    out.Sigma       = pred.Sigma;
    out.pFail       = pred.pFail;
    return out;
}
