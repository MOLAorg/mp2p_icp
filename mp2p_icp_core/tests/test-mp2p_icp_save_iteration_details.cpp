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
 * @file   test-mp2p_icp_save_iteration_details.cpp
 * @brief  Regression test: ICP::align() with saveIterationDetails but no debug
 *         record requested must not write into the (empty) record, which
 *         leaked the per-iteration details on every call. The details must
 *         still be recorded when a record is requested.
 */

#include <mp2p_icp/LogRecord.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>

// Allocations made through the global operator new that are still alive: a
// leak from inside the library shows up here without needing a sanitizer.
namespace
{
std::atomic<long> liveAllocations{0};
}

void* operator new(std::size_t n)
{
    void* p = std::malloc(n == 0 ? 1 : n);
    if (p == nullptr)
    {
        throw std::bad_alloc();
    }
    liveAllocations++;
    return p;
}

void* operator new[](std::size_t n) { return ::operator new(n); }

void operator delete(void* p) noexcept
{
    if (p != nullptr)
    {
        liveAllocations--;
        std::free(p);
    }
}

void operator delete[](void* p) noexcept { ::operator delete(p); }
void operator delete(void* p, [[maybe_unused]] std::size_t n) noexcept { ::operator delete(p); }
void operator delete[](void* p, [[maybe_unused]] std::size_t n) noexcept { ::operator delete(p); }

namespace
{
const char* PIPELINE = R"(
class_name: mp2p_icp::ICP
params:
  maxIterations: 20
  minAbsStep_trans: 1e-9
  minAbsStep_rot: 1e-9
  saveIterationDetails: true
  decimationIterationDetails: 1
solvers:
  - class: mp2p_icp::Solver_Horn
    params: ~
matchers:
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 1.0
      thresholdAngularDeg: 0
quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params: ~
)";

void test_save_iteration_details()
{
    auto [icp, params] =
        mp2p_icp::icp_pipeline_from_yaml(mrpt::containers::yaml::FromText(PIPELINE));
    ASSERT_(params.saveIterationDetails);
    ASSERT_(!params.generateDebugFiles);

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (int i = 0; i < 2000; i++)
    {
        pts->insertPoint(
            static_cast<float>(rng.drawUniform(-5.0, 5.0)),
            static_cast<float>(rng.drawUniform(-5.0, 5.0)),
            static_cast<float>(rng.drawUniform(-0.5, 0.5)));
    }
    mp2p_icp::metric_map_t global;
    mp2p_icp::metric_map_t local;
    global.layers["raw"] = pts;
    local.layers["raw"]  = pts;

    const mrpt::math::TPose3D initialGuess(0.1, 0.05, 0.0, 0.02, 0.0, 0.0);
    mp2p_icp::Results         result;

    // No debug record requested: nothing to store the details into. The first
    // call fills whatever the pipeline caches; after it, align() must not
    // leave a single allocation behind.
    icp->align(local, global, initialGuess, params, result);
    ASSERT_GT_(result.nIterations, 1U);

    const long before = liveAllocations.load();
    for (int i = 0; i < 10; i++)
    {
        icp->align(local, global, initialGuess, params, result);
    }
    const long leaked = liveAllocations.load() - before;
    std::cout << "[save_iteration_details] allocations left by 10 align() calls: " << leaked
              << "\n";
    ASSERT_EQUAL_(leaked, 0L);

    // With a record requested, the details of every iteration are there:
    mp2p_icp::LogRecord log;
    icp->align(local, global, initialGuess, params, result, std::nullopt, log);
    // (the iteration that ends the loop is recorded too, but not counted in
    // nIterations when it ends it by convergence)
    ASSERT_(log.iterationsDetails.has_value());
    for (size_t i = 0; i < result.nIterations; i++)
    {
        ASSERT_EQUAL_(log.iterationsDetails->count(i), 1UL);
    }
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_save_iteration_details();
        std::cout << "[save_iteration_details] OK\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
