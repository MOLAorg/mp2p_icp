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
 * @file   test-mp2p_icp_iteration_hook.cpp
 * @brief  The ICP iteration hook sees every iteration that updates the pose,
 *         including the one that ends ICP, whatever the termination reason.
 */

#include <mp2p_icp/LogRecord.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random.h>
#include <mrpt/typemeta/TEnumType.h>

#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace std::string_literals;

namespace
{
const char* PIPELINE = R"(
class_name: mp2p_icp::ICP
params:
  maxIterations: 50
  minAbsStep_trans: 1e-4
  minAbsStep_rot: 1e-4
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

struct HookCall
{
    uint32_t             iteration;
    mrpt::poses::CPose3D pose;
};

void run_case(
    const char* name, const mrpt::math::TPose3D& initialGuess,
    const std::function<void(mp2p_icp::Parameters&)>& changeParams,
    mp2p_icp::IterTermReason expectedReason, std::optional<size_t> expectedIterations,
    std::optional<uint32_t> stopAtIteration = std::nullopt)
{
    auto [icp, params] =
        mp2p_icp::icp_pipeline_from_yaml(mrpt::containers::yaml::FromText(PIPELINE));
    changeParams(params);

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

    std::vector<HookCall> calls;
    icp->setIterationHook(
        [&](const mp2p_icp::ICP::IterationHook_Input& in)
        {
            calls.push_back({in.currentIteration, in.currentSolution->optimalPose});
            mp2p_icp::ICP::IterationHook_Output out;
            out.request_stop = stopAtIteration && in.currentIteration >= *stopAtIteration;
            return out;
        });

    mp2p_icp::Results   result;
    mp2p_icp::LogRecord log;
    icp->align(local, global, initialGuess, params, result, std::nullopt, log);

    using reasons = mrpt::typemeta::TEnumType<mp2p_icp::IterTermReason>;
    std::cout << "[iteration_hook] " << name << ": " << result.nIterations << " iterations, "
              << reasons::value2name(result.terminationReason) << "\n";

    ASSERTMSG_(
        result.terminationReason == expectedReason,
        "Expected termination: "s + reasons::value2name(expectedReason));
    if (expectedIterations)
    {
        ASSERT_EQUAL_(calls.size(), *expectedIterations);
    }

    // One hook call per executed iteration, in order, as the recorded details:
    ASSERT_(log.iterationsDetails.has_value());
    ASSERT_EQUAL_(log.iterationsDetails->size(), calls.size());
    for (size_t i = 0; i < calls.size(); i++)
    {
        ASSERT_EQUAL_(calls[i].iteration, i);
        ASSERT_EQUAL_(log.iterationsDetails->count(i), 1UL);
        ASSERT_(log.iterationsDetails->at(i).optimalPose == calls[i].pose);
    }

    // The last pose the hook sees is the result:
    ASSERT_(!calls.empty());
    ASSERT_(calls.back().pose == result.optimal_tf.mean);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        using mp2p_icp::IterTermReason;
        const mrpt::math::TPose3D offset(0.1, 0.05, 0.0, 0.02, 0.0, 0.0);
        const auto                noChange = [](mp2p_icp::Parameters&) {};

        // Converges after 3 iterations:
        run_case("converged", offset, noChange, IterTermReason::Stalled, 3U);

        // Already aligned: stalls in its first iteration:
        run_case("already aligned", {}, noChange, IterTermReason::Stalled, 1U);

        run_case(
            "max iterations", offset, [](auto& p) { p.maxIterations = 2; },
            IterTermReason::MaxIterations, 2U);

        run_case(
            "quality checkpoint", offset,
            [](auto& p) {
                p.quality_checkpoints = {{0, 1.1}};
            },
            IterTermReason::QualityCheckpointFailed, 1U);

        run_case(
            "hook request", offset, noChange, IterTermReason::HookRequest, 2U,
            1U /* stop at iteration index 1 */);
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
