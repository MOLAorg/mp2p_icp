/*
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
 * @file   test-mp2p_LogRecord.cpp
 * @brief  Unit tests for the LogRecord class (file I/O)
 * @author Jose Luis Blanco Claraco
 * @date   Jan 27, 2026
 */

#include <mp2p_icp/LogRecord.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{

// Helper to create a dummy metric map
metric_map_t::Ptr create_dummy_map()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    pc->insertPoint(1.0, 2.0, 3.0);
    pc->insertPoint(4.0, 5.0, 6.0);

    auto mm           = metric_map_t::Create();
    mm->layers["raw"] = pc;
    return mm;
}

void test_LogRecord_IO()
{
    const std::string filename = mrpt::system::getTempFileName() + ".icplog";

    // 1. Create and populate LogRecord
    // ---------------------------------------------------
    {
        LogRecord log;

        // Populate standard fields
        log.icpResult.nIterations       = 42;
        log.icpResult.quality           = 0.95;
        log.icpParameters.maxIterations = 100;
        log.initialGuessLocalWrtGlobal  = mrpt::math::TPose3D(1, 2, 3, 0.1, 0.2, 0.3);

        // Populate maps
        log.pcGlobal = create_dummy_map();
        log.pcLocal  = create_dummy_map();

        // Populate dynamic variables
        log.dynamicVariables["temperature"] = 25.5;
        log.dynamicVariables["pressure"]    = 101.3;

        // Populate iterations details (DebugInfoPerIteration)
        LogRecord::IterationsDetails details;

        LogRecord::DebugInfoPerIteration step1;
        step1.optimalPose = mrpt::poses::CPose3D(0.1, 0, 0, 0, 0, 0);
        step1.pairings.paired_pt2pt.resize(5);  // Dummy pairings
        details[0] = step1;

        LogRecord::DebugInfoPerIteration step2;
        step2.optimalPose = mrpt::poses::CPose3D(0.2, 0, 0, 0, 0, 0);
        step2.pairings.paired_pt2pl.resize(2);  // Dummy pairings
        details[1] = step2;

        log.iterationsDetails = details;

        // Save
        const bool save_ok = log.save_to_file(filename);
        ASSERT_(save_ok);
    }

    // 2. Load and Verify
    // ---------------------------------------------------
    {
        LogRecord  log;
        const bool load_ok = log.load_from_file(filename);
        ASSERT_(load_ok);

        // Verify Scalars
        ASSERT_EQUAL_(log.icpResult.nIterations, 42ULL);
        ASSERT_NEAR_(log.icpResult.quality, 0.95, 1e-6);
        ASSERT_EQUAL_(log.icpParameters.maxIterations, 100U);
        ASSERT_NEAR_(log.initialGuessLocalWrtGlobal.x, 1.0, 1e-6);

        // Verify Maps
        ASSERT_(log.pcGlobal);
        ASSERT_(log.pcLocal);
        ASSERT_EQUAL_(log.pcGlobal->layers.count("raw"), 1ULL);

        // Verify Dynamic Variables
        ASSERT_EQUAL_(log.dynamicVariables.size(), 2ULL);
        ASSERT_NEAR_(log.dynamicVariables.at("temperature"), 25.5, 1e-6);

        // Verify Iterations
        ASSERT_(log.iterationsDetails.has_value());
        ASSERT_EQUAL_(log.iterationsDetails->size(), 2ULL);

        const auto& step1 = log.iterationsDetails->at(0);
        ASSERT_NEAR_(step1.optimalPose.x(), 0.1, 1e-6);
        ASSERT_EQUAL_(step1.pairings.paired_pt2pt.size(), 5ULL);

        const auto& step2 = log.iterationsDetails->at(1);
        ASSERT_EQUAL_(step2.pairings.paired_pt2pl.size(), 2ULL);
    }

    // Cleanup
    mrpt::system::deleteFile(filename);

    std::cout << "[Test Passed] LogRecord File I/O\n";
}
}  // namespace

int main()
{
    try
    {
        test_LogRecord_IO();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}