/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \\ / _ \\| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\\___/|_|\\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   test-mp2p_FilterAbsoluteTimestamp.cpp
 * @brief  Unit test for FilterAbsoluteTimestamp
 * @author Jose Luis Blanco Claraco, Google Gemini
 * @date   Jan 6, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterAbsoluteTimestamp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/system/datetime.h>
#include <mrpt/version.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
#if MRPT_VERSION < 0x020f04
    std::cerr << "*SKIPPING TEST*: due to missing MRPT>=2.15.4 for this filter\n";
    return 0;
#elif !defined MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION
    std::cerr << "*SKIPPING TEST*: due to missing mola_imu_preintegration for this filter\n";
    return 0;
#else
    try
    {
        // 1. Setup a point cloud with a "t" (relative time) channel
        auto pc = mrpt::maps::CGenericPointsMap::Create();
        pc->registerField_float("t");

        pc->insertPoint(1.0, 2.0, 3.0);
        pc->insertPointField_float("t", 0.1f);

        pc->insertPoint(4.0, 5.0, 6.0);
        pc->insertPointField_float("t", 0.5f);

        // 2. Setup Filter
        FilterAbsoluteTimestamp filter;
        mrpt::containers::yaml  p;
        p["pointcloud_layer"]  = "raw";
        p["output_field_name"] = "abs_time";
        filter.initialize(p);

        // 3. Mock the Processing Context (attached source)
        // We need to set the reference zero time in the velocity buffer
        mp2p_icp::ParameterSource context;
        const auto                now    = mrpt::Clock::now();
        const double              nowSec = mrpt::Clock::toDouble(now);
        context.localVelocityBuffer.set_reference_zero_time(nowSec);

        filter.attachToParameterSource(context);

        // 4. Run Filter
        mp2p_icp::metric_map_t map;
        map.layers["raw"] = pc;
        filter.filter(map);

        // 5. Verify Results
        auto        outPc = map.layer<mrpt::maps::CPointsMap>("raw");
        const auto* absT  = outPc->getPointsBufferRef_double_field("abs_time");

        ASSERT_(absT != nullptr);
        ASSERT_EQUAL_(absT->size(), 2ULL);

        // Verification: absolute = reference + relative
        ASSERT_NEAR_((*absT)[0], nowSec + 0.1, 1e-6);
        ASSERT_NEAR_((*absT)[1], nowSec + 0.5, 1e-6);
        std::cout << "FilterAbsoluteTimestamp Unit Test Passed!" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
#endif
}