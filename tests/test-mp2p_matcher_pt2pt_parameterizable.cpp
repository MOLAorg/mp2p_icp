/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   test-mp2p_matcher_pt2pt_parameterizable.cpp
 * @brief  Unit tests for matcher with Parameterizable dynamic parameters
 * @author Jose Luis Blanco Claraco
 * @date   Dec 13, 2023
 */

#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        {
            mp2p_icp::Matcher_Points_DistanceThreshold m;
            mrpt::containers::yaml                     p;
            p["threshold"]           = "MATCH_THRESHOLD*2.0";  // Define as an expr.
            p["thresholdAngularDeg"] = .0;

            m.initialize(p);

            mp2p_icp::ParameterSource globalParams;
            globalParams.attach(m);

            // v1:
            globalParams.updateVariable("MATCH_THRESHOLD", 1.5);
            globalParams.realize();

            ASSERT_NEAR_(m.threshold, 3.0, 1e-4);
            ASSERT_NEAR_(m.thresholdAngularDeg, .0, 1e-4);

            // v2:
            globalParams.updateVariable("MATCH_THRESHOLD", 0.5);
            globalParams.realize();

            ASSERT_NEAR_(m.threshold, 1.0, 1e-4);
            ASSERT_NEAR_(m.thresholdAngularDeg, .0, 1e-4);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
