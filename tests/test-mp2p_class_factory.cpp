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
 * @file   test-mp2p_class_factory.cpp
 * @brief  Unit tests to check correct registration of classes
 * @author Jose Luis Blanco Claraco
 * @date   Jan 21, 2026
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/rtti/CObject.h>

#include <iostream>  // cerr

namespace
{
int test_class_factory()
{
    // Enforce linking against these two libraries:
    {
        mp2p_icp::ICP icp;
        ASSERT_EQUAL_(icp.matchers().size(), 0U);
    }
    {
        mp2p_icp_filters::FilterBoundingBox filter;
        ASSERT_(filter.attachedSource() == nullptr);
    }

    const std::vector<const char*> class_names = {
        "mp2p_icp_filters::Generator", "mp2p_icp_filters::GeneratorEdgesFromRangeImage",
        "mp2p_icp_filters::GeneratorEdgesFromCurvature",

        // Filters:
        "mp2p_icp_filters::FilterAbsoluteTimestamp", "mp2p_icp_filters::FilterAdjustTimestamps",
        "mp2p_icp_filters::FilterBoundingBox", "mp2p_icp_filters::FilterByExpression",
        "mp2p_icp_filters::FilterByIntensity", "mp2p_icp_filters::FilterByRange",
        "mp2p_icp_filters::FilterByRing", "mp2p_icp_filters::FilterCurvature",
        "mp2p_icp_filters::FilterDecimate", "mp2p_icp_filters::FilterDecimateAdaptive",
        "mp2p_icp_filters::FilterDecimateVoxels", "mp2p_icp_filters::FilterDeleteLayer",
        "mp2p_icp_filters::FilterDeskew", "mp2p_icp_filters::FilterEdgesPlanes",
        "mp2p_icp_filters::FilterFartherPointSampling", "mp2p_icp_filters::FilterMLS",
        "mp2p_icp_filters::FilterMerge", "mp2p_icp_filters::FilterNormalizeIntensity",
        "mp2p_icp_filters::FilterPoleDetector", "mp2p_icp_filters::FilterRemoveByVoxelOccupancy",
        "mp2p_icp_filters::FilterRenameLayer", "mp2p_icp_filters::FilterSOR",
        "mp2p_icp_filters::FilterVoxelSOR", "mp2p_icp_filters::FilterVoxelSlice",

        // mp2p_icp classes:
        "mp2p_icp::ICP", "mp2p_icp::Solver_GaussNewton", "mp2p_icp::Solver_Horn",
        "mp2p_icp::Solver_OLAE", "mp2p_icp::Matcher_Adaptive", "mp2p_icp::Matcher_Cov2Cov",
        "mp2p_icp::Matcher_Point2Line", "mp2p_icp::Matcher_Point2Plane",
        "mp2p_icp::Matcher_Points_DistanceThreshold", "mp2p_icp::Matcher_Points_InlierRatio",
        "mp2p_icp::QualityEvaluator_PairedRatio", "mp2p_icp::QualityEvaluator_RangeImageSimilarity",
        "mp2p_icp::QualityEvaluator_Voxels", "mp2p_icp::LogRecord", "mp2p_icp::Parameters"};

    // Try to create an object of each known class:
    size_t failures = 0;
    for (const auto& class_name : class_names)
    {
        const auto obj = mrpt::rtti::classFactory(class_name);
        if (!obj)
        {
            failures++;
            std::cerr << "Failed to create object of class: '" << class_name << "'\n";
        }
    }
    if (failures == 0)
    {
        std::cout << "✓ All classes created successfully\n";
        return 0;
    }

    std::cerr << "✗ " << failures << " class(es) failed to create\n";
    return 1;
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        return test_class_factory();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
