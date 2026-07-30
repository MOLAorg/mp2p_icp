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
 * @file   test-mp2p_PointCloudRobustRange.cpp
 * @brief  Unit test for robust_max_range()
 * @author Jose Luis Blanco Claraco
 * @date   Jul 30, 2026
 */

#include <mp2p_icp_filters/PointCloudRobustRange.h>
#include <mrpt/maps/CSimplePointsMap.h>

using namespace mp2p_icp_filters;

namespace
{

// 97 "real" points spread between 1.0 and 5.0 m, plus 3 far outliers at
// 400/401/402 m (3%): a stand-in for the Livox specular-reflection artifacts
// that motivated this function -- a tiny minority of returns hundreds of
// meters away in an otherwise small scene.
mrpt::maps::CSimplePointsMap::Ptr createOutlierPointCloud()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();

    for (int i = 0; i < 97; i++)
    {
        const float range = 1.0f + i * (4.0f / 96.0f);  // 1.0 ... 5.0
        pc->insertPoint(range, 0.0f, 0.0f);
    }
    pc->insertPoint(400.0f, 0.0f, 0.0f);
    pc->insertPoint(401.0f, 0.0f, 0.0f);
    pc->insertPoint(402.0f, 0.0f, 0.0f);

    return pc;
}

void test_empty_cloud()
{
    mrpt::maps::CSimplePointsMap pc;
    ASSERT_EQUAL_(robust_max_range(pc), 0.0);
}

void test_outliers_are_ignored_at_p95()
{
    auto pc = createOutlierPointCloud();

    const double r = robust_max_range(*pc, 0.95);

    // rank = floor(0.95 * 100) = 95, which falls within the 97 "real"
    // points (indices 0..96 once sorted), well clear of the 3 outliers.
    ASSERT_GE_(r, 4.5);
    ASSERT_LE_(r, 5.01);
}

void test_percentile_one_reproduces_plain_max()
{
    auto pc = createOutlierPointCloud();

    const double r = robust_max_range(*pc, 1.0);

    // Must include the worst outlier, exactly what a bounding-box-derived
    // estimate would also give -- the behavior this function is meant to
    // improve on when a lower percentile is used instead.
    ASSERT_(r > 401.9 && r < 402.1);
}

void test_custom_center()
{
    auto pc = mrpt::maps::CSimplePointsMap::Create();
    pc->insertPoint(10.0f, 0.0f, 0.0f);
    pc->insertPoint(10.0f, 0.0f, 0.0f);

    // Measuring from (10,0,0) itself: range should collapse to ~0.
    const double r = robust_max_range(*pc, 1.0, {10.0f, 0.0f, 0.0f});
    ASSERT_LE_(r, 1e-4);
}

void test_invalid_percentile_throws()
{
    auto pc = createOutlierPointCloud();

    bool have_thrown = false;
    try
    {
        robust_max_range(*pc, 0.0);
    }
    catch (const std::exception&)
    {
        have_thrown = true;
    }
    ASSERT_(have_thrown);

    have_thrown = false;
    try
    {
        robust_max_range(*pc, 1.5);
    }
    catch (const std::exception&)
    {
        have_thrown = true;
    }
    ASSERT_(have_thrown);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_empty_cloud();
        std::cout << "test_empty_cloud: Success ✅" << std::endl;

        test_outliers_are_ignored_at_p95();
        std::cout << "test_outliers_are_ignored_at_p95: Success ✅" << std::endl;

        test_percentile_one_reproduces_plain_max();
        std::cout << "test_percentile_one_reproduces_plain_max: Success ✅" << std::endl;

        test_custom_center();
        std::cout << "test_custom_center: Success ✅" << std::endl;

        test_invalid_percentile_throws();
        std::cout << "test_invalid_percentile_throws: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
