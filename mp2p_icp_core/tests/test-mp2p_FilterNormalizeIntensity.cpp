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
 * @file   test-mp2p_FilterNormalizeIntensity.cpp
 * @brief  Unit test for FilterNormalizeIntensity
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterNormalizeIntensity.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <iostream>

using namespace mp2p_icp_filters;
using namespace mp2p_icp;

namespace
{
mrpt::maps::CGenericPointsMap::Ptr createCloudWithIntensities(const std::vector<float>& Is)
{
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float(POINT_FIELD_INTENSITY);

    for (float I : Is)
    {
        pc->insertPoint(0.0f, 0.0f, 0.0f);
        pc->insertPointField_float(POINT_FIELD_INTENSITY, I);
    }
    return pc;
}

/** Basic case: min/max are taken from the data itself, mapped to [0,1]. */
void test_normalizes_to_0_1_range()
{
    FilterNormalizeIntensity filter;
    mrpt::containers::yaml   p;
    p["pointcloud_layer"] = "raw";
    filter.initialize(p);

    auto input = createCloudWithIntensities({10.0f, 20.0f, 30.0f, 40.0f});

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto* Is = input->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    ASSERT_(Is != nullptr);
    ASSERT_EQUAL_(Is->size(), 4U);
    ASSERT_NEAR_(Is->at(0), 0.0f, 1e-5f);
    ASSERT_NEAR_(Is->at(1), 1.0 / 3, 1e-5);
    ASSERT_NEAR_(Is->at(2), 2.0 / 3, 1e-5);
    ASSERT_NEAR_(Is->at(3), 1.0f, 1e-5f);
}

/** With fixed_maximum_intensity set, that value (and fixed_minimum_intensity)
 *  must be used instead of the data's own min/max. */
void test_fixed_range_overrides_data_range()
{
    FilterNormalizeIntensity filter;
    mrpt::containers::yaml   p;
    p["pointcloud_layer"]        = "raw";
    p["fixed_maximum_intensity"] = 100.0;
    p["fixed_minimum_intensity"] = 0.0;
    filter.initialize(p);

    // Data range is [10,40], but the fixed range [0,100] must be used:
    auto input = createCloudWithIntensities({10.0f, 40.0f});

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto* Is = input->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    ASSERT_NEAR_(Is->at(0), 0.10f, 1e-5f);
    ASSERT_NEAR_(Is->at(1), 0.40f, 1e-5f);
}

/** All points with the same intensity: delta=0 must not produce a
 *  divide-by-zero, and the whole cloud must normalize to the low end
 *  (clamped 0). */
void test_constant_intensity_does_not_divide_by_zero()
{
    FilterNormalizeIntensity filter;
    mrpt::containers::yaml   p;
    p["pointcloud_layer"] = "raw";
    filter.initialize(p);

    auto input = createCloudWithIntensities({5.0f, 5.0f, 5.0f});

    metric_map_t map;
    map.layers["raw"] = input;

    filter.filter(map);

    auto* Is = input->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
    for (size_t i = 0; i < Is->size(); i++)
    {
        ASSERT_(std::isfinite(Is->at(i)));
        ASSERT_NEAR_(Is->at(i), 0.0f, 1e-5f);
    }
}

/** An empty intensity channel must be a no-op, not throw. */
void test_empty_channel_is_noop()
{
    FilterNormalizeIntensity filter;
    mrpt::containers::yaml   p;
    p["pointcloud_layer"] = "raw";
    filter.initialize(p);

    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->registerField_float(POINT_FIELD_INTENSITY);
    // no points inserted => intensity buffer is empty

    metric_map_t map;
    map.layers["raw"] = pc;

    filter.filter(map);  // must not throw

    ASSERT_EQUAL_(pc->size(), 0U);
}

/** Missing layer, and missing intensity field, must both throw. */
void test_missing_layer_and_missing_field_throw()
{
    {
        FilterNormalizeIntensity filter;
        mrpt::containers::yaml   p;
        p["pointcloud_layer"] = "does_not_exist";
        filter.initialize(p);

        metric_map_t map;

        bool didThrow = false;
        try
        {
            filter.filter(map);
        }
        catch (const std::exception&)
        {
            didThrow = true;
        }
        ASSERT_(didThrow);
    }
    {
        FilterNormalizeIntensity filter;
        mrpt::containers::yaml   p;
        p["pointcloud_layer"] = "raw";
        filter.initialize(p);

        auto pc = mrpt::maps::CGenericPointsMap::Create();
        pc->insertPoint(1.0f, 2.0f, 3.0f);  // no intensity field registered

        metric_map_t map;
        map.layers["raw"] = pc;

        bool didThrow = false;
        try
        {
            filter.filter(map);
        }
        catch (const std::exception&)
        {
            didThrow = true;
        }
        ASSERT_(didThrow);
    }
}

/** With `remember_intensity_range`, the min/max observed across successive
 *  calls must be merged (widened), not reset each time. */
void test_remember_intensity_range_widens_across_calls()
{
    FilterNormalizeIntensity filter;
    mrpt::containers::yaml   p;
    p["pointcloud_layer"]         = "raw";
    p["remember_intensity_range"] = true;
    filter.initialize(p);

    // First cloud: range [10,20]
    {
        auto         input = createCloudWithIntensities({10.0f, 20.0f});
        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);

        auto* Is = input->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
        ASSERT_NEAR_(Is->at(0), 0.0f, 1e-5f);
        ASSERT_NEAR_(Is->at(1), 1.0f, 1e-5f);
    }

    // Second cloud: range [0,10]. The remembered range must now be [0,20],
    // so a point at intensity 10 maps to 0.5, not 1.0:
    {
        auto         input = createCloudWithIntensities({0.0f, 10.0f});
        metric_map_t map;
        map.layers["raw"] = input;
        filter.filter(map);

        auto* Is = input->getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
        ASSERT_NEAR_(Is->at(0), 0.0f, 1e-5f);
        ASSERT_NEAR_(Is->at(1), 0.5f, 1e-5f);
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_normalizes_to_0_1_range();
        std::cout << "test_normalizes_to_0_1_range: Success ✅" << std::endl;

        test_fixed_range_overrides_data_range();
        std::cout << "test_fixed_range_overrides_data_range: Success ✅" << std::endl;

        test_constant_intensity_does_not_divide_by_zero();
        std::cout << "test_constant_intensity_does_not_divide_by_zero: Success ✅" << std::endl;

        test_empty_channel_is_noop();
        std::cout << "test_empty_channel_is_noop: Success ✅" << std::endl;

        test_missing_layer_and_missing_field_throw();
        std::cout << "test_missing_layer_and_missing_field_throw: Success ✅" << std::endl;

        test_remember_intensity_range_widens_across_calls();
        std::cout << "test_remember_intensity_range_widens_across_calls: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
