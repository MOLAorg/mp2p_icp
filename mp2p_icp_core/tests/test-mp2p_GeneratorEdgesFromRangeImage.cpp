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
 * @file   test-mp2p_GeneratorEdgesFromRangeImage.cpp
 * @brief  Unit test for GeneratorEdgesFromRangeImage with 3D range images
 * @author Jose Luis Blanco Claraco
 * @date   Nov 18, 2026
 */
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/GeneratorEdgesFromRangeImage.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <iostream>

namespace
{
// A range image whose rows are two flat segments with a range step in the
// middle: a few edge points at the step, plane points elsewhere.
mrpt::obs::CObservation3DRangeScan make_step_range_image()
{
    constexpr int W = 128;
    constexpr int H = 32;

    mrpt::obs::CObservation3DRangeScan obs;
    obs.hasRangeImage = true;
    obs.rangeImage_setSize(H, W);
    obs.cameraParams.ncols = W;
    obs.cameraParams.nrows = H;
    obs.cameraParams.fx(100.0);
    obs.cameraParams.fy(100.0);
    obs.cameraParams.cx(W / 2.0);
    obs.cameraParams.cy(H / 2.0);

    for (int r = 0; r < H; r++)
    {
        for (int c = 0; c < W; c++)
        {
            obs.rangeImage(r, c) = c < W / 2 ? 2000 : 4000;  // [mm]
        }
    }
    return obs;
}

mp2p_icp::metric_map_t run_generator(const std::string& planesLayer)
{
    mp2p_icp_filters::GeneratorEdgesFromRangeImage gen;

    mrpt::containers::yaml params = mrpt::containers::yaml::Map();
    params["target_layer"]        = "edges";
    params["planes_target_layer"] = planesLayer;
    params["score_threshold"]     = 2;
    gen.initialize(params);

    mp2p_icp::metric_map_t map;
    const bool             processed = gen.process(make_step_range_image(), map);
    ASSERT_(processed);
    return map;
}

void test_edges_and_planes_layers()
{
    const auto map = run_generator("planes");

    ASSERT_EQUAL_(map.layers.count("edges"), 1UL);
    ASSERT_EQUAL_(map.layers.count("planes"), 1UL);
    // Each layer must hold its own points, not the same cloud:
    ASSERT_(map.layers.at("edges") != map.layers.at("planes"));

    const auto edges  = map.point_layer("edges");
    const auto planes = map.point_layer("planes");
    ASSERT_(edges && planes);
    ASSERT_(!edges->empty());
    ASSERT_(!planes->empty());
    ASSERT_GT_(planes->size(), edges->size());

    std::cout << "[Test Passed] edges=" << edges->size() << " planes=" << planes->size()
              << std::endl;
}

void test_no_planes_layer()
{
    // An empty planes layer name is allowed: only edges are generated.
    const auto map = run_generator("");

    ASSERT_EQUAL_(map.layers.count("edges"), 1UL);
    ASSERT_EQUAL_(map.layers.size(), 1UL);
    ASSERT_(!map.point_layer("edges")->empty());

    std::cout << "[Test Passed] no planes layer" << std::endl;
}
}  // namespace

int main()
{
    try
    {
        test_edges_and_planes_layers();
        test_no_planes_layer();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
