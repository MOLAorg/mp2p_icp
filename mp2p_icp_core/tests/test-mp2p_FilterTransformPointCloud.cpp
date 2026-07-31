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
 * @file   test-mp2p_FilterTransformPointCloud.cpp
 * @brief  Unit test for FilterTransformPointCloud
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterTransformPointCloud.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>

using namespace mp2p_icp_filters;

int main()
{
    try
    {
        auto pc = mrpt::maps::CSimplePointsMap::Create();
        pc->insertPoint(1.0f, 0.0f, 0.0f);
        pc->insertPoint(0.0f, 1.0f, 0.0f);
        pc->insertPoint(0.0f, 0.0f, 1.0f);

        const mrpt::poses::CPose3D pose(1.0, 2.0, 3.0, 0.3, -0.2, 0.1);  // x,y,z,yaw,pitch,roll

        // ---------------------------------------------------------
        // Test: forward transform matches p' = pose (+) p
        // ---------------------------------------------------------
        {
            mp2p_icp::metric_map_t map;
            map.layers["raw"] = pc;

            FilterTransformPointCloud filter;
            mrpt::containers::yaml    p;
            p["input_pointcloud_layer"]  = "raw";
            p["output_pointcloud_layer"] = "world";
            p["pose"]                    = mrpt::containers::yaml::Sequence(
                                   {pose.x(), pose.y(), pose.z(), pose.yaw(), pose.pitch(), pose.roll()});
            p["invert_pose"] = false;

            filter.initialize(p);
            filter.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("world");
            ASSERT_(out);
            ASSERT_EQUAL_(out->size(), pc->size());

            for (size_t i = 0; i < pc->size(); i++)
            {
                float x, y, z;
                pc->getPoint(i, x, y, z);
                const auto expected = pose.composePoint({x, y, z});

                float ox, oy, oz;
                out->getPoint(i, ox, oy, oz);

                ASSERT_NEAR_(ox, expected.x, 1e-4);
                ASSERT_NEAR_(oy, expected.y, 1e-4);
                ASSERT_NEAR_(oz, expected.z, 1e-4);
            }
            std::cout << "[Test Passed] Forward transform matches pose composition\n";
        }

        // ---------------------------------------------------------
        // Test: forward then invert_pose=true round-trip recovers the input
        // ---------------------------------------------------------
        {
            mp2p_icp::metric_map_t map;
            map.layers["raw"] = pc;

            const auto poseSeq = mrpt::containers::yaml::Sequence(
                {pose.x(), pose.y(), pose.z(), pose.yaw(), pose.pitch(), pose.roll()});

            FilterTransformPointCloud fwd;
            mrpt::containers::yaml    pf;
            pf["input_pointcloud_layer"]  = "raw";
            pf["output_pointcloud_layer"] = "world";
            pf["pose"]                    = poseSeq;
            fwd.initialize(pf);
            fwd.filter(map);

            FilterTransformPointCloud back;
            mrpt::containers::yaml    pb;
            pb["input_pointcloud_layer"]  = "world";
            pb["output_pointcloud_layer"] = "local_again";
            pb["pose"]                    = poseSeq;
            pb["invert_pose"]             = true;
            back.initialize(pb);
            back.filter(map);

            auto out = map.layer<mrpt::maps::CPointsMap>("local_again");
            ASSERT_(out);
            ASSERT_EQUAL_(out->size(), pc->size());

            for (size_t i = 0; i < pc->size(); i++)
            {
                float x, y, z, ox, oy, oz;
                pc->getPoint(i, x, y, z);
                out->getPoint(i, ox, oy, oz);
                ASSERT_NEAR_(ox, x, 1e-4);
                ASSERT_NEAR_(oy, y, 1e-4);
                ASSERT_NEAR_(oz, z, 1e-4);
            }
            std::cout << "[Test Passed] Forward + inverse round-trip recovers input points\n";
        }

        // ---------------------------------------------------------
        // Test: view-direction unit vectors (view_x/y/z) are rotated too,
        // forward and back, not just the XYZ coordinates.
        // ---------------------------------------------------------
        {
            auto pcv = mrpt::maps::CGenericPointsMap::Create();
            pcv->registerField_float("view_x");
            pcv->registerField_float("view_y");
            pcv->registerField_float("view_z");
            pcv->insertPointFast(5.f, 0.f, 0.f);
            pcv->insertPointField_float("view_x", -1.f);
            pcv->insertPointField_float("view_y", 0.f);
            pcv->insertPointField_float("view_z", 0.f);
            pcv->mark_as_modified();

            mp2p_icp::metric_map_t map;
            map.layers["raw"] = pcv;

            const auto poseSeq = mrpt::containers::yaml::Sequence(
                {pose.x(), pose.y(), pose.z(), pose.yaw(), pose.pitch(), pose.roll()});

            FilterTransformPointCloud fwd;
            mrpt::containers::yaml    pf;
            pf["input_pointcloud_layer"]  = "raw";
            pf["output_pointcloud_layer"] = "world";
            pf["pose"]                    = poseSeq;
            fwd.initialize(pf);
            fwd.filter(map);

            auto outFwd = map.layer<mrpt::maps::CGenericPointsMap>("world");
            ASSERT_(outFwd);

            const auto expectedView = pose.rotateVector(mrpt::math::TVector3D(-1, 0, 0));
            ASSERT_NEAR_(
                outFwd->getPointField_float(0, "view_x"), static_cast<float>(expectedView.x),
                1e-4f);
            ASSERT_NEAR_(
                outFwd->getPointField_float(0, "view_y"), static_cast<float>(expectedView.y),
                1e-4f);
            ASSERT_NEAR_(
                outFwd->getPointField_float(0, "view_z"), static_cast<float>(expectedView.z),
                1e-4f);

            FilterTransformPointCloud back;
            mrpt::containers::yaml    pb;
            pb["input_pointcloud_layer"]  = "world";
            pb["output_pointcloud_layer"] = "local_again";
            pb["pose"]                    = poseSeq;
            pb["invert_pose"]             = true;
            back.initialize(pb);
            back.filter(map);

            auto outBack = map.layer<mrpt::maps::CGenericPointsMap>("local_again");
            ASSERT_(outBack);
            ASSERT_NEAR_(outBack->getPointField_float(0, "view_x"), -1.0f, 1e-4f);
            ASSERT_NEAR_(outBack->getPointField_float(0, "view_y"), 0.0f, 1e-4f);
            ASSERT_NEAR_(outBack->getPointField_float(0, "view_z"), 0.0f, 1e-4f);

            std::cout << "[Test Passed] View-direction fields rotated forward and restored on the "
                         "inverse round-trip\n";
        }

        // ---------------------------------------------------------
        // Test: same input/output layer name is rejected
        // ---------------------------------------------------------
        {
            mp2p_icp::metric_map_t map;
            map.layers["raw"] = pc;

            FilterTransformPointCloud filter;
            mrpt::containers::yaml    p;
            p["input_pointcloud_layer"]  = "raw";
            p["output_pointcloud_layer"] = "raw";
            p["pose"] = mrpt::containers::yaml::Sequence({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

            bool threw = false;
            try
            {
                filter.initialize(p);
            }
            catch (const std::exception&)
            {
                threw = true;
            }
            ASSERT_(threw);
            std::cout << "[Test Passed] Same input/output layer name is rejected\n";
        }

        std::cout << "\nFilterTransformPointCloud Unit Tests Passed!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
