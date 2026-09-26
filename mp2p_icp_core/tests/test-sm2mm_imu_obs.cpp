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
 * @file   test-sm2mm_imu_obs.cpp
 * @brief  sm2mm() on keyframes that carry IMU observations next to the point cloud
 * @author Jose Luis Blanco Claraco
 * @date   Sep 2026
 *
 * IMU readings are "handled" by the generators (they feed the velocity buffer) but
 * produce no map layer. The per-frame filter pipeline must therefore not run for
 * them: a pipeline that consumes and then deletes the "raw" layer would find it
 * missing and throw.
 */

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <iostream>

namespace
{
constexpr size_t NUM_POINTS    = 10;
constexpr size_t NUM_KEYFRAMES = 3;

mrpt::obs::CObservationPointCloud::Ptr buildPointCloudObs(const mrpt::Clock::time_point& t)
{
    auto obs = mrpt::obs::CObservationPointCloud::Create();
    auto pc  = mrpt::maps::CSimplePointsMap::Create();
    for (size_t i = 0; i < NUM_POINTS; i++)
    {
        pc->insertPointFast(5.0f + static_cast<float>(i), 0.0f, 0.0f);
    }
    obs->pointcloud  = pc;
    obs->sensorLabel = "lidar";
    obs->timestamp   = t;
    return obs;
}

mrpt::obs::CObservationIMU::Ptr buildImuObs(const mrpt::Clock::time_point& t)
{
    auto obs         = mrpt::obs::CObservationIMU::Create();
    obs->sensorLabel = "imu";
    obs->timestamp   = t;
    obs->set(mrpt::obs::IMU_WX, 0.0);
    obs->set(mrpt::obs::IMU_WY, 0.0);
    obs->set(mrpt::obs::IMU_WZ, 0.1);
    obs->set(mrpt::obs::IMU_X_ACC, 0.0);
    obs->set(mrpt::obs::IMU_Y_ACC, 0.0);
    obs->set(mrpt::obs::IMU_Z_ACC, 9.81);
    return obs;
}

// Same observation order the LiDAR odometry module writes: point cloud first, IMU after it.
mrpt::maps::CSimpleMap buildSimpleMap()
{
    mrpt::maps::CSimpleMap sm;
    for (size_t k = 0; k < NUM_KEYFRAMES; k++)
    {
        const auto t = mrpt::Clock::fromDouble(1000.0 + 0.1 * static_cast<double>(k));

        auto posePDF  = mrpt::poses::CPose3DPDFGaussian::Create();
        posePDF->mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            static_cast<double>(k), 0.0, 0.0, 0.0, 0.0, 0.0);

        auto sf = mrpt::obs::CSensoryFrame::Create();
        sf->insert(buildPointCloudObs(t));
        sf->insert(buildImuObs(t));
        sm.insert(posePDF, sf);
    }
    return sm;
}

void test_sm2mm_KeyframesWithImu()
{
    std::cout << "Testing sm2mm with IMU observations in keyframes... ";

    const auto pipeline = mrpt::containers::yaml::FromText(R"(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: raw

filters:
  - class_name: mp2p_icp_filters::FilterByRange
    params:
      input_pointcloud_layer: raw
      output_layer_outside: filtered
      range_min: 0.0
      range_max: 1.0
      center: [robot_x, robot_y, robot_z]

  - class_name: mp2p_icp_filters::FilterDeleteLayer
    params:
      pointcloud_layer_to_remove: [raw]
)");

    mp2p_icp_filters::sm2mm_options_t opts;
    opts.showProgressBar = false;
    opts.verbosity       = mrpt::system::LVL_WARN;

    mp2p_icp::metric_map_t mm;
    mp2p_icp_filters::simplemap_to_metricmap(buildSimpleMap(), mm, pipeline, opts);

    ASSERT_EQUAL_(mm.layers.count("raw"), 0U);
    ASSERT_EQUAL_(mm.point_layer("filtered")->size(), NUM_POINTS * NUM_KEYFRAMES);

    std::cout << "Success ✅\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_sm2mm_KeyframesWithImu();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
