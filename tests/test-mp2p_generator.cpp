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
 * @file   test-mp2p_generator.cpp
 * @brief  Unit tests for Generator
 * @author Jose Luis Blanco Claraco
 * @date   Dec 28, 2025
 */

#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <iostream>  // cerr

namespace
{
void test_generator_create_map()
{
    constexpr const char* yaml_content = R"(
# --------------------------------------------------------
# 1) Generator (observation -> local frame metric maps)
# --------------------------------------------------------
generators:
  # This first generator is used to just create the metric map "gridmap" once:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: "voxelmap"
      # The default '.*' is replaced by '' (none): do not insert directly any observation,
      # since we want to insert them after decimation
      process_class_names_regex: ""
      metric_map_definition:
        # Any class derived from mrpt::maps::CMetricMap https://docs.mrpt.org/reference/latest/group_mrpt_maps_grp.html
        class: mrpt::maps::CVoxelMap
        #plugin: 'libmola_metric_maps.so' # Import additional custom user-defined map classes (search in LD_LIBRARY_PATH)
        creationOpts:
          resolution: 0.20 # [m]
          #resolution: $f{0.05*MAX_SENSOR_RANGE} # [m]  # You can also use formulas in any numeric field
        #insertionOpts:
        # none required for this class
        likelihoodOpts:
          decimation: 1
          occupiedThreshold: 0.51
        renderOpts:
          occupiedThreshold: 0.51
          freeThreshold: 0.40
          generateFreeVoxels: false

  # Default generator: convert all observations into a point cloud layer "raw":
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: "raw"
      throw_on_unhandled_observation_class: true
      process_class_names_regex: ".*"
      #process_sensor_labels_regex: '.*'

    )";

    const auto generators = mp2p_icp_filters::generators_from_yaml(
        mrpt::containers::yaml::FromText(yaml_content)["generators"]);

    ASSERT_EQUAL_(generators.size(), 2UL);

    mrpt::obs::CObservationPointCloud obs;
    {
        auto pts = std::make_shared<mrpt::maps::CSimplePointsMap>();
        pts->insertPoint(1.0f, 2.0f, 3.0f);
        pts->insertPoint(4.0f, 5.0f, 6.0f);
        pts->insertPoint(7.0f, 8.0f, 9.0f);
        obs.pointcloud = pts;
    }

    const auto map = mp2p_icp_filters::apply_generators(generators, obs);

    ASSERT_EQUAL_(map.layers.size(), 2UL);

    auto layerMap = map.layer<mrpt::maps::CPointsMap>("raw");
    ASSERT_(layerMap);
    ASSERT_EQUAL_(layerMap->size(), 3UL);

    auto voxelMap = map.layer<mrpt::maps::CVoxelMap>("voxelmap");
    ASSERT_(voxelMap);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_generator_create_map();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
