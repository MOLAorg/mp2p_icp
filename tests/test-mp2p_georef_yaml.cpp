/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_georef_yaml.cpp
 * @brief  Unit tests for georeference <-> yaml conversion
 * @author Jose Luis Blanco Claraco
 * @date   Apr 3, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>

#include <sstream>

namespace
{
void unit_test()
{
    {
        mp2p_icp::metric_map_t map;

        std::stringstream ss;
        ss << mp2p_icp::ToYAML(map.georeferencing);
        const auto str = ss.str();

        const auto gRead = mp2p_icp::FromYAML(mrpt::containers::yaml::FromText(str));
        ASSERT_EQUAL_(gRead.has_value(), map.georeferencing.has_value());
    }

    {
        mp2p_icp::metric_map_t map;
        auto&                  gIn = map.georeferencing.emplace();
        gIn.geo_coord.lat          = 4.2;
        gIn.geo_coord.lon          = 23.0;
        gIn.geo_coord.height       = 100.0;

        gIn.T_enu_to_map.cov.setDiagonal(0.1);
        gIn.T_enu_to_map.mean = mrpt::poses::CPose3D(1.0, 2.0, 3.0, 0, 0, 0);

        std::stringstream ss;
        ss << mp2p_icp::ToYAML(map.georeferencing);
        const auto str = ss.str();

        const auto gRead = mp2p_icp::FromYAML(mrpt::containers::yaml::FromText(str));
        ASSERT_EQUAL_(gRead.has_value(), map.georeferencing.has_value());
        ASSERT_(gRead->T_enu_to_map.mean == map.georeferencing->T_enu_to_map.mean);
        ASSERT_(gRead->T_enu_to_map.cov == map.georeferencing->T_enu_to_map.cov);
        ASSERT_(gRead->geo_coord == map.georeferencing->geo_coord);
    }
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
