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
 * @file   CreateMetricMapFromDefinition.cpp
 * @brief  Auxiliary function CreateMetricMapFromDefinition
 * @author Jose Luis Blanco Claraco
 * @date   Aug 25, 2026
 */

#include <mp2p_icp/load_plugin.h>
#include <mp2p_icp_filters/CreateMetricMapFromDefinition.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/system/filesystem.h>

mrpt::maps::CMetricMap::Ptr mp2p_icp_filters::CreateMetricMapFromDefinition(
    const mrpt::containers::yaml& metricMapDefinition,
    const std::string& metricMapDefinitionIniFile, const mrpt::system::COutputLogger* logger)
{
    using namespace std::string_literals;

    ASSERTMSG_(
        !metricMapDefinition.empty() || !metricMapDefinitionIniFile.empty(),
        "Either metricMapDefinition or metricMapDefinitionIniFile must be provided.");

    const std::string cfgPrefix = "map"s;

    mrpt::maps::TSetOfMetricMapInitializers mapInits;

    if (!metricMapDefinitionIniFile.empty())
    {
        // Load from an external INI file:
        ASSERT_FILE_EXISTS_(metricMapDefinitionIniFile);
        mrpt::config::CConfigFile cfg(metricMapDefinitionIniFile);
        mapInits.loadFromConfigFile(cfg, cfgPrefix);
    }
    else
    {
        // Build an in-memory INI file with the structure expected by
        // TSetOfMetricMapInitializers::loadFromConfigFile():
        ASSERT_(!metricMapDefinition.empty());
        const auto& c = metricMapDefinition;

        mrpt::config::CConfigFileMemory cfg;

        ASSERT_(c.has("class"));
        const std::string mapClass = c["class"].as<std::string>();
        cfg.write(cfgPrefix, mapClass + "_count"s, "1");

        // optional plugin module?
        if (c.has("plugin"))
        {
            const auto moduleToLoad = c["plugin"].as<std::string>();
            mp2p_icp::load_plugin(moduleToLoad, logger);
        }

        // fill the rest of the sub-sections (creationOpts, insertOpts, ...):
        for (const auto& [k, v] : c.asMap())
        {
            if (!v.isMap())
            {
                continue;
            }
            const auto keyVal   = k.as<std::string>();
            const auto sectName = cfgPrefix + "_"s + mapClass + "_00_"s + keyVal;
            for (const auto& [kk, vv] : v.asMap())
            {
                ASSERT_(kk.isScalar());
                ASSERT_(vv.isScalar());

                cfg.write(sectName, kk.as<std::string>(), vv.as<std::string>());
            }
        }

        mapInits.loadFromConfigFile(cfg, cfgPrefix);
    }

    mrpt::maps::CMultiMetricMap theMap;
    theMap.setListOfMaps(mapInits);

    ASSERT_(theMap.maps.size() >= 1);
    return theMap.maps.at(0);
}
