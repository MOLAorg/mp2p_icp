/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBase.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/GetOrCreatePointLayer.h>

mrpt::maps::CPointsMap::Ptr mp2p_icp_filters::GetOrCreatePointLayer(
    mp2p_icp::metric_map_t& m, const std::string& layerName, bool allowEmptyName,
    const std::string& classForLayerCreation)
{
    mrpt::maps::CPointsMap::Ptr outPc;

    if (layerName.empty())
    {
        if (allowEmptyName)
            return nullptr;
        else
            THROW_EXCEPTION("Layer name cannot be empty");
    }

    if (auto itLy = m.layers.find(layerName); itLy != m.layers.end())
    {
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itLy->second);
        if (!outPc)
            THROW_EXCEPTION_FMT("Layer '%s' must be of point cloud type.", layerName.c_str());
    }
    else
    {
        auto o = mrpt::rtti::classFactory(classForLayerCreation);
        ASSERTMSG_(
            o, mrpt::format(
                   "Could not create layer of type '%s' (wrong or "
                   "unregistered class name?)",
                   classForLayerCreation.c_str()));

        auto newMap = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(o);
        ASSERTMSG_(
            newMap, mrpt::format(
                        "Provided class name '%s' seems not to be derived from "
                        "mrpt::maps::CPointsMap",
                        classForLayerCreation.c_str()));

        outPc               = newMap;
        m.layers[layerName] = newMap;
    }
    return outPc;
}
