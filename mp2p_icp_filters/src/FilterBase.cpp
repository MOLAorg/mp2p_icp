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

#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/system/CTimeLogger.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(
    FilterBase, mrpt::rtti::CObject, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterBase::FilterBase() : mrpt::system::COutputLogger("FilterBase") {}

FilterBase::~FilterBase() = default;

void mp2p_icp_filters::apply_filter_pipeline(
    const FilterPipeline& filters, mp2p_icp::metric_map_t& inOut,
    const mrpt::optional_ref<mrpt::system::CTimeLogger>& profiler)
{
    for (const auto& f : filters)
    {
        ASSERT_(f.get() != nullptr);

        std::optional<mrpt::system::CTimeLoggerEntry> tle;
        if (profiler) tle.emplace(*profiler, f->GetRuntimeClass()->className);

        f->filter(inOut);
    }
}

FilterPipeline mp2p_icp_filters::filter_pipeline_from_yaml(
    const mrpt::containers::yaml& c, const mrpt::system::VerbosityLevel& vLevel)
{
    if (c.isNullNode()) return {};

    ASSERT_(c.isSequence());

    FilterPipeline filters;

    for (const auto& entry : c.asSequence())
    {
        const auto& e = entry.asMap();

        const auto sClass = e.at("class_name").as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto f = std::dynamic_pointer_cast<FilterBase>(o);
        ASSERTMSG_(
            f, mrpt::format(
                   "`%s` class seems not to be derived from FilterBase",
                   sClass.c_str()));

        f->setMinLoggingLevel(vLevel);

        f->initialize(e.at("params"));
        filters.push_back(f);
    }

    return filters;
}

FilterPipeline mp2p_icp_filters::filter_pipeline_from_yaml_file(
    const std::string& filename, const mrpt::system::VerbosityLevel& vLevel)
{
    const auto yamlContent = mrpt::containers::yaml::FromFile(filename);

    ASSERT_(yamlContent.has("filters") && yamlContent["filters"].isSequence());

    return filter_pipeline_from_yaml(yamlContent["filters"], vLevel);
}

mrpt::maps::CPointsMap* FilterBase::GetOrCreatePointLayer(
    mp2p_icp::metric_map_t& m, const std::string& layerName,
    bool allowEmptyName, const std::string& classForLayerCreation)
{
    mrpt::maps::CPointsMap* outPc = nullptr;

    if (layerName.empty())
    {
        if (allowEmptyName)
            return nullptr;
        else
            THROW_EXCEPTION("Layer name cannot be empty");
    }

    if (auto itLy = m.layers.find(layerName); itLy != m.layers.end())
    {
        outPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!outPc)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.", layerName.c_str());
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

        outPc               = newMap.get();
        m.layers[layerName] = newMap;
    }
    return outPc;
}
