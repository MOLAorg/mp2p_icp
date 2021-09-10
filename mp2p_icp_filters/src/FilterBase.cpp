/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBase.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/FilterBase.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(
    FilterBase, mrpt::rtti::CObject, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterBase::FilterBase() : mrpt::system::COutputLogger("FilterBase") {}

FilterBase::~FilterBase() = default;

void mp2p_icp_filters::apply_filter_pipeline(
    const FilterPipeline& filters, mp2p_icp::pointcloud_t& inOut)
{
    for (const auto& f : filters)
    {
        ASSERT_(f.get() != nullptr);
        f->filter(inOut);
    }
}

FilterPipeline mp2p_icp_filters::filter_pipeline_from_yaml(
    const mrpt::containers::yaml& c)
{
    ASSERT_(c.isSequence());

    FilterPipeline filters;

    return filters;
}

FilterPipeline mp2p_icp_filters::filter_pipeline_from_yaml_file(
    const std::string& filename)
{
    const auto yamlContent = mrpt::containers::yaml::FromFile(filename);

    ASSERT_(yamlContent.has("filters") && yamlContent["filters"].isSequence());

    return filter_pipeline_from_yaml(yamlContent["filters"]);
}
