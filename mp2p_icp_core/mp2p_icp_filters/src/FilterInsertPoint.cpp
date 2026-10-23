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
 * @file   FilterInsertPoint.cpp
 * @brief  Appends one point, given by (possibly variable) coordinates, to a layer
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#include <mp2p_icp_filters/FilterInsertPoint.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>

IMPLEMENTS_MRPT_OBJECT(FilterInsertPoint, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterInsertPoint::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterInsertPoint& parent)
{
    MCP_LOAD_REQ(c, target_layer);
    ASSERTMSG_(!target_layer.empty(), "'target_layer' must not be empty");

    ASSERTMSG_(
        c.has("point") && c["point"].isSequence() && c["point"].asSequence().size() == 3,
        "'point' must be a sequence of 3 coordinates [x, y, z]");

    const auto pt = c["point"].asSequence();
    for (int i = 0; i < 3; i++)
    {
        parent.parseAndDeclareParameter(pt.at(i).as<std::string>(), point[i]);
    }
}

FilterInsertPoint::FilterInsertPoint()
{
    mrpt::system::COutputLogger::setLoggerName("FilterInsertPoint");
}

void FilterInsertPoint::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterInsertPoint::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    auto out = GetOrCreatePointLayer(inOut, params.target_layer, false /*allow empty*/);
    ASSERT_(out);
    // Only x, y, z are appended: per-point fields of other classes would get out of sync.
    ASSERTMSG_(
        IS_CLASS(*out, mrpt::maps::CSimplePointsMap),
        mrpt::format(
            "Layer '%s' must be a mrpt::maps::CSimplePointsMap", params.target_layer.c_str()));

    out->insertPoint(params.point.x, params.point.y, params.point.z);

    MRPT_END
}
