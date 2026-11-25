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
 * @file   FilterPlaneSegmentation.cpp
 * @brief  Segments the large planes of a point cloud or map
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#include <mp2p_icp_filters/FilterPlaneSegmentation.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>

#include <cstdint>
#include <limits>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(FilterPlaneSegmentation, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterPlaneSegmentation::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterPlaneSegmentation& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, output_layer_planes);
    MCP_LOAD_OPT(c, output_layer_others);
    MCP_LOAD_OPT(c, plane_id_field);
    MCP_LOAD_OPT(c, store_planes);
    MCP_LOAD_OPT(c, clear_previous_planes);

    DECLARE_PARAMETER_IN_OPT(c, voxel_size, parent);
    DECLARE_PARAMETER_IN_OPT(c, normal_knn, parent);
    DECLARE_PARAMETER_IN_OPT(c, normal_radius, parent);
    DECLARE_PARAMETER_IN_OPT(c, normal_agreement_deg, parent);
    DECLARE_PARAMETER_IN_OPT(c, distance_threshold, parent);
    DECLARE_PARAMETER_IN_OPT(c, inlier_distance, parent);
    DECLARE_PARAMETER_IN_OPT(c, max_thickness, parent);
    DECLARE_PARAMETER_IN_OPT(c, min_area, parent);
    DECLARE_PARAMETER_IN_OPT(c, min_points, parent);
    DECLARE_PARAMETER_IN_OPT(c, merge_normal_deg, parent);
    DECLARE_PARAMETER_IN_OPT(c, merge_offset, parent);
    DECLARE_PARAMETER_IN_OPT(c, merge_max_gap, parent);
    MCP_LOAD_OPT(c, parallelization_grain_size);

    if (c.has("viewpoint"))
    {
        ASSERTMSG_(
            c["viewpoint"].isSequence() && c["viewpoint"].asSequence().size() == 3,
            "'viewpoint' must be a sequence [x, y, z]");
        const auto v = c["viewpoint"].asSequence();
        parent.parseAndDeclareParameter(v.at(0).as<std::string>(), viewpoint.x);
        parent.parseAndDeclareParameter(v.at(1).as<std::string>(), viewpoint.y);
        parent.parseAndDeclareParameter(v.at(2).as<std::string>(), viewpoint.z);
    }
}

FilterPlaneSegmentation::FilterPlaneSegmentation()
{
    mrpt::system::COutputLogger::setLoggerName("FilterPlaneSegmentation");
}

void FilterPlaneSegmentation::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterPlaneSegmentation::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    const auto pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));
    auto& pc = *pcPtr;

    const PlaneSegmentationResult res = segment_planes(pc, params);

    MRPT_LOG_DEBUG_STREAM(
        "[" << params.input_pointcloud_layer << "] " << res.num_input_points << " points, "
            << res.num_voxels << " voxels, " << res.num_normals << " normals, " << res.num_regions
            << " regions, " << res.num_merged << " after merging, " << res.planes.size()
            << " planes.");

    if (params.store_planes)
    {
        if (params.clear_previous_planes)
        {
            inOut.planes.clear();
        }
        for (const auto& pl : res.planes)
        {
            inOut.planes.push_back(pl.as_plane_patch());
        }
    }

    const bool wantOutputs = !params.output_layer_planes.empty() ||
                             !params.output_layer_others.empty() || !params.plane_id_field.empty();
    if (!wantOutputs)
    {
        return;
    }

    // Plane id of each input point (0: none):
    const std::size_t N          = pc.size();
    constexpr auto    kMaxPlanes = std::numeric_limits<uint16_t>::max();
    if (res.planes.size() > kMaxPlanes)
    {
        MRPT_LOG_WARN_STREAM(
            "Found " << res.planes.size() << " planes: only the first " << kMaxPlanes
                     << " get a plane id.");
    }
    std::vector<uint16_t> planeId(N, 0);
    for (std::size_t k = 0; k < res.planes.size() && k < kMaxPlanes; k++)
    {
        for (const auto i : res.planes[k].inliers)
        {
            planeId[i] = static_cast<uint16_t>(k + 1);
        }
    }

    // In-place plane id, if there is no planes layer to hold it:
    if (!params.plane_id_field.empty() && params.output_layer_planes.empty())
    {
        if (!pc.hasPointField(params.plane_id_field))
        {
            ASSERTMSG_(
                pc.registerField_uint16(params.plane_id_field),
                mrpt::format(
                    "Layer '%s' (class '%s') does not support extra fields: set "
                    "'output_layer_planes' to hold the plane id instead.",
                    params.input_pointcloud_layer.c_str(), pc.GetRuntimeClass()->className));
        }
        auto* ids = pc.getPointsBufferRef_uint16_field(params.plane_id_field);
        ASSERTMSG_(
            ids && ids->size() == N,
            mrpt::format("Field '%s' is not a uint16 field.", params.plane_id_field.c_str()));
        for (std::size_t i = 0; i < N; i++)
        {
            (*ids)[i] = planeId[i];
        }
    }

    const std::string inputClass = pc.GetRuntimeClass()->className;

    mrpt::maps::CPointsMap::Ptr outPlanes = GetOrCreatePointLayer(
        inOut, params.output_layer_planes, true /*allow empty name for nullptr*/,
        params.plane_id_field.empty() ? inputClass : std::string("mrpt::maps::CGenericPointsMap"));
    mrpt::maps::CPointsMap::Ptr outOthers =
        GetOrCreatePointLayer(inOut, params.output_layer_others, true, inputClass);

    ASSERTMSG_(
        (!outPlanes || outPlanes.get() != &pc) && (!outOthers || outOthers.get() != &pc),
        "Output layers must be different from the input layer.");

    mrpt::maps::CPointsMap::InsertCtx ctxPlanes;
    mrpt::maps::CPointsMap::InsertCtx ctxOthers;
    if (outPlanes)
    {
        outPlanes->registerPointFieldsFrom(pc);
        if (!params.plane_id_field.empty() && !outPlanes->hasPointField(params.plane_id_field))
        {
            ASSERTMSG_(
                outPlanes->registerField_uint16(params.plane_id_field),
                mrpt::format(
                    "Layer '%s' does not support extra fields.",
                    params.output_layer_planes.c_str()));
        }
        ctxPlanes = outPlanes->prepareForInsertPointsFrom(pc);
    }
    if (outOthers)
    {
        outOthers->registerPointFieldsFrom(pc);
        ctxOthers = outOthers->prepareForInsertPointsFrom(pc);
    }

    const std::size_t firstNew = outPlanes ? outPlanes->size() : 0;
    for (std::size_t i = 0; i < N; i++)
    {
        if (planeId[i] != 0)
        {
            if (outPlanes)
            {
                outPlanes->insertPointFrom(i, ctxPlanes);
            }
        }
        else if (outOthers)
        {
            outOthers->insertPointFrom(i, ctxOthers);
        }
    }

    if (outPlanes && !params.plane_id_field.empty())
    {
        auto* ids = outPlanes->getPointsBufferRef_uint16_field(params.plane_id_field);
        ASSERTMSG_(
            ids && ids->size() == outPlanes->size(),
            mrpt::format("Field '%s' is not a uint16 field.", params.plane_id_field.c_str()));
        std::size_t k = firstNew;
        for (std::size_t i = 0; i < N; i++)
        {
            if (planeId[i] != 0)
            {
                (*ids)[k++] = planeId[i];
            }
        }
    }

    MRPT_END
}
