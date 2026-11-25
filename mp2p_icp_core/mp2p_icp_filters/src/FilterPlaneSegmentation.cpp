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
#include <mrpt/containers/yaml.h>
#include <mrpt/rtti/CObject.h>

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

    // Check the configuration before changing anything in the map:
    const auto existingLayer = [&](const std::string& layerName) -> mrpt::maps::CPointsMap::Ptr
    {
        if (layerName.empty())
        {
            return {};
        }
        const auto it = inOut.layers.find(layerName);
        if (it == inOut.layers.end())
        {
            return {};
        }
        auto ptr = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(it->second);
        ASSERTMSG_(
            ptr,
            mrpt::format("Output layer '%s' exists and is not a point cloud.", layerName.c_str()));
        return ptr;
    };
    const auto exPlanes = existingLayer(params.output_layer_planes);
    const auto exOthers = existingLayer(params.output_layer_others);
    ASSERTMSG_(
        params.output_layer_planes != params.input_pointcloud_layer &&
            params.output_layer_others != params.input_pointcloud_layer &&
            (!exPlanes || exPlanes.get() != &pc) && (!exOthers || exOthers.get() != &pc),
        "Output layers must be different from the input layer.");
    ASSERTMSG_(
        params.output_layer_planes.empty() ||
            (params.output_layer_planes != params.output_layer_others &&
             (!exPlanes || exPlanes != exOthers)),
        "'output_layer_planes' and 'output_layer_others' must be different layers.");

    const PlaneSegmentationResult res = segment_planes(pc, params);

    MRPT_LOG_DEBUG_STREAM(
        "[" << params.input_pointcloud_layer << "] " << res.num_input_points << " points, "
            << res.num_voxels << " voxels, " << res.num_normals << " normals, " << res.num_regions
            << " regions, " << res.num_merged << " after merging, " << res.planes.size()
            << " planes.");

    const bool wantIds = !params.plane_id_field.empty();
    ASSERTMSG_(
        !wantIds || res.planes.size() <= std::numeric_limits<uint16_t>::max(),
        mrpt::format(
            "Found %zu planes, more than a uint16 plane id field can hold.", res.planes.size()));

    // Plane of each input point (kNone: none):
    constexpr uint32_t    kNone = std::numeric_limits<uint32_t>::max();
    const std::size_t     N     = pc.size();
    std::vector<uint32_t> planeOf;
    const bool            wantOutputs =
        wantIds || !params.output_layer_planes.empty() || !params.output_layer_others.empty();
    if (wantOutputs)
    {
        planeOf.assign(N, kNone);
        for (std::size_t k = 0; k < res.planes.size(); k++)
        {
            for (const auto i : res.planes[k].inliers)
            {
                planeOf[i] = static_cast<uint32_t>(k);
            }
        }
    }

    // Output layers: the existing ones, or new ones, added to the map only
    // once everything has been checked.
    const std::string inputClass   = pc.GetRuntimeClass()->className;
    const auto        prepareLayer = [&](const std::string&                 layerName,
                                  const mrpt::maps::CPointsMap::Ptr& existing,
                                  const std::string& cls) -> mrpt::maps::CPointsMap::Ptr
    {
        if (layerName.empty())
        {
            return {};
        }
        mrpt::maps::CPointsMap::Ptr out = existing;
        if (!out)
        {
            out = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mrpt::rtti::classFactory(cls));
            ASSERTMSG_(out, mrpt::format("Could not create a layer of class '%s'.", cls.c_str()));
        }
        ASSERTMSG_(
            out->registerPointFieldsFrom(pc),
            mrpt::format(
                "Output layer '%s' (class '%s') cannot hold all the fields of the input layer.",
                layerName.c_str(), out->GetRuntimeClass()->className));
        return out;
    };
    const auto outPlanes = prepareLayer(
        params.output_layer_planes, exPlanes,
        wantIds ? std::string("mrpt::maps::CGenericPointsMap") : inputClass);
    const auto outOthers = prepareLayer(params.output_layer_others, exOthers, inputClass);

    // Where the plane ids go: the planes layer, or the input layer itself.
    mrpt::maps::CPointsMap* idTarget = nullptr;
    if (wantIds)
    {
        idTarget = outPlanes ? outPlanes.get() : &pc;
        if (!idTarget->hasPointField(params.plane_id_field))
        {
            ASSERTMSG_(
                idTarget->registerField_uint16(params.plane_id_field),
                mrpt::format(
                    "Layer of class '%s' does not support extra fields: set "
                    "'output_layer_planes' to hold the plane id field instead.",
                    idTarget->GetRuntimeClass()->className));
        }
        ASSERTMSG_(
            idTarget->getPointsBufferRef_uint16_field(params.plane_id_field) != nullptr,
            mrpt::format("Field '%s' is not a uint16 field.", params.plane_id_field.c_str()));
    }

    // Plane ids are 1-based, in the order of this call's planes; 0 is none.
    const auto idOf = [&](std::size_t i)
    { return planeOf[i] == kNone ? uint16_t{0} : static_cast<uint16_t>(planeOf[i] + 1); };

    if (wantIds && !outPlanes)
    {
        auto* ids = pc.getPointsBufferRef_uint16_field(params.plane_id_field);
        ASSERT_EQUAL_(ids->size(), N);
        for (std::size_t i = 0; i < N; i++)
        {
            (*ids)[i] = idOf(i);
        }
    }

    if (outPlanes || outOthers)
    {
        mrpt::maps::CPointsMap::InsertCtx ctxPlanes;
        mrpt::maps::CPointsMap::InsertCtx ctxOthers;
        if (outPlanes)
        {
            ctxPlanes = outPlanes->prepareForInsertPointsFrom(pc);
        }
        if (outOthers)
        {
            ctxOthers = outOthers->prepareForInsertPointsFrom(pc);
        }
        const std::size_t firstNew = outPlanes ? outPlanes->size() : 0;
        for (std::size_t i = 0; i < N; i++)
        {
            if (planeOf[i] != kNone)
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
        if (outPlanes && wantIds)
        {
            auto* ids = outPlanes->getPointsBufferRef_uint16_field(params.plane_id_field);
            ASSERT_EQUAL_(ids->size(), outPlanes->size());
            std::size_t k = firstNew;
            for (std::size_t i = 0; i < N; i++)
            {
                if (planeOf[i] != kNone)
                {
                    (*ids)[k++] = idOf(i);
                }
            }
        }
        if (outPlanes && !exPlanes)
        {
            inOut.layers[params.output_layer_planes] = outPlanes;
        }
        if (outOthers && !exOthers)
        {
            inOut.layers[params.output_layer_others] = outOthers;
        }
    }

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

    MRPT_END
}
