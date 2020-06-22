/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_Base.cpp
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */

#include <mp2p_icp/Matcher_Points_Base.h>

using namespace mp2p_icp;

void Matcher_Points_Base::match(
    const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, Pairings& out) const
{
    MRPT_START

    out = Pairings();

    // Analyze point cloud layers, one by one:
    for (const auto& glLayerKV : pcGlobal.point_layers)
    {
        const auto& name = glLayerKV.first;

        if (!weight_pt2pt_layers.empty() &&
            weight_pt2pt_layers.count(name) == 0)
            // If we have weights and this layer is not listed, Skip it:
            continue;

        // Look for a matching layer in "local":
        auto itLocal = pcLocal.point_layers.find(name);
        if (itLocal == pcLocal.point_layers.end()) continue;

        const mrpt::maps::CPointsMap::Ptr& glLayer = glLayerKV.second;
        const mrpt::maps::CPointsMap::Ptr& lcLayer = itLocal->second;

        ASSERT_(glLayer);
        ASSERT_(lcLayer);

        const size_t nBefore = out.paired_points.size();

        implMatchOneLayer(*glLayer, *lcLayer, localPose, out);

        const size_t nAfter = out.paired_points.size();

        if (!weight_pt2pt_layers.empty() && nAfter != nBefore)
        {
            const double w = weight_pt2pt_layers.at(name);
            out.point_weights.emplace_back(nAfter - nBefore, w);
        }
    }
    MRPT_END
}

void Matcher_Points_Base::initializeLayerWeights(
    const mrpt::containers::Parameters& p)
{
    weight_pt2pt_layers.clear();
    ASSERT_(p.isMap());

    for (const auto& kv : p.asMap())
    {
        const std::string ly = kv.first;
        const double      w  = std::get<double>(kv.second);

        weight_pt2pt_layers[ly] = w;
    }
}
