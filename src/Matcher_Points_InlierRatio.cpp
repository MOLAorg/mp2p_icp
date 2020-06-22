/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_InlierRatio.cpp
 * @brief  Pointcloud matcher: fixed ratio of inliers/outliers by distance
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */

#include <mp2p_icp/Matcher_Points_InlierRatio.h>
#include <mrpt/core/exceptions.h>

IMPLEMENTS_MRPT_OBJECT(Matcher_Points_InlierRatio, Matcher, mp2p_icp);

using namespace mp2p_icp;

Matcher_Points_InlierRatio::Matcher_Points_InlierRatio()
{
    mrpt::system::COutputLogger::setLoggerName("Matcher_Points_InlierRatio");
}

void Matcher_Points_InlierRatio::initialize(
    const mrpt::containers::Parameters& params)
{
    inliersRatio_ = params["inliersRatio"];

    if (params.has("pointLayerWeights"))
        initializeLayerWeights(params["pointLayerWeights"]);
}

void Matcher_Points_InlierRatio::implMatchOneLayer(
    const mrpt::maps::CPointsMap& pcGlobal,
    const mrpt::maps::CPointsMap& pcLocal,
    const mrpt::poses::CPose3D& localPose, Pairings& out) const
{
    //
}