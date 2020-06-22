/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_Base.h
 * @brief  Pointcloud matcher auxiliary class for iterating over point layers.
 * @author Jose Luis Blanco Claraco
 * @date   June 25, 2020
 */
#pragma once

#include <mp2p_icp/Matcher.h>

namespace mp2p_icp
{
/** Pointcloud matcher auxiliary class for iterating over point layers.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_Base : public Matcher
{
   public:
    Matcher_Points_Base() = default;

    void match(
        const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
        const mrpt::poses::CPose3D& localPose,
        Pairings&                   out) const override final;

    /** Weights for each layer. If empty, the output Pairings::point_weights
     * will left empty (=all points have equal weight).
     * \note Note: this field can be loaded from a configuration file via
     * initializeLayerWeights().
     */
    std::map<std::string, double> weight_pt2pt_layers;

    /** Call with a dictionary element, with keys=layer names, values (double)
     * the weights. */
    void initializeLayerWeights(const mrpt::containers::Parameters& p);

   private:
    virtual void implMatchOneLayer(
        const mrpt::maps::CPointsMap& pcGlobal,
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, Pairings& out) const = 0;
};

}  // namespace mp2p_icp
