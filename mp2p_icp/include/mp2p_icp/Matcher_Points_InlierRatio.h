/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_InlierRatio.h
 * @brief  Pointcloud matcher: fixed ratio of inliers/outliers by distance
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: fixed ratio of inliers/outliers by distance
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_InlierRatio : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Points_InlierRatio, mp2p_icp)

   public:
    Matcher_Points_InlierRatio();
    Matcher_Points_InlierRatio(const double ratio)
        : Matcher_Points_InlierRatio()
    {
        inliersRatio = ratio;
    }

    /*** Parameters:
     * `inliersRatio`: Inliers distance ratio threshold [0-1]
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    /** Inliers distance ratio threshold [0-1] */
    double inliersRatio = 0.80;

    void implMatchOneLayer(
        const mrpt::maps::CPointsMap& pcGlobal,
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms,
        const layer_name_t& globalName, const layer_name_t& localName,
        Pairings& out) const override;
};

}  // namespace mp2p_icp
