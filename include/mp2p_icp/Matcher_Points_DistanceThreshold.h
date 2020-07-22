/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Points_DistanceThreshold.h
 * @brief  Pointcloud matcher: fixed distance thresholds
 * @author Jose Luis Blanco Claraco
 * @date   June 22, 2020
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: fixed distance thresholds
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Points_DistanceThreshold : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Points_DistanceThreshold, mp2p_icp);

   public:
    Matcher_Points_DistanceThreshold();

    /*** Parameters:
     * - `threshold`: Inliers distance threshold [meters][mandatory]
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::Parameters& params) override;

   private:
    double threshold_ = 0.50;

    void implMatchOneLayer(
        const mrpt::maps::CPointsMap& pcGlobal,
        const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, Pairings& out) const override;
};

}  // namespace mp2p_icp
