/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_Voxels.h
 * @brief  Matching quality evaluator: comparison via voxel occupancy
 * @author Jose Luis Blanco Claraco
 * @date   July 14, 2020
 */
#pragma once

#include <mp2p_icp/QualityEvaluator.h>

namespace mp2p_icp
{
/** Matching quality evaluator: comparison via voxel occupancy.
 *
 * \ingroup mp2p_icp_grp
 */
class QualityEvaluator_Voxels : public QualityEvaluator
{
    DEFINE_MRPT_OBJECT(QualityEvaluator_Voxels, mp2p_icp)

   public:
    QualityEvaluator_Voxels();

    // See base class
    void   initialize(const mrpt::containers::yaml& params) override;
    double evaluate(
        const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
        const mrpt::poses::CPose3D& localPose,
        const Pairings&             finalPairings) const override;

    double resolution                  = 0.25;  //!< voxel size [meters]
    double maxOccupancyUpdateCertainty = 0.65;  //! <[0.5,1.0]
    double maxFreenessUpdateCertainty  = 0.55;
    double dist2quality_scale          = 0.1;

    /** Evaluate points only in these layers */
    std::set<std::string> pointLayers = {mp2p_icp::pointcloud_t::PT_LAYER_RAW};
};

}  // namespace mp2p_icp
