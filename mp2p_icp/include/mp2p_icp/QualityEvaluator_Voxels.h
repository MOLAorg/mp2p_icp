/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
    Result evaluate(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose,
        const Pairings&             pairingsFromICP) const override;

    /** The name of the input maps layer that is of type CVoxelMap */
    std::string voxel_layer_name;
    double      dist2quality_scale = 2.0;
};

}  // namespace mp2p_icp
