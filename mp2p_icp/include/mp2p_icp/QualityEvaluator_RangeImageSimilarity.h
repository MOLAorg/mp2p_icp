/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_RangeImageSimilarity.h
 * @brief  Matching quality evaluator from paper [Bogoslavskyi,IROS2017]
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */
#pragma once

#include <mp2p_icp/QualityEvaluator.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixDynamic.h>

namespace mp2p_icp
{
/** Matching quality evaluator: simple ratio [0,1] of paired entities.
 *
 * Implementation of the method in the papper:
 * - "Analyzing the Quality of Matched 3D Point Clouds of Objects", Igor
 * Bogoslavskyi, Cyrill Stachniss, IROS 2017.
 *
 * This implementation reprojects points into range images with the resolution
 * defined in parameters and compares the likelihood of both range images to be
 * generated from the same point clouds.
 *
 * At present, the PT_LAYER_RAW layer of points is the only one that is used to
 * evaluate the depth images.
 *
 * \ingroup mp2p_icp_grp
 */
class QualityEvaluator_RangeImageSimilarity : public QualityEvaluator
{
    DEFINE_MRPT_OBJECT(QualityEvaluator_RangeImageSimilarity, mp2p_icp)

   public:
    /** See base class. Parameters:
     *
     * \code
     * ncols: 200
     * nrows: 100
     * cx: 100
     * cy: 50
     * fx: 50
     * fy: 50
     * sigma: 0.1
     * #debug_show_all_in_window: false
     * \endcode
     */
    void initialize(const mrpt::containers::yaml& params) override;

    /** See base class.
     * This implementation does NOT use `pairingsFromICP` at all, it can be
     * empty.
     */
    Result evaluate(
        const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
        const mrpt::poses::CPose3D& localPose,
        const Pairings&             pairingsFromICP) const override;

    /** Parameters for the simulated camera */
    mrpt::img::TCamera rangeCamera;

    double sigma = 0.1;  ///!< Uncertainty of depth ranges [meters]

    /** Penalty for pixels only visible from one view point [in "sigmas"] */
    double penalty_not_visible = 2.0;

    bool debug_show_all_in_window = false;
    bool debug_save_all_matrices  = false;

    mrpt::math::CMatrixDouble projectPoints(
        const mrpt::maps::CPointsMap&              pts,
        const std::optional<mrpt::poses::CPose3D>& relativePose =
            std::nullopt) const;

    std::vector<double> scores(
        const mrpt::math::CMatrixDouble& m1,
        const mrpt::math::CMatrixDouble& m2) const;
};

}  // namespace mp2p_icp
