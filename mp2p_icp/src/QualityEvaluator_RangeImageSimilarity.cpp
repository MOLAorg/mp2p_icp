/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   QualityEvaluator_RangeImageSimilarity.cpp
 * @brief  Matching quality evaluator from paper [Bogoslavskyi,IROS2017]
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */

#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/vector_loadsave.h>

IMPLEMENTS_MRPT_OBJECT(QualityEvaluator_RangeImageSimilarity, QualityEvaluator, mp2p_icp)

using namespace mp2p_icp;

void QualityEvaluator_RangeImageSimilarity::initialize(const mrpt::containers::yaml& params)
{
    rangeCamera.ncols = params["ncols"].as<uint32_t>();
    rangeCamera.nrows = params["nrows"].as<uint32_t>();

    rangeCamera.cx(params["cx"].as<double>());
    rangeCamera.cy(params["cy"].as<double>());
    rangeCamera.fx(params["fx"].as<double>());
    rangeCamera.fy(params["fy"].as<double>());

    MCP_LOAD_OPT(params, sigma);
    MCP_LOAD_OPT(params, penalty_not_visible);

    MCP_LOAD_OPT(params, debug_show_all_in_window);
    MCP_LOAD_OPT(params, debug_save_all_matrices);
}

QualityEvaluator::Result QualityEvaluator_RangeImageSimilarity::evaluate(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D& localPose, [[maybe_unused]] const Pairings& pairingsFromICP) const
{
    // See Figure 3 of IROS2017 paper:
    // "Analyzing the Quality of Matched 3D Point Clouds of Objects"
    // Igor Bogoslavskyi, Cyrill Stachniss

    const auto& p1 = *pcGlobal.point_layer(metric_map_t::PT_LAYER_RAW);
    const auto& p2 = *pcLocal.point_layer(metric_map_t::PT_LAYER_RAW);

    const auto I11 = projectPoints(p1);
    const auto I12 = projectPoints(p1, localPose);
    const auto I22 = projectPoints(p2);
    const auto I21 = projectPoints(p2, -localPose);

    auto s1 = scores(I11, I21);
    auto s2 = scores(I12, I22);

    const size_t nScores = s1.size() + s2.size();

    double sum = .0;
    for (double v : s1) sum += v;
    for (double v : s2) sum += v;

    const double finalScore = nScores > 0 ? sum / nScores : .0;

    // ----- Debug ----------
    if (debug_show_all_in_window)
    {
        mrpt::gui::CDisplayWindow win_11("I11", 400, 300);
        mrpt::gui::CDisplayWindow win_12("I12", 400, 300);
        mrpt::gui::CDisplayWindow win_21("I21", 400, 300);
        mrpt::gui::CDisplayWindow win_22("I22", 400, 300);
        win_11.setPos(5, 5);
        win_12.setPos(410 + 5, 5);
        win_21.setPos(5, 310 + 5);
        win_22.setPos(410 + 5, 310 + 5);
        mrpt::img::CImage im11, im12, im21, im22;
        im11.setFromMatrix(I11, false);
        win_11.showImage(im11);
        im12.setFromMatrix(I12, false);
        win_12.showImage(im12);
        im21.setFromMatrix(I21, false);
        win_21.showImage(im21);
        im22.setFromMatrix(I22, false);
        win_22.showImage(im22);
        win_11.waitForKey();
    }
    if (debug_save_all_matrices)
    {
        static std::atomic_int iv = 0;
        const int              i  = iv++;
        I11.saveToTextFile(mrpt::format("I11_%05i.txt", i));
        I22.saveToTextFile(mrpt::format("I22_%05i.txt", i));
        I12.saveToTextFile(mrpt::format("I12_%05i.txt", i));
        I21.saveToTextFile(mrpt::format("I21_%05i.txt", i));

        mrpt::io::vectorToTextFile(s1, mrpt::format("I1_scores_%05i.txt", i));
        mrpt::io::vectorToTextFile(s2, mrpt::format("I2_scores_%05i.txt", i));
    }

    Result r;
    r.quality = finalScore;
    return r;
}

// Adapted from mrpt::vision::pinhole::projectPoint_with_distortion()
// 3-claused BSD
static void projectPoint(
    const mrpt::math::TPoint3D& P, const mrpt::img::TCamera& params, double& pixel_x,
    double& pixel_y)
{
    /* Pinhole model.
     *
     * Point reference            Pixel/camera reference
     *
     *     +Z ^                           / +Z
     *        |  /                       /
     *        | /  +X                   /
     *  +Y    |/                       /
     *  <-----+                       +-----------> +X
     *                                |
     *                                |
     *                                V +Y
     *
     */
    const double x = -P.y / P.x;
    const double y = -P.z / P.x;

    pixel_x = params.cx() + params.fx() * x;
    pixel_y = params.cy() + params.fy() * y;
}

mrpt::math::CMatrixDouble QualityEvaluator_RangeImageSimilarity::projectPoints(
    const mrpt::maps::CPointsMap&              pts,
    const std::optional<mrpt::poses::CPose3D>& relativePose) const
{
    const auto& rc = rangeCamera;

    mrpt::math::CMatrixDouble I(rc.nrows, rc.ncols);
    I.setZero();  // range=0 means "invalid"

    const auto& xs = pts.getPointsBufferRef_x();
    const auto& ys = pts.getPointsBufferRef_y();
    const auto& zs = pts.getPointsBufferRef_z();

    const auto nPoints = xs.size();
    // size_t     nValidPoints = 0;
    for (size_t i = 0; i < nPoints; i++)
    {
        mrpt::math::TPoint3D p(xs[i], ys[i], zs[i]);
        if (relativePose) p = relativePose->composePoint(p);

        double px, py;
        projectPoint(p, rc, px, py);
        int pixx = static_cast<int>(px);
        int pixy = static_cast<int>(py);

        // Out of range
        if (pixx < 0 || pixy < 0 || pixx >= int(rc.ncols) || pixy >= int(rc.nrows)) continue;

        const double newRange    = p.norm();
        double&      storedRange = I(pixy, pixx);

        if (storedRange == 0 || newRange < storedRange) storedRange = newRange;

        // nValidPoints++;
    }

    // std::cout << "Points: " << nValidPoints << "/" << nPoints << "\n";

    return I;
}

static double phi(double x) { return std::erf(x / std::sqrt(2)); }
static double errorForMismatch(const double x) { return 1.0 - phi(x); }
static double errorForMismatch(const double DeltaRange, const double sigma)
{
    const double x = std::abs(DeltaRange / sigma);
    return errorForMismatch(x);
}

std::vector<double> QualityEvaluator_RangeImageSimilarity::scores(
    const mrpt::math::CMatrixDouble& m1, const mrpt::math::CMatrixDouble& m2) const
{
    ASSERT_EQUAL_(m1.rows(), m2.rows());
    ASSERT_EQUAL_(m1.cols(), m2.cols());

    std::vector<double> scores;
    scores.reserve(m1.rows() * m1.cols() / 4);

    const size_t N = m1.rows() * m1.cols();
    for (size_t i = 0; i < N; i++)
    {
        const double r1 = m1.data()[i];
        const double r2 = m2.data()[i];

        if (r1 == 0 && r2 == 0) continue;

        double val;
        if (r1 == 0 || r2 == 0)
        {
            // Maximum error:
            val = errorForMismatch(penalty_not_visible);
        }
        else
        {
            // Maximum error:
            val = errorForMismatch(r1 - r2, sigma);
        }
        // printf("r1=%f r2=%f  -> val=%f\n", r1, r2, val);
        scores.push_back(val);
    }

    return scores;
}
