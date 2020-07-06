/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_RangeImageSimilarity.cpp
 * @brief  Matching quality evaluator from paper [Bogoslavskyi,IROS2017]
 * @author Jose Luis Blanco Claraco
 * @date   July 6, 2020
 */

#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mrpt/img/TPixelCoord.h>

IMPLEMENTS_MRPT_OBJECT(
    QualityEvaluator_RangeImageSimilarity, QualityEvaluator, mp2p_icp);

using namespace mp2p_icp;

void QualityEvaluator_RangeImageSimilarity::initialize(
    const mrpt::containers::Parameters& params)
{
    rangeCamera.ncols = params["ncols"].as<uint32_t>();
    rangeCamera.nrows = params["nrows"].as<uint32_t>();

    rangeCamera.cx(params["cx"].as<double>());
    rangeCamera.cy(params["cy"].as<double>());
    rangeCamera.fx(params["fx"].as<double>());
    rangeCamera.fy(params["fy"].as<double>());

    sigma = params["sigma"].as<double>();
}

double QualityEvaluator_RangeImageSimilarity::evaluate(
    const pointcloud_t& pcGlobal, const pointcloud_t& pcLocal,
    const mrpt::poses::CPose3D&      localPose,
    [[maybe_unused]] const Pairings& finalPairings) const
{
    // See Figure 3 of IROS2017 paper:
    // "Analyzing the Quality of Matched 3D Point Clouds of Objects"
    // Igor Bogoslavskyi, Cyrill Stachniss

    const auto& p1 = *pcGlobal.point_layers.at(pointcloud_t::PT_LAYER_RAW);
    const auto& p2 = *pcLocal.point_layers.at(pointcloud_t::PT_LAYER_RAW);

    const auto I11 = projectPoints(p1);
    const auto I12 = projectPoints(p1, localPose);
    const auto I22 = projectPoints(p2);
    const auto I21 = projectPoints(p2, -localPose);

#if 0
    I11.saveToTextFile("I11.txt");
    I22.saveToTextFile("I22.txt");
    I12.saveToTextFile("I12.txt");
    I21.saveToTextFile("I21.txt");
#endif

    auto s1 = scores(I11, I21);
    auto s2 = scores(I12, I22);

    const size_t nScores = s1.size() + s2.size();

    double sum = .0;
    for (double v : s1) sum += v;
    for (double v : s2) sum += v;

    return nScores > 0 ? sum / nScores : .0;
}

// Adapted from mrpt::vision::pinhole::projectPoint_with_distortion()
// 3-claused BSD
static void projectPoint(
    const mrpt::math::TPoint3D& P, const mrpt::img::TCamera& params,
    double& pixel_x, double& pixel_y)
{
    // Pinhole model:
    const double x = -P.y / P.x;
    const double y = P.z / P.x;

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

    const auto nPoints      = xs.size();
    size_t     nValidPoints = 0;
    for (size_t i = 0; i < xs.size(); i++)
    {
        mrpt::math::TPoint3D p(xs[i], ys[i], zs[i]);
        if (relativePose) p = relativePose->composePoint(p);

        double px, py;
        projectPoint(p, rc, px, py);
        int pixx = static_cast<int>(px);
        int pixy = static_cast<int>(py);

        // Out of range
        if (pixx < 0 || pixy < 0 || pixx >= int(rc.ncols) ||
            pixy >= int(rc.nrows))
            continue;

        const double newRange    = p.norm();
        double&      storedRange = I(pixy, pixx);

        if (storedRange == 0 || newRange < storedRange) storedRange = newRange;

        nValidPoints++;
    }

    // std::cout << "Points: " << nValidPoints << "/" << nPoints << "\n";

    return I;
}

static double phi(double x1, double x2)
{
    return (std::erf(x2 / std::sqrt(2)) - std::erf(x1 / std::sqrt(2))) / 2;
}
static double errorForMismatch(const double x) { return 1.0 - phi(-x, x); }
static double errorForMismatch(const double DeltaRange, const double sigma)
{
    const double x = std::abs(DeltaRange / sigma);
    return errorForMismatch(x);
}

std::vector<double> QualityEvaluator_RangeImageSimilarity::scores(
    const mrpt::math::CMatrixDouble& m1,
    const mrpt::math::CMatrixDouble& m2) const
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
            val = errorForMismatch(2.0);
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
