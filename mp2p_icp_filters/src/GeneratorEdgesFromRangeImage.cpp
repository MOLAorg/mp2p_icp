/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GeneratorEdgesFromRangeImage.cpp
 * @brief  Generator of edge points from organized point clouds
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2023
 */

#include <mp2p_icp_filters/GeneratorEdgesFromRangeImage.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/utils.h>  // absDiff()
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/version.h>

#include <utility>  // std::pair

IMPLEMENTS_MRPT_OBJECT(
    GeneratorEdgesFromRangeImage, Generator, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{
// Computes the mean and variance of a number of integer samples,
// using fixed-point integers for efficiency and avoiding float numbers.
auto calcStats(const int64_t* data, const size_t N)
    -> std::pair<int64_t /*mean*/, int64_t /*stdDev*/>
{
    ASSERT_(N > 1);

    int64_t sumMean = 0;
    for (size_t i = 0; i < N; i++) sumMean += data[i];

    const int64_t mean = sumMean / (N - 1);

    int64_t sumVariance = 0;
    for (size_t i = 0; i < N; i++)
        sumVariance +=
            mrpt::square(mrpt::math::absDiff<int64_t>(data[i], mean));

    const int64_t variance = sumVariance / (N - 1);

    return {mean, variance};
}

}  // namespace

void GeneratorEdgesFromRangeImage::ParametersEdges::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, row_window_length);
    MCP_LOAD_REQ(c, score_threshold);
}

void GeneratorEdgesFromRangeImage::initialize(const mrpt::containers::yaml& c)
{
    // parent base method:
    Generator::initialize(c);

    paramsEdges_.load_from_yaml(c);
}

bool GeneratorEdgesFromRangeImage::filterRotatingScan(  //
    const mrpt::obs::CObservationRotatingScan& pc,
    mp2p_icp::metric_map_t&                    out) const
{
#if MRPT_VERSION >= 0x020b04
    constexpr int FIXED_POINT_BITS = 8;

    auto outPc = mrpt::maps::CSimplePointsMap::Create();

    ASSERT_(!pc.organizedPoints.empty());

    const auto nRows = pc.rowCount;
    const auto nCols = pc.columnCount;

    ASSERT_EQUAL_(nRows, pc.organizedPoints.rows());
    ASSERT_EQUAL_(nCols, pc.organizedPoints.cols());

    ASSERT_EQUAL_(nRows, pc.rangeImage.rows());
    ASSERT_EQUAL_(nCols, pc.rangeImage.cols());

    const unsigned int W = paramsEdges_.row_window_length;

    std::vector<int64_t> rowRangeDiff;

    // for each row:
    for (size_t r = 0; r < nRows; r++)
    {
        rowRangeDiff.assign(nCols, 0);

        // compute range diff:
        for (size_t i = 1; i < nCols; i++)
        {
            rowRangeDiff[i] = (static_cast<int64_t>(pc.rangeImage(r, i)) -
                               static_cast<int64_t>(pc.rangeImage(r, i - 1)))
                              << FIXED_POINT_BITS;
        }

        for (size_t i = 1 + W; i < nCols - W; i++)
        {
            // filtered range diff (in fixed-point arithmetic)
            const auto [rdFiltered, rdVar] =
                calcStats(&rowRangeDiff[i - W], 1 + 2 * W);

            if (rdVar == 0) continue;  // by no way this is an edge! avoid x/0

            // significance of each point (in fixed-point arithmetic)
            const int64_t riFixPt = pc.rangeImage(r, i) << FIXED_POINT_BITS;
            int64_t       scoreSqrFixPt =
                mrpt::square(mrpt::math::absDiff(riFixPt, rdFiltered)) / rdVar;

            const int32_t scoreSqr = scoreSqrFixPt >> (2 * FIXED_POINT_BITS);

            if (scoreSqr > paramsEdges_.score_threshold)
            {
                // this point passes:
                outPc->insertPoint(pc.organizedPoints(r, i));
            }
        }

    }  // end for each row

    out.layers[params_.target_layer] = outPc;
    return true;  // Yes, it's implemented
#else
    THROW_EXCEPTION("This class requires MRPT >=v2.11.4");
#endif
}
