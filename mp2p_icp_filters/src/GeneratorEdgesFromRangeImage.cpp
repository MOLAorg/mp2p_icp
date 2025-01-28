/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GeneratorEdgesFromRangeImage.cpp
 * @brief  Generator of edge points from organized point clouds
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2023
 */

#include <mp2p_icp_filters/GeneratorEdgesFromRangeImage.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/utils.h>  // absDiff()
#include <mrpt/obs/CObservation3DRangeScan.h>
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
    MCP_LOAD_REQ(c, planes_target_layer);
    MCP_LOAD_REQ(c, score_threshold);
}

void GeneratorEdgesFromRangeImage::initialize(const mrpt::containers::yaml& c)
{
    // parent base method:
    Generator::initialize(c);

    paramsEdges_.load_from_yaml(c);
}

bool GeneratorEdgesFromRangeImage::filterRotatingScan(  //
    const mrpt::obs::CObservationRotatingScan& pc, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
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

    constexpr unsigned int BLOCK_BITS = 3;
    constexpr unsigned int W          = 1 << BLOCK_BITS;

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
                if (robotPose)
                    outPc->insertPoint(
                        robotPose->composePoint(pc.organizedPoints(r, i)));
                else
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

bool GeneratorEdgesFromRangeImage::filterScan3D(
    const mrpt::obs::CObservation3DRangeScan& rgbd, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    constexpr int FIXED_POINT_BITS = 8;

    // Optional output layer for deleted points:
    mrpt::maps::CPointsMap::Ptr outEdges = GetOrCreatePointLayer(
        out, params_.target_layer, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        "mrpt::maps::CSimplePointsMap");

    mrpt::maps::CPointsMap::Ptr outPlanes = GetOrCreatePointLayer(
        out, paramsEdges_.planes_target_layer, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        "mrpt::maps::CSimplePointsMap");

    if (outEdges) out.layers[params_.target_layer] = outEdges;
    if (outPlanes) out.layers[paramsEdges_.planes_target_layer] = outEdges;
    ASSERT_(outEdges || outPlanes);

    ASSERT_(rgbd.hasRangeImage);

    if (rgbd.rangeImage_isExternallyStored()) rgbd.load();

    // range is: CMatrix_u16. Zeros are invalid pixels.
    const auto nRows = rgbd.rangeImage.rows();
    const auto nCols = rgbd.rangeImage.cols();

    // Decimate range image, removing zeros:
    constexpr unsigned int BLOCK_BITS = 3;
    constexpr unsigned int BLOCKS     = 1 << BLOCK_BITS;

    const mrpt::math::CMatrix_u16& ri         = rgbd.rangeImage;
    const auto                     nRowsDecim = nRows >> BLOCK_BITS;
    const auto                     nColsDecim = nCols >> BLOCK_BITS;

    mrpt::math::CMatrix_u16 R(nRowsDecim, nColsDecim);
    R.fill(0);
    for (int rd = 0; rd < nRowsDecim; rd++)
    {
        for (int cd = 0; cd < nColsDecim; cd++)
        {
            size_t   count = 0;
            uint32_t sum   = 0;
            for (unsigned int i = 0; i < BLOCKS; i++)
            {
                for (unsigned int j = 0; j < BLOCKS; j++)
                {
                    const auto val =
                        ri((rd << BLOCK_BITS) + i, (cd << BLOCK_BITS) + j);
                    if (!val) continue;
                    count++;
                    sum += val;
                }
            }
            if (count) R(rd, cd) = sum / count;
        }
    }

    std::vector<int64_t> rowRangeDiff, rowRangeDiff2;

    const size_t WH  = nRows * nCols;
    const auto&  lut = rgbd.get_unproj_lut();

    // Select between coordinates wrt the robot/vehicle, or local wrt sensor:
    const auto& Kxs = lut.Kxs_rot;
    const auto& Kys = lut.Kys_rot;
    const auto& Kzs = lut.Kzs_rot;

    ASSERT_EQUAL_(WH, size_t(Kxs.size()));
    ASSERT_EQUAL_(WH, size_t(Kys.size()));
    ASSERT_EQUAL_(WH, size_t(Kzs.size()));
    const float* kxs = &Kxs[0];
    const float* kys = &Kys[0];
    const float* kzs = &Kzs[0];

    const auto sensorTranslation = rgbd.sensorPose.translation();

    const int MIN_SPACE_BETWEEN_PLANE_POINTS = nColsDecim / 16;

    auto lambdaUnprojectPoint = [&](const int rd, const int cd)
    {
        // unproject range -> 3D:
        const float D = R(rd, cd) * rgbd.rangeUnits;
        // LUT projection coefs:
        const int r = rd * BLOCKS + BLOCKS / 2;
        const int c = cd * BLOCKS + BLOCKS / 2;

        const auto kx = kxs[r * nCols + c], ky = kys[r * nCols + c],
                   kz = kzs[r * nCols + c];

        // unproject range -> 3D (includes sensorPose rotation):
        auto pt = mrpt::math::TPoint3Df(kx * D, ky * D /*y*/, kz * D /*z*/);
        pt += sensorTranslation;

        if (robotPose) pt = robotPose->composePoint(pt);

        return pt;
    };

    // analize each row:
    for (int rd = 0; rd < nRowsDecim; rd++)
    {
        rowRangeDiff.assign(nColsDecim, 0);
        rowRangeDiff2.assign(nColsDecim, 0);

        // compute range diff:
        for (int cd = 1; cd < nColsDecim; cd++)
        {
            if (!R(rd, cd) || !R(rd, cd - 1)) continue;  // ignore invalid pts

            rowRangeDiff[cd] = (static_cast<int64_t>(R(rd, cd)) -
                                static_cast<int64_t>(R(rd, cd - 1)))
                               << FIXED_POINT_BITS;
        }
        for (int cd = 1; cd < nColsDecim; cd++)
            rowRangeDiff2[cd] = rowRangeDiff[cd] - rowRangeDiff[cd - 1];

        // filtered range diff (in fixed-point arithmetic)
        const auto [rdMean, rdVar] =
            calcStats(rowRangeDiff2.data(), rowRangeDiff2.size());

        std::optional<int> currentPlaneStart;

        for (int cd = 1; cd < nColsDecim; cd++)
        {
            if (!R(rd, cd))
            {
                // invalid range here, stop.
                currentPlaneStart.reset();
                continue;
            }

            int64_t scoreSqr =
                rdVar != 0 ? mrpt::square(rowRangeDiff2[cd]) / rdVar : 0;

            if (scoreSqr > paramsEdges_.score_threshold)
            {
                // it's an edge:
                currentPlaneStart.reset();

                outEdges->insertPoint(lambdaUnprojectPoint(rd, cd));
            }
            else
            {
                // looks like a plane:
                if (!currentPlaneStart) currentPlaneStart = cd;

                if (cd - *currentPlaneStart > MIN_SPACE_BETWEEN_PLANE_POINTS)
                {
                    // create a plane point:
                    outPlanes->insertPoint(lambdaUnprojectPoint(rd, cd));
                    currentPlaneStart.reset();
                }
            }
        }
    }  // end for each row

    return true;
}
