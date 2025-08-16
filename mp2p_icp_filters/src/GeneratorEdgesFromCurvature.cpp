/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GeneratorEdgesFromCurvature.cpp
 * @brief  Generator of edge points from organized point clouds
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2023
 */

#include <mp2p_icp_filters/GeneratorEdgesFromCurvature.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/utils.h>  // absDiff()
#include <mrpt/obs/CObservationRotatingScan.h>

#include <utility>  // std::pair

IMPLEMENTS_MRPT_OBJECT(GeneratorEdgesFromCurvature, Generator, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void GeneratorEdgesFromCurvature::ParametersEdges::load_from_yaml(
    const mrpt::containers::yaml& c, GeneratorEdgesFromCurvature& parent)
{
    DECLARE_PARAMETER_IN_REQ(c, max_cosine, parent);
    DECLARE_PARAMETER_IN_OPT(c, min_point_clearance, parent);
}

void GeneratorEdgesFromCurvature::initialize(const mrpt::containers::yaml& c)
{
    // parent base method:
    Generator::initialize(c);

    paramsEdges_.load_from_yaml(c, *this);
}

bool GeneratorEdgesFromCurvature::process(
    const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    MRPT_START
    using namespace mrpt::obs;

    ASSERTMSG_(initialized_, "initialize() must be called once before using process().");

    checkAllParametersAreRealized();

    const auto obsClassName = o.GetRuntimeClass()->className;

    // default: use point clouds:
    ASSERT_(params_.metric_map_definition_ini_file.empty());

    bool processed = false;

    // user-given filters: Done *AFTER* creating the map, if needed.
    if (!std::regex_match(obsClassName, process_class_names_regex_))
    {
        return false;
    }
    if (!std::regex_match(o.sensorLabel, process_sensor_labels_regex_))
    {
        return false;
    }

    if (auto oRS = dynamic_cast<const CObservationRotatingScan*>(&o); oRS)
    {
        processed = filterRotatingScan(*oRS, out, robotPose);
    }

    // done?
    if (processed)
    {
        return true;  // we are done.
    }

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc;
    if (auto itLy = out.layers.find(params_.target_layer); itLy != out.layers.end())
    {
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itLy->second);
        if (!outPc)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.", params_.target_layer.c_str());
        }
    }
    else
    {
        outPc                            = mrpt::maps::CSimplePointsMap::Create();
        out.layers[params_.target_layer] = outPc;
    }

    if (!outPc)
    {
        outPc = mrpt::maps::CSimplePointsMap::Create();
    }

    // Leave output point cloud empty, since it was not handled by the
    // rotating scan handler above.

    return false;

    MRPT_END
}

bool GeneratorEdgesFromCurvature::filterRotatingScan(  //
    const mrpt::obs::CObservationRotatingScan& pc, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    auto outPc = mrpt::maps::CSimplePointsMap::Create();

    ASSERT_(!pc.organizedPoints.empty());

    const auto nRows = pc.rowCount;
    const auto nCols = pc.columnCount;

    ASSERT_EQUAL_(nRows, pc.organizedPoints.rows());
    ASSERT_EQUAL_(nCols, pc.organizedPoints.cols());

    ASSERT_EQUAL_(nRows, pc.rangeImage.rows());
    ASSERT_EQUAL_(nCols, pc.rangeImage.cols());

    // for each row:
    for (size_t r = 0; r < nRows; r++)
    {
        // compute range diff:
        for (size_t i = 1; i + 1 < nCols; i++)
        {
            // we need at least 3 consecutive valid points:
            if (!pc.rangeImage(r, i - 1) || !pc.rangeImage(r, i) || !pc.rangeImage(r, i + 1))
            {
                continue;
            }

            const auto& pt_im1 = pc.organizedPoints(r, i - 1);
            const auto& pt_i   = pc.organizedPoints(r, i);
            const auto& pt_ip1 = pc.organizedPoints(r, i + 1);

            const auto v1  = (pt_i - pt_im1);
            const auto v2  = (pt_ip1 - pt_i);
            const auto v1n = v1.norm();
            const auto v2n = v2.norm();

            if (v1n < paramsEdges_.min_point_clearance || v2n < paramsEdges_.min_point_clearance)
            {
                continue;
            }

            const float score = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

            if (std::abs(score) < paramsEdges_.max_cosine * v1n * v2n)
            {
                // this point passes:
                if (robotPose)
                {
                    outPc->insertPoint(robotPose->composePoint(pc.organizedPoints(r, i)));
                }
                else
                {
                    outPc->insertPoint(pc.organizedPoints(r, i));
                }
                // TODO(jlbc) Output intensity?
            }
        }

    }  // end for each row

    out.layers[params_.target_layer] = outPc;
    return true;  // Yes, it's implemented
}
