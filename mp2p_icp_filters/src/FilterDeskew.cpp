/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDeskew.cpp
 * @brief  Deskew (motion compensate) a pointcloud from a moving LIDAR
 * @author Jose Luis Blanco Claraco
 * @date   Dec 13, 2023
 */

#include <mp2p_icp_filters/FilterDeskew.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/version.h>

#if MRPT_VERSION >= 0x020b04
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

#if defined(MP2P_HAS_TBB)
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(
    FilterDeskew, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterDeskew::FilterDeskew()
{
    mrpt::system::COutputLogger::setLoggerName("FilterDeskew");
}

void FilterDeskew::initialize(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);

    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_OPT(c, silently_ignore_no_timestamps);
    MCP_LOAD_OPT(c, output_layer_class);
    MCP_LOAD_OPT(c, skip_deskew);

    ASSERT_(c.has("twist") && c["twist"].isSequence());
    ASSERT_EQUAL_(c["twist"].asSequence().size(), 6UL);

    const auto yamlTwist = c["twist"].asSequence();

    for (int i = 0; i < 6; i++)
        Parameterizable::parseAndDeclareParameter(
            yamlTwist.at(i).as<std::string>(), twist[i]);
}

void FilterDeskew::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START
#if MRPT_VERSION >= 0x020b04

    checkAllParametersAreRealized();

    // Out:
    ASSERT_(!output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, output_pointcloud_layer, false /*dont allow empty names*/,
        output_layer_class);

    // In:
    ASSERT_(!input_pointcloud_layer.empty());

    const mrpt::maps::CPointsMap* inPc = nullptr;
    if (auto itLy = inOut.layers.find(input_pointcloud_layer);
        itLy != inOut.layers.end())
    {
        inPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!inPc)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.",
                input_pointcloud_layer.c_str());

        outPc->reserve(outPc->size() + inPc->size());
    }
    else
    {
        // Input layer doesn't exist:
        THROW_EXCEPTION_FMT(
            "Input layer '%s' not found on input map.",
            input_pointcloud_layer.c_str());
    }

    // If the input is empty, just move on:
    if (inPc->empty())
    {
        MRPT_LOG_DEBUG_STREAM(
            "Silently ignoring empty input layer: '" << input_pointcloud_layer
                                                     << "'");
        return;
    }

    // mandatory fields:
    const auto&  xs = inPc->getPointsBufferRef_x();
    const auto&  ys = inPc->getPointsBufferRef_y();
    const auto&  zs = inPc->getPointsBufferRef_z();
    const size_t n  = xs.size();

    // optional fields:
    const auto* Is = inPc->getPointsBufferRef_intensity();
    const auto* Ts = inPc->getPointsBufferRef_timestamp();
    const auto* Rs = inPc->getPointsBufferRef_ring();

    auto* out_Is = outPc->getPointsBufferRef_intensity();
    auto* out_Rs = outPc->getPointsBufferRef_ring();
    auto* out_Ts = outPc->getPointsBufferRef_timestamp();

    // Do we have input timestamps per point?
    if (!Ts || Ts->empty() || skip_deskew)
    {
        // not possible to do de-skew:
        if (silently_ignore_no_timestamps || skip_deskew)
        {
            // just copy all points, including all optional attributes:
            for (size_t i = 0; i < n; i++)  //
                outPc->insertPointFrom(*inPc, i);

            MRPT_LOG_DEBUG_STREAM(
                "Skipping de-skewing in input cloud '"
                << input_pointcloud_layer
                << "' with contents: " << inPc->asString());
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Input layer '%s' does not contain per-point timestamps, "
                "cannot do scan deskew. Set "
                "'silently_ignore_no_timestamps=true' to skip de-skew."
                "Input map contents: '%s'",
                input_pointcloud_layer.c_str(), inPc->asString().c_str());
        }
    }
    else
    {
        ASSERT_EQUAL_(Ts->size(), n);

        // Yes, we have timestamps, apply de-skew:
        const size_t n0 = outPc->size();
        outPc->resize(n0 + n);

        const auto v = mrpt::math::TVector3D(twist.vx, twist.vy, twist.vz);
        const auto w = mrpt::math::TVector3D(twist.wx, twist.wy, twist.wz);

#if defined(MP2P_HAS_TBB)
        tbb::parallel_for(
            static_cast<size_t>(0), n,
            [&](size_t i)
#else
        for (size_t i = 0; i < n; i++)
#endif
            {
                const auto pt = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);
                if (!(pt.x == 0 && pt.y == 0 && pt.z == 0))
                {
                    // Forward integrate twist:
                    const mrpt::math::TVector3D v_dt = v * (*Ts)[i];
                    const mrpt::math::TVector3D w_dt = w * (*Ts)[i];

                    const auto p =
                        mrpt::poses::CPose3D::FromRotationAndTranslation(
                            // Rotation: From Lie group SO(3) exponential:
                            mrpt::poses::Lie::SO<3>::exp(
                                mrpt::math::CVectorFixedDouble<3>(w_dt)),
                            // Translation: simple constant velocity model:
                            v_dt);

                    const auto corrPt = p.composePoint(pt);

                    outPc->setPointFast(n0 + i, corrPt.x, corrPt.y, corrPt.z);
                    if (Is && out_Is) (*out_Is)[n0 + i] = (*Is)[i];
                    if (Rs && out_Rs) (*out_Rs)[n0 + i] = (*Rs)[i];
                    if (Ts && out_Ts) (*out_Ts)[n0 + i] = (*Ts)[i];
                }
            }
#if defined(MP2P_HAS_TBB)
        );
#endif
    }

    outPc->mark_as_modified();
#else
    THROW_EXCEPTION("This class requires MRPT >=v2.11.4");
#endif
    MRPT_END
}
