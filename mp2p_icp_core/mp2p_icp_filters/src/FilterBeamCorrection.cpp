/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterBeamCorrection.cpp
 * @brief  Applies per-beam (per-ring) intrinsic corrections to LiDAR points.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mp2p_icp/yaml_variables.h>
#include <mp2p_icp_filters/FilterBeamCorrection.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterBeamCorrection, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{
double optionalNumber(const mrpt::containers::yaml& row, const char* key)
{
    return row.has(key) ? row[key].as<double>() : 0.0;
}

void loadRows(const mrpt::containers::yaml& seq, std::vector<FilterBeamCorrection::Beam>& beams)
{
    ASSERTMSG_(seq.isSequence(), "'beams' must be a sequence of maps, one per ring");
    for (const auto& entry : seq.asSequence())
    {
        const mrpt::containers::yaml row(entry);
        ASSERTMSG_(row.isMap() && row.has("ring"), "every 'beams' entry needs a 'ring' key");
        const int ring = row["ring"].as<int>();
        ASSERTMSG_(ring >= 0, mrpt::format("invalid ring index %d", ring));
        if (beams.size() <= static_cast<size_t>(ring))
        {
            beams.resize(static_cast<size_t>(ring) + 1);
        }
        auto& b         = beams[static_cast<size_t>(ring)];
        b.d_elevation   = mrpt::DEG2RAD(optionalNumber(row, "d_elevation_deg"));
        b.d_azimuth     = mrpt::DEG2RAD(optionalNumber(row, "d_azimuth_deg"));
        b.d_range       = optionalNumber(row, "d_range_m");
        b.d_range_scale = optionalNumber(row, "d_range_scale");
        b.d_vertical    = optionalNumber(row, "d_vertical_m");
        b.d_horizontal  = optionalNumber(row, "d_horizontal_m");
        b.defined       = true;
    }
}

void loadAzimuthOffsets(const mrpt::containers::yaml& seq, std::vector<double>& out)
{
    ASSERTMSG_(seq.isSequence(), "beam azimuth offsets must be a sequence, one per ring");
    out.clear();
    for (const auto& v : seq.asSequence())
    {
        out.push_back(mrpt::DEG2RAD(mrpt::containers::yaml(v).as<double>()));
    }
}
}  // namespace

void FilterBeamCorrection::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterBeamCorrection& parent)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    MCP_LOAD_OPT(c, ring_field);
    MCP_LOAD_OPT(c, min_range);
    MCP_LOAD_OPT(c, parallelization_grain_size);
    ASSERT_GT_(parallelization_grain_size, 0UL);

    if (c.has("sensor_pose"))
    {
        ASSERTMSG_(
            c["sensor_pose"].isSequence() && c["sensor_pose"].asSequence().size() == 6,
            "'sensor_pose' must be [x, y, z, yaw, pitch, roll]");
        auto cc = c["sensor_pose"].asSequence();
        for (int i = 0; i < 6; i++)
        {
            parent.parseAndDeclareParameter(cc.at(i).as<std::string>(), sensor_pose[i]);
        }
    }

    ASSERTMSG_(
        c.has("beams") != c.has("beams_file"),
        "exactly one of 'beams' (inline table) or 'beams_file' must be given");

    beams.clear();
    beam_azimuth_offsets.clear();
    bool originFromFile = false;
    if (c.has("beams"))
    {
        loadRows(c["beams"], beams);
    }
    else
    {
        const auto file = c["beams_file"].as<std::string>();
        const auto doc  = mp2p_icp::load_yaml_file(file);
        if (doc.has("beams"))
        {
            loadRows(doc["beams"], beams);
        }
        else
        {
            ASSERTMSG_(
                doc.has("sensors") && doc["sensors"].isMap(),
                mrpt::format(
                    "'%s' has neither a 'beams' sequence nor a 'sensors' map", file.c_str()));
            std::string sensor = c.getOrDefault<std::string>("sensor", "");
            if (sensor.empty())
            {
                // The only sensor with a per-beam table:
                for (const auto& [k, v] : doc["sensors"].asMap())
                {
                    const mrpt::containers::yaml s(v);
                    if (s.has("intrinsics") && s["intrinsics"].isMap() &&
                        s["intrinsics"].has("beams"))
                    {
                        ASSERTMSG_(
                            sensor.empty(),
                            mrpt::format(
                                "'%s' has per-beam tables for several sensors: set 'sensor'",
                                file.c_str()));
                        sensor = k.as<std::string>();
                    }
                }
                ASSERTMSG_(
                    !sensor.empty(), mrpt::format("'%s' has no per-beam table", file.c_str()));
            }
            ASSERTMSG_(
                doc["sensors"].has(sensor),
                mrpt::format("'%s' has no sensor '%s'", file.c_str(), sensor.c_str()));
            const auto intr = doc["sensors"][sensor]["intrinsics"];
            ASSERTMSG_(
                intr.isMap() && intr.has("beams"),
                mrpt::format(
                    "sensor '%s' in '%s' has no 'intrinsics.beams'", sensor.c_str(), file.c_str()));
            loadRows(intr["beams"], beams);
            if (intr.has("beam_origin_m") && !c.has("beam_origin_m"))
            {
                beam_origin_m  = intr["beam_origin_m"].as<double>();
                originFromFile = true;
            }
            if (intr.has("beam_azimuth_offset_deg"))
            {
                loadAzimuthOffsets(intr["beam_azimuth_offset_deg"], beam_azimuth_offsets);
            }
        }
    }
    if (!originFromFile)
    {
        MCP_LOAD_OPT(c, beam_origin_m);
    }
    if (c.has("beam_azimuth_offsets_deg"))
    {
        loadAzimuthOffsets(c["beam_azimuth_offsets_deg"], beam_azimuth_offsets);
    }
}

FilterBeamCorrection::FilterBeamCorrection()
{
    mrpt::system::COutputLogger::setLoggerName("FilterBeamCorrection");
}

void FilterBeamCorrection::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

mrpt::math::TPoint3D FilterBeamCorrection::correctPoint(
    const mrpt::math::TPoint3D& p, std::size_t ring) const
{
    if (ring >= params.beams.size() || !params.beams[ring].defined)
    {
        return p;
    }
    const Beam&  c = params.beams[ring];
    const double b = params.beam_origin_m;
    const double aOff =
        ring < params.beam_azimuth_offsets.size() ? params.beam_azimuth_offsets[ring] : 0.0;

    // Encoder angle: horizontally p = b u(theta) + range_h u(theta + aOff), a
    // fixed point that contracts by b / range_h.
    double az = std::atan2(p.y, p.x);
    if (b != 0)
    {
        for (int it = 0; it < 4; it++)
        {
            const double th = az - aOff;
            az              = std::atan2(p.y - b * std::sin(th), p.x - b * std::cos(th));
        }
    }
    const double th  = az - aOff;
    const double ox  = b * std::cos(th);
    const double oy  = b * std::sin(th);
    const double qx  = p.x - ox;
    const double qy  = p.y - oy;
    const double qz  = p.z;
    const double qh  = std::hypot(qx, qy);
    const double rho = std::hypot(qh, qz);
    if (rho < params.min_range)
    {
        return p;
    }
    const double e    = std::atan2(qz, qh) + c.d_elevation;
    const double a    = std::atan2(qy, qx) + c.d_azimuth;
    const double rho2 = (1.0 + c.d_range_scale) * rho + c.d_range;
    const double ce   = std::cos(e);
    const double ca   = std::cos(a);
    const double sa   = std::sin(a);
    return {
        ox - c.d_horizontal * sa + rho2 * ce * ca, oy + c.d_horizontal * ca + rho2 * ce * sa,
        c.d_vertical + rho2 * std::sin(e)};
}

void FilterBeamCorrection::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    auto pcPtr = inOut.point_layer(params.pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.", params.pointcloud_layer.c_str()));

    auto&        pc = *pcPtr;
    const size_t N  = pc.size();
    if (N == 0 || params.beams.empty())
    {
        return;
    }

    const auto* ringsU16 = pc.getPointsBufferRef_uint16_field(params.ring_field);
    const auto* ringsF   = pc.getPointsBufferRef_float_field(params.ring_field);
    const bool  useU16   = ringsU16 && !ringsU16->empty();
    ASSERTMSG_(
        useU16 || (ringsF && !ringsF->empty()),
        mrpt::format(
            "Layer '%s' has no '%s' point field: per-beam corrections need the ring of each point",
            params.pointcloud_layer.c_str(), params.ring_field.c_str()));
    ASSERT_EQUAL_(useU16 ? ringsU16->size() : ringsF->size(), N);

    auto& xs = const_cast<mrpt::aligned_std_vector<float>&>(pc.getPointsBufferRef_x());
    auto& ys = const_cast<mrpt::aligned_std_vector<float>&>(pc.getPointsBufferRef_y());
    auto& zs = const_cast<mrpt::aligned_std_vector<float>&>(pc.getPointsBufferRef_z());

    const mrpt::poses::CPose3D T_cloud_sensor(params.sensor_pose);
    const mrpt::poses::CPose3D T_sensor_cloud = -T_cloud_sensor;

    // Each point depends only on itself: in place and in parallel.
    auto lambda_process = [&](size_t i)
    {
        const std::size_t ring =
            useU16 ? (*ringsU16)[i]
                   : static_cast<std::size_t>(std::max(0.0f, std::round((*ringsF)[i])));
        if (ring >= params.beams.size() || !params.beams[ring].defined)
        {
            return;
        }
        const mrpt::math::TPoint3D pS =
            T_sensor_cloud.composePoint(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]));
        const mrpt::math::TPoint3D q = T_cloud_sensor.composePoint(correctPoint(pS, ring));
        xs[i]                        = static_cast<float>(q.x);
        ys[i]                        = static_cast<float>(q.y);
        zs[i]                        = static_cast<float>(q.z);
    };

#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, N, params.parallelization_grain_size),
        [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                lambda_process(i);
            }
        });
#else
    for (size_t i = 0; i < N; ++i)
    {
        lambda_process(i);
    }
#endif
    pc.mark_as_modified();

    const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(pc);
    ASSERT_(sanityPassed);

    MRPT_END
}
