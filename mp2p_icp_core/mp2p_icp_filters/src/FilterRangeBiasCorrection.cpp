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
 * @file   FilterRangeBiasCorrection.cpp
 * @brief  Removes a range- and incidence-dependent LiDAR surface bias.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 24, 2026
 */

#include <mp2p_icp/estimate_points_eigen.h>
#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mp2p_icp_filters/FilterRangeBiasCorrection.h>
#include <mrpt/containers/yaml.h>

#include <cmath>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterRangeBiasCorrection, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{
void load_coefficients(
    const mrpt::containers::yaml& c, const char* name, std::array<double, 3>& out)
{
    if (!c.has(name))
    {
        return;
    }
    ASSERTMSG_(
        c[name].isSequence() && c[name].asSequence().size() == 3,
        mrpt::format("'%s' must be a sequence [a, b, c]", name));
    const auto seq = c[name].asSequence();
    for (size_t i = 0; i < 3; i++)
    {
        out[i] = seq.at(i).as<double>();
    }
}
}  // namespace

void FilterRangeBiasCorrection::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterRangeBiasCorrection& parent)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    MCP_LOAD_OPT(c, ground_min_nz);
    MCP_LOAD_OPT(c, wall_max_nz);
    MCP_LOAD_OPT(c, k_neighbors);
    MCP_LOAD_OPT(c, max_neighbor_distance);
    MCP_LOAD_OPT(c, min_neighbors);
    MCP_LOAD_OPT(c, max_planarity_ratio);
    MCP_LOAD_OPT(c, min_line_ratio);
    MCP_LOAD_OPT(c, max_correction);
    MCP_LOAD_OPT(c, parallelization_grain_size);

    load_coefficients(c, "ground", ground);
    load_coefficients(c, "slanted", slanted);
    load_coefficients(c, "wall", wall);

    ASSERT_GE_(k_neighbors, 3U);
    ASSERT_GE_(min_neighbors, 3U);
    ASSERT_LE_(min_neighbors, k_neighbors);
    ASSERT_GT_(parallelization_grain_size, 0UL);

    if (c.has("sensor_origin"))
    {
        ASSERT_(c["sensor_origin"].isSequence() && c["sensor_origin"].asSequence().size() == 3);
        auto cc = c["sensor_origin"].asSequence();
        for (int i = 0; i < 3; i++)
        {
            parent.parseAndDeclareParameter(cc.at(i).as<std::string>(), sensor_origin[i]);
        }
    }
}

FilterRangeBiasCorrection::FilterRangeBiasCorrection()
{
    mrpt::system::COutputLogger::setLoggerName("FilterRangeBiasCorrection");
}

void FilterRangeBiasCorrection::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterRangeBiasCorrection::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    auto pcPtr = inOut.point_layer(params.pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.", params.pointcloud_layer.c_str()));

    auto&        pc = *pcPtr;
    const size_t N  = pc.size();
    if (N == 0)
    {
        return;
    }

    // Nothing to correct: skip the (costly) normal estimation.
    const auto isZero = [](const std::array<double, 3>& k)
    { return k[0] == 0 && k[1] == 0 && k[2] == 0; };
    if (isZero(params.ground) && isZero(params.slanted) && isZero(params.wall))
    {
        return;
    }

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    // All normals come from the uncorrected cloud: compute every shift first,
    // apply them afterwards.
    std::vector<mrpt::math::TPoint3Df> shifts(N, {0, 0, 0});

    pc.nn_prepare_for_3d_queries();

    const float maxDistSqr = static_cast<float>(mrpt::square(params.max_neighbor_distance));
    const auto& o          = params.sensor_origin;

    auto lambda_process = [&](size_t i)
    {
        const mrpt::math::TPoint3Df p(xs[i], ys[i], zs[i]);

        std::vector<size_t> indices;
        std::vector<float>  distsSqr;
        pc.kdTreeNClosestPoint3DIdx(p, params.k_neighbors, indices, distsSqr);

        size_t nUsable = 0;
        while (nUsable < indices.size() && distsSqr[nUsable] <= maxDistSqr)
        {
            nUsable++;
        }
        if (nUsable < params.min_neighbors)
        {
            return;
        }
        indices.resize(nUsable);

        const auto  eig = mp2p_icp::estimate_points_eigen(xs.data(), ys.data(), zs.data(), indices);
        const auto& ev  = eig.eigVals;
        if (!(ev[0] < params.max_planarity_ratio * ev[1]) ||
            !(ev[1] > params.min_line_ratio * ev[2]))
        {
            return;
        }

        mrpt::math::TVector3D ray(p.x - o.x, p.y - o.y, p.z - o.z);
        const double          range = ray.norm();
        if (range < 1e-3)
        {
            return;
        }
        ray *= 1.0 / range;

        // Normal oriented away from the sensor:
        mrpt::math::TVector3D n    = eig.eigVectors[0];
        double                cosI = n.x * ray.x + n.y * ray.y + n.z * ray.z;
        if (cosI < 0)
        {
            n    = -n;
            cosI = -cosI;
        }
        const double sin2I = 1.0 - cosI * cosI;

        const double absNz = std::abs(n.z);
        const auto&  k     = absNz > params.ground_min_nz ? params.ground
                             : absNz < params.wall_max_nz ? params.wall
                                                          : params.slanted;

        const double d = mrpt::saturate_val(
            k[0] + k[1] * range + k[2] * sin2I, -params.max_correction, params.max_correction);

        shifts[i] = mrpt::math::TPoint3Df(
            static_cast<float>(-d * n.x), static_cast<float>(-d * n.y),
            static_cast<float>(-d * n.z));
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

    auto& mxs = const_cast<mrpt::aligned_std_vector<float>&>(xs);
    auto& mys = const_cast<mrpt::aligned_std_vector<float>&>(ys);
    auto& mzs = const_cast<mrpt::aligned_std_vector<float>&>(zs);

    size_t nCorrected = 0;
    for (size_t i = 0; i < N; ++i)
    {
        if (shifts[i].x == 0 && shifts[i].y == 0 && shifts[i].z == 0)
        {
            continue;
        }
        mxs[i] += shifts[i].x;
        mys[i] += shifts[i].y;
        mzs[i] += shifts[i].z;
        nCorrected++;
    }
    pc.mark_as_modified();

    MRPT_LOG_DEBUG_STREAM(
        "Corrected " << nCorrected << " of " << N << " points in layer '" << params.pointcloud_layer
                     << "'");

    const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(pc);
    ASSERT_(sanityPassed);

    MRPT_END
}
