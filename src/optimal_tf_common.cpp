/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   OptimalTF_common.cpp
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_common.h>
#include <mrpt/serialization/CArchive.h>
#include <iterator>  // std::make_move_iterator

IMPLEMENTS_MRPT_OBJECT(
    WeightParameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t WeightParameters::serializeGetVersion() const { return 0; }
void    WeightParameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << use_scale_outlier_detector << scale_outlier_threshold
        << attitude_weights.pt2pt << attitude_weights.l2l
        << attitude_weights.pl2pl << use_robust_kernel
        << current_estimate_for_robust << robust_kernel_param
        << robust_kernel_scale;
}
void WeightParameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> use_scale_outlier_detector >> scale_outlier_threshold >>
                attitude_weights.pt2pt >> attitude_weights.l2l >>
                attitude_weights.pl2pl >> use_robust_kernel >>
                current_estimate_for_robust >> robust_kernel_param >>
                robust_kernel_scale;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D>
    mp2p_icp::eval_centroids_robust(
        const Pairings& in, const OutlierIndices& outliers)
{
    using mrpt::math::TPoint3D;

    const auto nPoints = in.paired_points.size();

    // We need more points than outliers (!)
    ASSERT_ABOVE_(nPoints, outliers.point2point.size());

    // Normalized weights for centroids.
    // Discount outliers.
    const double wcPoints = 1.0 / (nPoints - outliers.point2point.size());

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);
    {
        std::size_t cnt             = 0;
        auto        it_next_outlier = outliers.point2point.begin();
        for (std::size_t i = 0; i < in.paired_points.size(); i++)
        {
            // Skip outlier?
            if (it_next_outlier != outliers.point2point.end() &&
                i == *it_next_outlier)
            {
                ++it_next_outlier;
                continue;
            }
            const auto& pair = in.paired_points[i];

            ct_this += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
            cnt++;
        }
        // Sanity check:
        ASSERT_EQUAL_(cnt, nPoints - outliers.point2point.size());

        ct_other *= wcPoints;
        ct_this *= wcPoints;
    }

    return {ct_other, ct_this};
}

void Pairings::push_back(const Pairings& o)
{
    paired_points.insert(
        paired_points.end(), o.paired_points.begin(), o.paired_points.end());
    paired_lines.insert(
        paired_lines.end(), o.paired_lines.begin(), o.paired_lines.end());
    paired_planes.insert(
        paired_planes.end(), o.paired_planes.begin(), o.paired_planes.end());
    point_weights.insert(
        point_weights.end(), o.point_weights.begin(), o.point_weights.end());
}

void Pairings::push_back(Pairings&& o)
{
    paired_points.insert(
        paired_points.end(), std::make_move_iterator(o.paired_points.begin()),
        std::make_move_iterator(o.paired_points.end()));
    paired_lines.insert(
        paired_lines.end(), std::make_move_iterator(o.paired_lines.begin()),
        std::make_move_iterator(o.paired_lines.end()));
    paired_planes.insert(
        paired_planes.end(), std::make_move_iterator(o.paired_planes.begin()),
        std::make_move_iterator(o.paired_planes.end()));
    point_weights.insert(
        point_weights.end(), std::make_move_iterator(o.point_weights.begin()),
        std::make_move_iterator(o.point_weights.end()));
}

size_t Pairings::size() const
{
    return paired_points.size() + paired_lines.size() + paired_planes.size();
}

void WeightParameters::loadFrom(const mrpt::containers::Parameters& p)
{
    MCP_LOAD_REQ(p, use_scale_outlier_detector);
    MCP_LOAD_OPT(p, scale_outlier_threshold);

    MCP_LOAD_REQ(p, use_robust_kernel);
    MCP_LOAD_OPT_DEG(p, robust_kernel_param);
    MCP_LOAD_OPT(p, robust_kernel_scale);

    if (p.has("attitude_weights"))
        attitude_weights.loadFrom(p["attitude_weights"]);
}
void WeightParameters::saveTo(mrpt::containers::Parameters& p) const
{
    MCP_SAVE(p, use_scale_outlier_detector);
    MCP_SAVE(p, scale_outlier_threshold);

    MCP_SAVE(p, use_robust_kernel);
    MCP_SAVE_DEG(p, robust_kernel_param);
    MCP_SAVE(p, robust_kernel_scale);

    auto a = mrpt::containers::Parameters::Map();
    attitude_weights.saveTo(a);
    p["attitude_weights"] = a;
}

void WeightParameters::AttitudeWeights::loadFrom(
    const mrpt::containers::Parameters& p)
{
    MCP_LOAD_REQ(p, pt2pt);
    MCP_LOAD_REQ(p, l2l);
    MCP_LOAD_REQ(p, pl2pl);
}

void WeightParameters::AttitudeWeights::saveTo(
    mrpt::containers::Parameters& p) const
{
    MCP_SAVE(p, pt2pt);
    MCP_SAVE(p, l2l);
    MCP_SAVE(p, pl2pl);
}
