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

/** The property the rotating rule exists for.
 *
 * Keeping the first point of every voxel samples in scan order, so the
 * survivors sit on the same side of their voxels and the decimated cloud is
 * displaced as a whole. Rotating which point is taken from one voxel to the
 * next leaves the individual points where they are while cancelling that
 * shared displacement.
 */

#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
constexpr double VOXEL = 1.0;
constexpr double BIAS  = 0.25;  // where in each voxel the earliest point sits

/** A ground-like sheet whose points enter each voxel in a fixed order: the
 *  one nearest a corner first. That is the pattern a sweeping sensor
 *  produces, and the one the rule is meant to defuse.
 */
mp2p_icp::metric_map_t makeOrderedSheet()
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();

    // Points are appended corner-first within every voxel, so "first point"
    // always means the same corner.
    for (int gx = 0; gx < 14; gx++)
    {
        for (int gy = 0; gy < 14; gy++)
        {
            const double x0 = gx * VOXEL;
            const double y0 = gy * VOXEL;
            for (int k = 0; k < 4; k++)
            {
                const double f = BIAS + k * 0.2;
                pts->insertPointFast(
                    static_cast<float>(x0 + f), static_cast<float>(y0 + f),
                    static_cast<float>(0.1 * k));
            }
        }
    }
    pts->mark_as_modified();

    mp2p_icp::metric_map_t mm;
    mm.layers["raw"] = pts;
    return mm;
}

/** Mean offset of the kept points from the center of the voxel they came from:
 *  the displacement the decimation gives the cloud as a whole.
 */
double meanVoxelOffset(const mrpt::maps::CPointsMap& out)
{
    const auto& xs = out.getPointsBufferRef_x();
    const auto& ys = out.getPointsBufferRef_y();

    double sx = 0;
    double sy = 0;
    for (size_t i = 0; i < out.size(); i++)
    {
        const double cx = (std::floor(xs[i] / VOXEL) + 0.5) * VOXEL;
        const double cy = (std::floor(ys[i] / VOXEL) + 0.5) * VOXEL;
        sx += xs[i] - cx;
        sy += ys[i] - cy;
    }
    const double n = static_cast<double>(out.size());
    return std::hypot(sx / n, sy / n);
}

double runAdaptive(mp2p_icp_filters::DecimateMethod method)
{
    auto mm = makeOrderedSheet();

    mp2p_icp_filters::FilterDecimateAdaptive f;
    mrpt::containers::yaml                   p;
    p["input_pointcloud_layer"]     = "raw";
    p["output_pointcloud_layer"]    = "out";
    p["voxel_size"]                 = VOXEL;
    p["desired_output_point_count"] = 196;  // one per occupied voxel
    p["decimate_method"]            = mrpt::typemeta::enum2str(method);
    f.initialize(p);
    f.filter(mm);

    auto out = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mm.layers.at("out"));
    ASSERT_(out);
    ASSERT_(out->size() > 100);
    return meanVoxelOffset(*out);
}
}  // namespace

int main()
{
    try
    {
        using mp2p_icp_filters::DecimateMethod;

        const double first    = runAdaptive(DecimateMethod::FirstPoint);
        const double rotating = runAdaptive(DecimateMethod::RotatingIndex);

        std::cout << "mean voxel offset: FirstPoint=" << first << " RotatingIndex=" << rotating
                  << "\n";

        // Taking the earliest point of every voxel must displace the cloud,
        // or the fixture is not exercising the effect at all:
        ASSERT_GT_(first, 0.1);

        // The rotating rule must remove most of that shared displacement:
        ASSERT_LT_(rotating, 0.5 * first);

        // Determinism: the choice comes from the voxel coordinates, so a
        // second run of the very same input must agree exactly.
        const double again = runAdaptive(DecimateMethod::RotatingIndex);
        ASSERT_EQUAL_(rotating, again);
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
