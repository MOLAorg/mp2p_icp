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
 * @file   test-mp2p_voxel_grid_indexing.cpp
 * @brief  Regression test: every voxel of the voxel grids has the same size,
 *         including those touching the planes x=0, y=0 and z=0.
 */

#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mp2p_icp_filters/PointCloudToVoxelGridAverage.h>
#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <iostream>

namespace
{
using namespace mp2p_icp_filters;

constexpr float VOXEL = 1.0f;

/** Points along one axis, from -4.9 to +4.9 every 0.2, i.e. covering
 *  [-5, 5): with unit voxels they must fall into exactly 10 voxels. Rounding
 *  toward zero instead of down puts (-1,1) into a single voxel, giving 9.
 */
mrpt::maps::CSimplePointsMap pointsAlongAxis(int axis)
{
    mrpt::maps::CSimplePointsMap pts;
    for (int i = -49; i <= 49; i += 2)
    {
        float p[3] = {0.5f, 0.5f, 0.5f};
        p[axis]    = 0.1f * static_cast<float>(i);
        pts.insertPoint(p[0], p[1], p[2]);
    }
    return pts;
}

/** Two points 0.8 apart, on both sides of the plane `axis`=0: different voxels. */
mrpt::maps::CSimplePointsMap pointsAcrossZero(int axis)
{
    mrpt::maps::CSimplePointsMap pts;
    for (const float v : {-0.4f, 0.4f})
    {
        float p[3] = {0.5f, 0.5f, 0.5f};
        p[axis]    = v;
        pts.insertPoint(p[0], p[1], p[2]);
    }
    return pts;
}

template <class GRID, class PROCESS>
void checkGrid(const char* name, PROCESS&& process)
{
    for (int axis = 0; axis < 3; axis++)
    {
        {
            GRID grid;
            grid.setConfiguration(VOXEL, true);
            process(grid, pointsAlongAxis(axis));
            ASSERT_EQUAL_(grid.size(), 10UL);
        }
        {
            GRID grid;
            grid.setConfiguration(VOXEL, true);
            process(grid, pointsAcrossZero(axis));
            ASSERT_EQUAL_(grid.size(), 2UL);
        }
    }
    std::cout << "[voxel_grid_indexing] " << name << ": OK\n";
}

}  // namespace

int main()
{
    try
    {
        checkGrid<PointCloudToVoxelGrid>(
            "PointCloudToVoxelGrid", [](PointCloudToVoxelGrid& g, const mrpt::maps::CPointsMap& p)
            { g.processPointCloud(p); });

        checkGrid<PointCloudToVoxelGridSingle>(
            "PointCloudToVoxelGridSingle",
            [](PointCloudToVoxelGridSingle& g, const mrpt::maps::CPointsMap& p)
            { g.processPointCloud(p); });

        checkGrid<PointCloudToVoxelGridAverage>(
            "PointCloudToVoxelGridAverage",
            [](PointCloudToVoxelGridAverage& g, const mrpt::maps::CPointsMap& p)
            { g.processPointCloud(p, false); });

        // The coordinate-to-index rule must be the one documented for the
        // keys, also for negative coordinates:
        PointCloudToVoxelGrid g;
        g.setConfiguration(0.5f, true);
        ASSERT_EQUAL_(g.coord2idx(-0.01f), -1);
        ASSERT_EQUAL_(g.coord2idx(-0.5f), -1);
        ASSERT_EQUAL_(g.coord2idx(-0.51f), -2);
        ASSERT_EQUAL_(g.coord2idx(0.0f), 0);
        ASSERT_EQUAL_(g.coord2idx(0.49f), 0);
        ASSERT_EQUAL_(g.coord2idx(0.5f), 1);
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
