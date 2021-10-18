/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_Voxels.cpp
 * @brief  Matching quality evaluator: comparison via voxel occupancy
 * @author Jose Luis Blanco Claraco
 * @date   July 14, 2020
 */

#include <mp2p_icp/QualityEvaluator_Voxels.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(QualityEvaluator_Voxels, QualityEvaluator, mp2p_icp)

using namespace mp2p_icp;

QualityEvaluator_Voxels::QualityEvaluator_Voxels()
{
    mrpt::system::COutputLogger::setLoggerName("QualityEvaluator_Voxels");
}

void QualityEvaluator_Voxels::initialize(  //
    const mrpt::containers::yaml& params)
{
    MCP_LOAD_OPT(params, resolution);
    MCP_LOAD_OPT(params, maxOccupancyUpdateCertainty);
    MCP_LOAD_OPT(params, maxFreenessUpdateCertainty);
    MCP_LOAD_OPT(params, dist2quality_scale);
}

static double loss(double x, double y)
{
    /*

    D=[0 0 +1;0 0.5 0; 0 1 -1;...
       0.5 0 0; 0.5 0.5 0; 0.5 1 0;...
       1 0 -1; 1 0.5 0; 1 1 +1];
    x=D(:,1); Y=D(:,2); z=D(:,3);

    sf = fit([x, y],z,'poly22')
    plot(sf,[x,y],z)

     */

    return 1.0 - 2 * x - 2 * y + 4 * x * y;
}

double QualityEvaluator_Voxels::evaluate(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D&      localPose,
    [[maybe_unused]] const Pairings& pairingsFromICP) const
{
    // Build both voxel maps:
    // ----------------------------------
    const auto r = resolution;

    mrpt::maps::COccupancyGridMap3D voxelsGlo({-r, -r, -r}, {r, r, r}, r);
    mrpt::maps::COccupancyGridMap3D voxelsLoc({-r, -r, -r}, {r, r, r}, r);
    // voxels default occupancy probability: 0.5

    auto& io = voxelsGlo.insertionOptions;

    io.maxOccupancyUpdateCertainty = maxOccupancyUpdateCertainty;
    io.maxFreenessUpdateCertainty  = maxFreenessUpdateCertainty;

    for (const auto& ly : pointLayers)
    {
        auto itG = pcGlobal.layers.find(ly);
        auto itL = pcLocal.layers.find(ly);
        if (itG == pcGlobal.layers.end() || itL == pcLocal.layers.end())
        {
            MRPT_LOG_ERROR_FMT(
                "Layer `%s` not found in both global/local layers.",
                ly.c_str());
            return 0;
        }

        auto ptsG =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itG->second);
        auto ptsL =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itL->second);
        ASSERT_(ptsG);
        ASSERT_(ptsL);

        mrpt::maps::CSimplePointsMap localTransformed;
        localTransformed.insertAnotherMap(ptsL.get(), localPose);

        // resize voxel grids?
        MRPT_TODO("Check against current size too, for many layers");
#if MRPT_VERSION >= 0x218
        {
            const auto bb = ptsG->boundingBox();
            voxelsGlo.resizeGrid(bb.min, bb.max);
        }
        {
            const auto bb = localTransformed.boundingBox();
            voxelsLoc.resizeGrid(bb.min, bb.max);
        }
#else
        {
            mrpt::math::TPoint3D bbmax, bbmin;
            ptsG->boundingBox(bbmin, bbmax);
            voxelsGlo.resizeGrid(bbmin, bbmax);
        }
        {
            mrpt::math::TPoint3D bbmax, bbmin;
            localTransformed.boundingBox(bbmin, bbmax);
            voxelsLoc.resizeGrid(bbmin, bbmax);
        }
#endif
        // insert:
        voxelsGlo.insertPointCloud({0, 0, 0}, *ptsG);
        voxelsLoc.insertPointCloud(
            mrpt::math::TPoint3D(localPose.asTPose()), localTransformed);
    }

    // Compare them:
    // ----------------------------------
    const auto& g = voxelsGlo.m_grid;
    const auto& l = voxelsLoc.m_grid;

    // Kullback-Leibler distance:
    double dist       = 0;
    size_t dist_cells = 0;

    for (size_t igx = 0; igx < g.getSizeX(); igx++)
    {
        const int ilx = l.x2idx(g.idx2x(igx));
        if (ilx < 0 || static_cast<size_t>(ilx) >= l.getSizeX()) continue;

        for (size_t igy = 0; igy < g.getSizeY(); igy++)
        {
            const int ily = l.y2idx(g.idx2y(igy));
            if (ily < 0 || static_cast<size_t>(ily) >= l.getSizeY()) continue;

            for (size_t igz = 0; igz < g.getSizeZ(); igz++)
            {
                const int ilz = l.z2idx(g.idx2z(igz));
                if (ilz < 0 || static_cast<size_t>(ilz) >= l.getSizeZ())
                    continue;

                // derreferencing should be safe given all checks above: don't
                // throw exceptions to allow compiler optimizations:
                const auto gCell = *g.cellByIndex(igx, igy, igz);
                const auto lCell = *l.cellByIndex(ilx, ily, ilz);

                const float gProb = voxelsGlo.l2p(gCell);
                const float lProb = voxelsLoc.l2p(lCell);

                if (std::abs(gProb - 0.5) < 0.01 ||
                    std::abs(lProb - 0.5) < 0.01)
                    continue;

                const double d = loss(lProb, gProb);
                dist += d;

                dist_cells++;
            }
        }
    }

    double quality = 0;

    if (dist_cells)
    {
        dist /= dist_cells;
        quality = 1.0 / (1.0 + std::exp(-dist2quality_scale * dist));
    }
#if 0
    MRPT_LOG_WARN_STREAM(
        "dist: " << dist << " dist_cells: " << dist_cells
                 << " quality: " << quality);
#endif

    return quality;
}
