/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   QualityEvaluator_Voxels.cpp
 * @brief  Matching quality evaluator: comparison via voxel occupancy
 * @author Jose Luis Blanco Claraco
 * @date   July 14, 2020
 */

#include <mp2p_icp/QualityEvaluator_Voxels.h>
#include <mrpt/maps/CVoxelMap.h>

IMPLEMENTS_MRPT_OBJECT(QualityEvaluator_Voxels, QualityEvaluator, mp2p_icp)

using namespace mp2p_icp;

QualityEvaluator_Voxels::QualityEvaluator_Voxels()
{
    mrpt::system::COutputLogger::setLoggerName("QualityEvaluator_Voxels");
}

void QualityEvaluator_Voxels::initialize(  //
    const mrpt::containers::yaml& params)
{
    MCP_LOAD_REQ(params, voxel_layer_name);
    MCP_LOAD_OPT(params, dist2quality_scale);
}

namespace
{
double loss(double x, double y)
{
    /*

    D=[0 0 +1;0 0.5 0; 0 1 -10;...
       0.5 0 0; 0.5 0.5 0; 0.5 1 0;...
       1 0 -10; 1 0.5 0; 1 1 +1];
    x=D(:,1); y=D(:,2); z=D(:,3);

    sf = fit([x, y],z,'poly22')
    plot(sf,[x,y],z)

     */

    // return 1.0 - 2 * x - 2 * y + 4 * x * y;
    return 1.5 + x + y - 12 * x * x + 22 * x * y - 12 * y * y;
}
}  // namespace

QualityEvaluator::Result QualityEvaluator_Voxels::evaluate(
    const metric_map_t& pcGlobal, const metric_map_t& pcLocal,
    const mrpt::poses::CPose3D&      localPose,
    [[maybe_unused]] const Pairings& pairingsFromICP) const
{
    // Take both voxel maps from each map:
    // ------------------------------------
    if (pcGlobal.layers.count(voxel_layer_name) == 0)
        THROW_EXCEPTION_FMT(
            "Input global map was expected to contain a layer named '%s'",
            voxel_layer_name.c_str());

    if (pcLocal.layers.count(voxel_layer_name) == 0)
        THROW_EXCEPTION_FMT(
            "Input local map was expected to contain a layer named '%s'",
            voxel_layer_name.c_str());

    const mrpt::maps::CVoxelMap::Ptr globalVoxels =
        std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(
            pcGlobal.layers.at(voxel_layer_name));
    if (!globalVoxels)
        THROW_EXCEPTION_FMT(
            "Input global map was expected to contain a layer named '%s' of "
            "type 'CVoxelMap'",
            voxel_layer_name.c_str());

    const mrpt::maps::CVoxelMap::Ptr localVoxels =
        std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(
            pcLocal.layers.at(voxel_layer_name));

    if (!localVoxels)
        THROW_EXCEPTION_FMT(
            "Input local map was expected to contain a layer named '%s' of "
            "type 'CVoxelMap'",
            voxel_layer_name.c_str());

    // Compare them:
    // ----------------------------------

    // TODO(jlbc): Contribute upstream to Bonxai a "forEachCell() const":

    // get Bonxai grids:
    auto& g = const_cast<Bonxai::VoxelGrid<mrpt::maps::VoxelNodeOccupancy>&>(
        globalVoxels->grid());
    auto& l = const_cast<Bonxai::VoxelGrid<mrpt::maps::VoxelNodeOccupancy>&>(
        localVoxels->grid());

    auto gAccessor = g.createAccessor();
    auto lAccessor = l.createAccessor();

    // Kullback-Leibler distance:
    double dist       = 0;
    size_t dist_cells = 0;

    auto lmbdPerLocalVoxel = [&](mrpt::maps::CVoxelMap::voxel_node_t& data,
                                 const Bonxai::CoordT&                coord)
    {
        // get the corresponding cell in the global map:
        const auto ptLocal = Bonxai::CoordToPos(coord, l.resolution);
        const auto ptGlobal =
            localPose.composePoint({ptLocal.x, ptLocal.y, ptLocal.z});

        auto* cell = gAccessor.value(Bonxai::PosToCoord(
            {ptGlobal.x, ptGlobal.y, ptGlobal.z}, g.inv_resolution));
        if (!cell) return;  // cell not observed in global grid

        const float localOcc  = localVoxels->l2p(data.occupancy);
        const float globalOcc = globalVoxels->l2p(cell->occupancy);

        // barely observed cells?
        if (std::abs(globalOcc - 0.5f) < 0.01f ||
            std::abs(localOcc - 0.5f) < 0.01f)
            return;

        const double d = loss(localOcc, globalOcc);
        dist += d;
        dist_cells++;
    };  // end lambda for each voxel

    // run it:
    l.forEachCell(lmbdPerLocalVoxel);

    auto lmbdPerGlobalVoxel = [&](mrpt::maps::CVoxelMap::voxel_node_t& data,
                                  const Bonxai::CoordT&                coord)
    {
        // get the corresponding cell in the local map:
        const auto ptGlobal = Bonxai::CoordToPos(coord, l.resolution);
        const auto ptLocal =
            (-localPose).composePoint({ptGlobal.x, ptGlobal.y, ptGlobal.z});

        auto* cell = lAccessor.value(Bonxai::PosToCoord(
            {ptLocal.x, ptLocal.y, ptLocal.z}, l.inv_resolution));
        if (!cell) return;  // cell not observed in global grid

        const float localOcc  = localVoxels->l2p(cell->occupancy);
        const float globalOcc = globalVoxels->l2p(data.occupancy);

        // barely observed cells?
        if (std::abs(globalOcc - 0.5f) < 0.01f ||
            std::abs(localOcc - 0.5f) < 0.01f)
            return;

        const double d = loss(localOcc, globalOcc);
        dist += d;
        dist_cells++;
    };  // end lambda for each voxel

    g.forEachCell(lmbdPerGlobalVoxel);

    // const auto nTotalLocalCells = l.activeCellsCount();
    Result r;
    r.quality = 0;
    if (dist_cells)
    {
        dist /= dist_cells;
        r.quality = 1.0 / (1.0 + std::exp(-dist2quality_scale * dist));
    }
    MRPT_LOG_DEBUG_STREAM(
        "dist: " << dist << " dist_cells: " << dist_cells
                 << " quality: " << r.quality);

    return r;
}
