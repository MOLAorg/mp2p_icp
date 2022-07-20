/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterEdgesPlanes.cpp
 * @brief  Classify pointcloud voxels into planes / "edges".
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/FilterEdgesPlanes.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // dotProduct
#include <mrpt/obs/CObservation2DRangeScan.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterEdgesPlanes, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterEdgesPlanes::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);

    MCP_LOAD_REQ(c, voxel_filter_resolution);
    MCP_LOAD_REQ(c, voxel_filter_decimation);
    MCP_LOAD_REQ(c, full_pointcloud_decimation);
    MCP_LOAD_REQ(c, voxel_filter_max_e2_e0);
    MCP_LOAD_REQ(c, voxel_filter_max_e1_e0);
    MCP_LOAD_REQ(c, voxel_filter_min_e2_e0);
    MCP_LOAD_REQ(c, voxel_filter_min_e1_e0);
    MCP_LOAD_OPT(c, voxel_filter_min_e1);

    MCP_LOAD_OPT(c, init_extension_min_x);
    MCP_LOAD_OPT(c, init_extension_min_y);
    MCP_LOAD_OPT(c, init_extension_min_z);
    MCP_LOAD_OPT(c, init_extension_max_x);
    MCP_LOAD_OPT(c, init_extension_max_y);
    MCP_LOAD_OPT(c, init_extension_max_z);
}

FilterEdgesPlanes::FilterEdgesPlanes() = default;

void FilterEdgesPlanes::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    filter_grid_.resize(
        {params_.init_extension_min_x, params_.init_extension_min_y,
         params_.init_extension_min_z},
        {params_.init_extension_max_x, params_.init_extension_max_y,
         params_.init_extension_max_z},
        params_.voxel_filter_resolution);

    MRPT_END
}

void FilterEdgesPlanes::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Out:
    auto pc_edges           = mrpt::maps::CSimplePointsMap::Create();
    auto pc_planes          = mrpt::maps::CSimplePointsMap::Create();
    auto pc_full_decim      = mrpt::maps::CSimplePointsMap::Create();
    auto pc_plane_centroids = mrpt::maps::CSimplePointsMap::Create();

    inOut.layers["edge_points"]     = pc_edges;
    inOut.layers["plane_points"]    = pc_planes;
    inOut.layers["full_decim"]      = pc_full_decim;
    inOut.layers["plane_centroids"] = pc_plane_centroids;

    pc_edges->reserve(pc.size() / 10);
    pc_planes->reserve(pc.size() / 10);
    pc_full_decim->reserve(pc.size() / 10);
    pc_plane_centroids->reserve(pc.size() / 1000);

    filter_grid_.clear();
    filter_grid_.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const float max_e20 = params_.voxel_filter_max_e2_e0;
    const float max_e10 = params_.voxel_filter_max_e1_e0;
    const float min_e20 = params_.voxel_filter_min_e2_e0;
    const float min_e10 = params_.voxel_filter_min_e1_e0;
    const float min_e1  = params_.voxel_filter_min_e1;

    std::size_t nEdgeVoxels = 0, nPlaneVoxels = 0, nTotalVoxels = 0;
    for (const auto& vxl_pts : filter_grid_.pts_voxels)
    {
        if (!vxl_pts.indices.empty()) nTotalVoxels++;
        if (vxl_pts.indices.size() < 5) continue;

        // Analyze the voxel contents:
        mrpt::math::TPoint3Df mean{0, 0, 0};
        const float           inv_n = (1.0f / vxl_pts.indices.size());
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto pt_idx = vxl_pts.indices[i];
            mean.x += xs[pt_idx];
            mean.y += ys[pt_idx];
            mean.z += zs[pt_idx];
        }
        mean.x *= inv_n;
        mean.y *= inv_n;
        mean.z *= inv_n;

        mrpt::math::CMatrixFixed<double, 3, 3> mat_a;
        mat_a.setZero();
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto                  pt_idx = vxl_pts.indices[i];
            const mrpt::math::TPoint3Df a(
                xs[pt_idx] - mean.x, ys[pt_idx] - mean.y, zs[pt_idx] - mean.z);
            mat_a(0, 0) += a.x * a.x;
            mat_a(1, 0) += a.x * a.y;
            mat_a(2, 0) += a.x * a.z;
            mat_a(1, 1) += a.y * a.y;
            mat_a(2, 1) += a.y * a.z;
            mat_a(2, 2) += a.z * a.z;
        }
        mat_a *= inv_n;

        // Find eigenvalues & eigenvectors:
        // This only looks at the lower-triangular part of the cov matrix.
        mrpt::math::CMatrixFixed<double, 3, 3> eig_vectors;
        std::vector<double>                    eig_vals;
        mat_a.eig_symmetric(eig_vectors, eig_vals);

        const float e0 = eig_vals[0], e1 = eig_vals[1], e2 = eig_vals[2];

        mrpt::maps::CPointsMap* dest = nullptr;
        if (e2 < max_e20 * e0 && e1 < max_e10 * e0)
        {
            // Classified as EDGE
            // ------------------------
            nEdgeVoxels++;
            dest = pc_edges.get();
        }
        else if (e2 > min_e20 * e0 && e1 > min_e10 * e0 && e1 > min_e1)
        {
            // Classified as PLANE
            // ------------------------
            nPlaneVoxels++;

            // Define a plane from its centroid + a normal:
            const auto pl_c = mrpt::math::TPoint3D(mean);

            // Normal = largest eigenvector:
            const auto ev0 =
                eig_vectors.extractColumn<mrpt::math::TVector3D>(0);
            auto pl_n = mrpt::math::TVector3D(ev0.x, ev0.y, ev0.z);

            // Normal direction criterion: make it to face towards the vehicle.
            // We can use the dot product to find it out, since pointclouds are
            // given in vehicle-frame coordinates.
            {
                // Unit vector: vehicle -> plane centroid:
                ASSERT_GT_(pl_c.norm(), 1e-3);
                const auto u = pl_c * (1.0 / pl_c.norm());
                const auto dot_prod =
                    mrpt::math::dotProduct<3, double>(u, pl_n);

                // It should be <0 if the normal is pointing to the vehicle.
                // Otherwise, reverse the normal.
                if (dot_prod > 0) pl_n = -pl_n;
            }

            // Add plane & centroid:
            const auto pl = mrpt::math::TPlane3D(pl_c, pl_n);
            inOut.planes.emplace_back(pl, pl_c);

            // Also: add the centroid to this special layer:
            pc_plane_centroids->insertPointFast(pl_c.x, pl_c.y, pl_c.z);

            // Filter out horizontal planes, since their uneven density
            // makes ICP fail to converge.
            // A plane on the ground has its 0'th eigenvector like [0 0 1]
            if (std::abs(ev0.z) < 0.9f) { dest = pc_planes.get(); }
        }
        if (dest != nullptr)
        {
            for (size_t i = 0; i < vxl_pts.indices.size();
                 i += params_.voxel_filter_decimation)
            {
                const auto pt_idx = vxl_pts.indices[i];
                dest->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
        // full_pointcloud_decimation=0 means dont use this layer
        if (params_.full_pointcloud_decimation > 0)
        {
            for (size_t i = 0; i < vxl_pts.indices.size();
                 i += params_.full_pointcloud_decimation)
            {
                const auto pt_idx = vxl_pts.indices[i];
                pc_full_decim->insertPointFast(
                    xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
    }
    MRPT_LOG_DEBUG_STREAM(
        "[VoxelGridFilter] Voxel counts: total=" << nTotalVoxels
                                                 << " edges=" << nEdgeVoxels
                                                 << " planes=" << nPlaneVoxels);

    MRPT_END
}
