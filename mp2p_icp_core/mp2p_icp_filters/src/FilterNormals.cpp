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

#include <mp2p_icp_filters/FilterNormals.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>

#include <Eigen/Eigenvalues>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterNormals, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterNormals::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterNormals& parent)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    DECLARE_PARAMETER_IN_OPT(c, knn, parent);
    DECLARE_PARAMETER_IN_OPT(c, max_search_distance, parent);
    MCP_LOAD_OPT(c, parallelization_grain_size);
}

void FilterNormals::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c, *this);
}

void FilterNormals::filter(mp2p_icp::metric_map_t& inOut) const
{
    checkAllParametersAreRealized();
    ASSERT_GE_(params.knn, 3U);
    ASSERT_GT_(params.max_search_distance, 0);
    ASSERT_GT_(params.parallelization_grain_size, 0U);

    auto pcPtr = inOut.layer<mrpt::maps::CPointsMap>(params.pointcloud_layer);
    ASSERTMSG_(pcPtr, "FilterNormals: input layer not found or not a point cloud");
    auto& pc = *pcPtr;

    // Register the output fields before building the KD-tree:
    for (const char* f : {"normal_x", "normal_y", "normal_z"})
    {
        if (pc.getPointsBufferRef_float_field(f) != nullptr)
        {
            continue;
        }
        ASSERTMSG_(
            pc.registerField_float(f),
            "FilterNormals: the input layer class does not support extra fields");
    }

    const size_t N = pc.size();
    if (N == 0)
    {
        return;
    }

    auto* nxs = pc.getPointsBufferRef_float_field("normal_x");
    auto* nys = pc.getPointsBufferRef_float_field("normal_y");
    auto* nzs = pc.getPointsBufferRef_float_field("normal_z");
    ASSERT_(nxs && nys && nzs);

    const auto* vxs     = pc.getPointsBufferRef_float_field("view_x");
    const auto* vys     = pc.getPointsBufferRef_float_field("view_y");
    const auto* vzs     = pc.getPointsBufferRef_float_field("view_z");
    const bool  hasView = vxs && vys && vzs && vxs->size() == N;

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    pc.nn_prepare_for_3d_queries();

    const float maxDist2 = params.max_search_distance * params.max_search_distance;

    auto lambda_process = [&](size_t i)
    {
        thread_local std::vector<size_t> indices;
        thread_local std::vector<float>  dists_sq;

        (*nxs)[i] = 0;
        (*nys)[i] = 0;
        (*nzs)[i] = 0;

        pc.kdTreeNClosestPoint3DIdx(xs[i], ys[i], zs[i], params.knn, indices, dists_sq, maxDist2);
        if (indices.size() < 3)
        {
            return;
        }

        // Covariance of the neighbors, relative to the query point for
        // numerical accuracy:
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        Eigen::Matrix3f S    = Eigen::Matrix3f::Zero();
        for (const auto idx : indices)
        {
            const Eigen::Vector3f d(xs[idx] - xs[i], ys[idx] - ys[i], zs[idx] - zs[i]);
            mean += d;
            S.noalias() += d * d.transpose();
        }
        const auto k = static_cast<float>(indices.size());
        mean /= k;
        S = S / k - mean * mean.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.computeDirect(S, Eigen::ComputeEigenvectors);

        // Coincident or collinear neighbors do not define a plane:
        const auto& ev = es.eigenvalues();  // ascending order
        if (!(ev[1] > 1e-3f * ev[2]))
        {
            return;
        }
        Eigen::Vector3f n = es.eigenvectors().col(0);  // smallest eigenvalue

        if (hasView && n.x() * (*vxs)[i] + n.y() * (*vys)[i] + n.z() * (*vzs)[i] < 0)
        {
            n = -n;
        }
        (*nxs)[i] = n.x();
        (*nys)[i] = n.y();
        (*nzs)[i] = n.z();
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
}
