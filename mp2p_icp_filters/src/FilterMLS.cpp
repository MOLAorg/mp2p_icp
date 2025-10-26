/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   FilterMLS.cpp
 * @brief  Applies a Moving Least Squares (MLS) filter to a point cloud.
 * @author Jose Luis Blanco and Google Gemini
 * @date   Oct 26, 2025
 */

#include <mp2p_icp_filters/FilterMLS.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/ops_containers.h>  // mrpt::math::meanAndCov
#include <mrpt/system/progress.h>
#include <mrpt/version.h>

// Use Eigen in the .cpp file only
#include <Eigen/Dense>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterMLS, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterMLS::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    MCP_LOAD_OPT(c, distinct_cloud_layer);
    MCP_LOAD_REQ(c, search_radius);
    MCP_LOAD_REQ(c, polynomial_order);
    MCP_LOAD_OPT(c, min_neighbors_for_fit);

    MCP_LOAD_OPT(c, projection_method);
    MCP_LOAD_OPT(c, upsampling_method);

    MCP_LOAD_OPT(c, parallelization_grain_size);
}

/**
 * @brief Stores the result of a local MLS surface fit,
 * inspired by pcl::MLSResult.
 *
 * This struct will live inside FilterMLS::Impl
 */
struct MLSResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool            valid = false;
    Eigen::Vector3d mean;
    Eigen::Vector3d plane_normal;
    Eigen::Vector3d u_axis;
    Eigen::Vector3d v_axis;
    Eigen::VectorXd c_vec;  // Polynomial coefficients
    int             order = 0;

    /**
     * @brief Computes the local surface fit for a query point and its neighbors.
     * This is a port of the logic from pcl::MovingLeastSquares.
     */
    void computeMLSSurface(
        const Eigen::Vector3d& query_point, const mrpt::maps::CPointsMap& pc,
        const std::vector<uint64_t>& neighbor_indices, int poly_order, double search_radius)
    {
        valid                    = false;
        order                    = poly_order;
        const size_t n_neighbors = neighbor_indices.size();

        // 1. Compute weighted mean and covariance matrix
        double total_weight = 0.0;
        mean.setZero();
        Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
        const double    sqr_radius        = search_radius * search_radius;

        std::vector<double>          weights(n_neighbors);
        std::vector<Eigen::Vector3d> points(n_neighbors);

        for (size_t i = 0; i < n_neighbors; ++i)
        {
            pc.getPoint(neighbor_indices[i], points[i].x(), points[i].y(), points[i].z());
            const double sqr_dist = (points[i] - query_point).squaredNorm();

            // Gaussian weight
            weights[i] = std::exp(-sqr_dist / sqr_radius);

            total_weight += weights[i];
            mean += weights[i] * points[i];
        }

        if (total_weight < 1e-10)
        {
            return;  // Avoid division by zero
        }
        mean /= total_weight;

        for (size_t i = 0; i < n_neighbors; ++i)
        {
            Eigen::Vector3d diff = points[i] - mean;
            covariance_matrix += weights[i] * diff * diff.transpose();
        }
        covariance_matrix /= total_weight;

        // 2. Compute eigenvectors to find the local plane (normal, u, v)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance_matrix);
        if (eigenSolver.info() != Eigen::Success)
        {
            return;
        }

        // The normal is the eigenvector corresponding to the smallest eigenvalue
        plane_normal = eigenSolver.eigenvectors().col(0);
        u_axis       = eigenSolver.eigenvectors().col(2);
        v_axis       = eigenSolver.eigenvectors().col(1);

        // Enforce a right-handed coordinate system
        v_axis = plane_normal.cross(u_axis);
        if (v_axis.dot(eigenSolver.eigenvectors().col(1)) < 0)
        {
            v_axis = -v_axis;
        }

        // 3. If order > 1, fit the polynomial
        if (order > 1)
        {
            int numCoeffs = (order + 1) * (order + 2) / 2;
            if (static_cast<int>(n_neighbors) < numCoeffs)
            {
                return;  // Not enough points to fit
            }

            Eigen::MatrixXd A(n_neighbors, numCoeffs);
            Eigen::VectorXd b(n_neighbors);
            Eigen::VectorXd w(n_neighbors);

            // Build the weighted least-squares problem A*c = b
            for (size_t i = 0; i < n_neighbors; ++i)
            {
                const auto            ii      = static_cast<Eigen::Index>(i);
                const Eigen::Vector3d diff    = points[i] - mean;
                const double          u       = diff.dot(u_axis);
                const double          v       = diff.dot(v_axis);
                const double          w_coord = diff.dot(plane_normal);

                // Polynomial terms: 1, v, v^2, ..., u, uv, u^2, ...
                int j = 0;
                for (int ui = 0; ui <= order; ++ui)
                {
                    for (int vi = 0; vi <= order - ui; ++vi)
                    {
                        A(ii, j++) = std::pow(u, ui) * std::pow(v, vi);
                    }
                }
                b(ii) = w_coord;
                w(ii) = weights[i];
            }

            // Solve for c (the coefficients) using Weighted LLS
            // (A^T * W * A) * c = A^T * W * b
            Eigen::MatrixXd AtW  = A.transpose() * w.asDiagonal();
            Eigen::MatrixXd AtWA = AtW * A;
            Eigen::VectorXd AtWb = AtW * b;

            c_vec = AtWA.llt().solve(AtWb);
            if (AtWA.llt().info() != Eigen::Success)
            {
                return;
            }
        }
        else  // order 1 (planar fit)
        {
            order = 1;
            c_vec.resize(3);  // 1, v, u
            c_vec.setZero();  // z = 0 for a simple plane at the mean
        }

        valid = true;
    }

    /**
     * @brief Projects a point onto the fitted surface using the SIMPLE method.
     */
    void projectPointSimple(
        const Eigen::Vector3d& pt, Eigen::Vector3d& projected_pt,
        Eigen::Vector3d& projected_normal) const
    {
        // 1. Compute (u, v) coordinates of 'pt' in the local frame
        const Eigen::Vector3d diff = pt - mean;
        const double          u    = diff.dot(u_axis);
        const double          v    = diff.dot(v_axis);

        double z     = 0.0;
        double dz_du = 0.0;
        double dz_dv = 0.0;

        // 2. Evaluate the polynomial z(u,v) and its partial derivatives
        if (order > 1)
        {
            int j = 0;
            for (int ui = 0; ui <= order; ++ui)
            {
                for (int vi = 0; vi <= order - ui; ++vi)
                {
                    z += c_vec[j] * std::pow(u, ui) * std::pow(v, vi);
                    if (ui > 0)
                    {
                        dz_du += c_vec[j] * ui * std::pow(u, ui - 1) * std::pow(v, vi);
                    }
                    if (vi > 0)
                    {
                        dz_dv += c_vec[j] * std::pow(u, ui) * vi * std::pow(v, vi - 1);
                    }
                    j++;
                }
            }
        }
        // else: z, dz_du, dz_dv remain 0 (planar fit)

        // 3. Calculate the projected point and normal
        // Projected point = mean + u*u_axis + v*v_axis + z*normal
        projected_pt = mean + u * u_axis + v * v_axis + z * plane_normal;

        // Normal = normal - dz/du * u_axis - dz/dv * v_axis
        // (This is derived from the tangent plane)
        projected_normal = plane_normal - dz_du * u_axis - dz_dv * v_axis;
        projected_normal.normalize();

        // Ensure normal points towards the original query point
        if (projected_normal.dot(pt - projected_pt) < 0)
        {
            projected_normal = -projected_normal;
        }
    }
};

// --- Main PIMPL struct ---
struct FilterMLS::Impl
{
#if defined(MP2P_HAS_TBB)
    // Thread-safe vectors for collecting results
    using TThreadSafePointVec  = tbb::concurrent_vector<mrpt::math::TPoint3Df>;
    using TThreadSafeNormalVec = tbb::concurrent_vector<mrpt::math::TPoint3Df>;
    using TThreadSafeIndexVec  = tbb::concurrent_vector<std::size_t>;
#else
    // Use standard vectors if TBB is not available
    using TThreadSafePointVec  = std::vector<mrpt::math::TPoint3Df>;
    using TThreadSafeNormalVec = std::vector<mrpt::math::TPoint3Df>;
    using TThreadSafeIndexVec  = std::vector<std::size_t>;
#endif

    TThreadSafePointVec  new_points;
    TThreadSafeIndexVec  new_points_source_index;
    TThreadSafeNormalVec new_normals;

    // Main processing function, to be called in parallel
    void process_point(
        size_t index, const mrpt::maps::CPointsMap& input_pc,
        const mrpt::maps::CPointsMap& query_pc, const Parameters& p)
    {
        mrpt::math::TPoint3Df p_query;
        query_pc.getPoint(index, p_query.x, p_query.y, p_query.z);

        // 1. Find neighbors in the *input* cloud
        std::vector<mrpt::math::TPoint3Df> neighbor_pts;
        std::vector<float>                 neighbor_dists_sq;
        std::vector<uint64_t>              neighbor_indices;

        input_pc.nn_radius_search(
            p_query, p.search_radius * p.search_radius, neighbor_pts, neighbor_dists_sq,
            neighbor_indices, 0 /*max results, 0=all*/
        );  //

        if (static_cast<int>(neighbor_indices.size()) < p.min_neighbors_for_fit)
        {
            return;  // Not enough points to fit
        }

        // 2. Compute MLS Surface
        MLSResult result;
        result.computeMLSSurface(
            Eigen::Vector3d(p_query.x, p_query.y, p_query.z), input_pc, neighbor_indices,
            p.polynomial_order, p.search_radius);

        if (!result.valid)
        {
            return;
        }

        // 3. Project point and compute normal
        Eigen::Vector3d projected_pt_eig, projected_normal_eig;

        // Only SIMPLE is implemented as requested
        result.projectPointSimple(
            Eigen::Vector3d(p_query.x, p_query.y, p_query.z), projected_pt_eig,
            projected_normal_eig);

        // 4. Store results
        new_points.push_back(
            mrpt::math::TPoint3D(projected_pt_eig.x(), projected_pt_eig.y(), projected_pt_eig.z())
                .cast<float>());
        new_points_source_index.push_back(index);
        new_normals.push_back(
            mrpt::math::TPoint3D(
                projected_normal_eig.x(), projected_normal_eig.y(), projected_normal_eig.z())
                .cast<float>());
    }
};

/* ----------------------------------------------------------------
                     FilterMLS implementation
   ---------------------------------------------------------------- */
FilterMLS::FilterMLS() : impl_(mrpt::make_impl<Impl>())
{
    mrpt::system::COutputLogger::setLoggerName("FilterMLS");
}

void FilterMLS::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);
    ASSERT_GT_(params.search_radius, 0.0);
    ASSERT_GE_(params.polynomial_order, 1);
    ASSERT_GE_(params.min_neighbors_for_fit, 3);
    ASSERT_GE_(params.parallelization_grain_size, 1);
    MRPT_END
}

void FilterMLS::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // 1. Get Input Point Cloud (the one used to build the surface)
    ASSERTMSG_(
        inOut.layers.count(params.input_pointcloud_layer),
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    auto pcPtr = mp2p_icp::MapToPointsMap(*inOut.layers.at(params.input_pointcloud_layer));
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Layer '%s' must be of point cloud type.", params.input_pointcloud_layer.c_str()));

    const auto& input_pc = *pcPtr;
    if (input_pc.empty())
    {
        MRPT_LOG_WARN_STREAM(
            "Input layer '" << params.input_pointcloud_layer << "' is empty, skipping filter.");
        return;
    }

    // 2. Get Query Point Cloud (the points to be projected)
    const mrpt::maps::CPointsMap* query_pc = &input_pc;

    if (params.upsampling_method == UpsamplingMethod::DISTINCT_CLOUD)
    {
        ASSERTMSG_(
            !params.distinct_cloud_layer.empty(),
            "UpsamplingMethod::DISTINCT_CLOUD requires 'distinct_cloud_layer' to be set.");
        ASSERTMSG_(
            inOut.layers.count(params.distinct_cloud_layer),
            mrpt::format(
                "Distinct cloud layer '%s' was not found.", params.distinct_cloud_layer.c_str()));

        auto distinctPcPtr =
            mp2p_icp::MapToPointsMap(*inOut.layers.at(params.distinct_cloud_layer));
        ASSERTMSG_(
            distinctPcPtr,
            mrpt::format(
                "Layer '%s' must be of point cloud type.", params.distinct_cloud_layer.c_str()));

        query_pc = distinctPcPtr;
    }

    const size_t nQueryPoints = query_pc->size();
    if (nQueryPoints == 0)
    {
        MRPT_LOG_WARN_STREAM("Query point cloud is empty, skipping filter.");
        return;
    }

    // 3. Prepare Output Point Cloud
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params.output_pointcloud_layer,
        /*do not allow empty*/ false,
        /* create cloud of the same type as input */
        pcPtr->GetRuntimeClass()->className);

    // Clear pimpl state from previous runs
    impl_->new_points.clear();
    impl_->new_points_source_index.clear();
    impl_->new_normals.clear();
#if !defined(MP2P_HAS_TBB)
    // Reserve memory if not using concurrent_vector
    impl_->new_points.reserve(nQueryPoints);
    impl_->new_points_source_index.reserve(nQueryPoints);
    impl_->new_normals.reserve(nQueryPoints);
#endif

    // 4. Build KD-Tree for the *input* cloud for fast neighbor search
    MRPT_LOG_INFO_STREAM("Building KD-Tree for " << input_pc.size() << " points...");
    input_pc.nn_prepare_for_3d_queries();  //
    MRPT_LOG_DEBUG("KD-Tree built.");

    // 5. Run parallel processing
#if defined(MP2P_HAS_TBB)
    const size_t grainsize = params.parallelization_grain_size;
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, nQueryPoints, grainsize),
        [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                impl_->process_point(i, input_pc, *query_pc, params);
            }
        });
#else
    // Non-TBB version
    for (size_t i = 0; i < nQueryPoints; ++i)
    {
        impl_->process_point(i, input_pc, *query_pc, params);
    }
#endif

    // 6. Copy results to output map
    const size_t nNewPoints = impl_->new_points.size();
    if (nNewPoints == 0)
    {
        MRPT_LOG_WARN("MLS filter produced 0 points.");
        return;
    }

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    // Register the normal fields in the output map.
    outPc->registerField_float("normal_x");  //
    outPc->registerField_float("normal_y");  //
    outPc->registerField_float("normal_z");  //

    // and the ones at origin cloud:
    outPc->registerPointFieldsFrom(input_pc);

    const auto ctx = outPc->prepareForInsertPointsFrom(input_pc);
#endif

    const size_t firstIdx = outPc->size();
    outPc->reserve(firstIdx + nNewPoints);

    auto& xs = const_cast<mrpt::aligned_std_vector<float>&>(outPc->getPointsBufferRef_x());
    auto& ys = const_cast<mrpt::aligned_std_vector<float>&>(outPc->getPointsBufferRef_y());
    auto& zs = const_cast<mrpt::aligned_std_vector<float>&>(outPc->getPointsBufferRef_z());

    for (size_t i = 0; i < nNewPoints; ++i)
    {
        // add point: this copies all existing fields:
#if MRPT_VERSION >= 0x020f00  // 2.15.0
        outPc->insertPointFrom(input_pc, impl_->new_points_source_index[i], ctx);
#else
        outPc->insertPointFrom(input_pc, impl_->new_points_source_index[i]);
#endif
        // Overwrite XYZ with new projected version:
        xs.back() = impl_->new_points[i].x;
        ys.back() = impl_->new_points[i].y;
        zs.back() = impl_->new_points[i].z;

#if MRPT_VERSION >= 0x020f00  // 2.15.0
        // Set normals
        outPc->insertPointField_float("normal_x", impl_->new_normals[i].x);
        outPc->insertPointField_float("normal_y", impl_->new_normals[i].y);
        outPc->insertPointField_float("normal_z", impl_->new_normals[i].z);
#endif
    }

    outPc->mark_as_modified();  //

    MRPT_LOG_INFO_STREAM(
        "MLS filter finished. Query points: " << nQueryPoints << ", Output points: " << nNewPoints);

    MRPT_END
}