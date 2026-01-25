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
 * @file   test-mp2p_estimate_points_eigen.cpp
 * @brief  Unit test for estimate_points_eigen
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/estimate_points_eigen.h>
#include <mrpt/math/CMatrixFixed.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{

void test_planar_points()
{
    // Create points on the XY plane
    std::vector<float> xs, ys, zs;

    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            xs.push_back(static_cast<float>(i));
            ys.push_back(static_cast<float>(j));
            zs.push_back(0.0f);  // All on Z=0 plane
        }
    }

    auto result = estimate_points_eigen(xs.data(), ys.data(), zs.data(), std::nullopt, xs.size());

    // Mean should be at center of grid
    ASSERT_NEAR_(result.meanCov.mean.x(), 4.5, 0.1);
    ASSERT_NEAR_(result.meanCov.mean.y(), 4.5, 0.1);
    ASSERT_NEAR_(result.meanCov.mean.z(), 0.0, 0.1);

    // For planar points, smallest eigenvalue should be near zero
    ASSERT_LT_(result.eigVals[0], 0.1);

    // Smallest eigenvector should point in Z direction (normal to plane)
    ASSERT_NEAR_(std::abs(result.eigVectors[0].z), 1.0, 0.1);

    // Eigenvalues should be sorted
    ASSERT_LE_(result.eigVals[0], result.eigVals[1]);
    ASSERT_LE_(result.eigVals[1], result.eigVals[2]);
}

void test_linear_points()
{
    // Create points along a line (X axis)
    std::vector<float> xs, ys, zs;

    for (int i = 0; i < 20; i++)
    {
        xs.push_back(static_cast<float>(i) * 0.1f);
        ys.push_back(0.0f);
        zs.push_back(0.0f);
    }

    auto result = estimate_points_eigen(xs.data(), ys.data(), zs.data(), std::nullopt, xs.size());

    // Mean should be at midpoint
    ASSERT_NEAR_(result.meanCov.mean.x(), 0.95, 0.1);
    ASSERT_NEAR_(result.meanCov.mean.y(), 0.0, 0.01);
    ASSERT_NEAR_(result.meanCov.mean.z(), 0.0, 0.01);

    // For linear points, two smallest eigenvalues should be near zero
    ASSERT_LT_(result.eigVals[0], 0.01);
    ASSERT_LT_(result.eigVals[1], 0.01);

    // Largest eigenvector should point along X axis
    ASSERT_NEAR_(std::abs(result.eigVectors[2].x), 1.0, 0.1);
}

void test_spherical_distribution()
{
    // Create points distributed in a sphere
    std::vector<float> xs, ys, zs;

    for (int i = -5; i <= 5; i++)
    {
        for (int j = -5; j <= 5; j++)
        {
            for (int k = -5; k <= 5; k++)
            {
                const auto r2 = i * i + j * j + k * k;
                if (r2 <= 25)  // Radius 5
                {
                    xs.push_back(static_cast<float>(i));
                    ys.push_back(static_cast<float>(j));
                    zs.push_back(static_cast<float>(k));
                }
            }
        }
    }

    auto result = estimate_points_eigen(xs.data(), ys.data(), zs.data(), std::nullopt, xs.size());

    // Mean should be at origin
    ASSERT_NEAR_(result.meanCov.mean.x(), 0.0, 0.5);
    ASSERT_NEAR_(result.meanCov.mean.y(), 0.0, 0.5);
    ASSERT_NEAR_(result.meanCov.mean.z(), 0.0, 0.5);

    // For spherical distribution, eigenvalues should be similar
    double ratio1 = result.eigVals[1] / result.eigVals[0];
    double ratio2 = result.eigVals[2] / result.eigVals[1];

    ASSERT_GT_(ratio1, 0.5);
    ASSERT_LT_(ratio1, 2.0);
    ASSERT_GT_(ratio2, 0.5);
    ASSERT_LT_(ratio2, 2.0);
}

void test_with_indices()
{
    // Create a set of points, but only use some via indices
    std::vector<float> xs = {0, 1, 2, 3, 4, 5};
    std::vector<float> ys = {0, 0, 0, 0, 0, 0};
    std::vector<float> zs = {0, 0, 0, 0, 0, 0};

    // Only use points at indices 1, 2, 3
    std::vector<size_t> indices = {1, 2, 3};

    auto result = estimate_points_eigen(xs.data(), ys.data(), zs.data(), indices, std::nullopt);

    // Mean should be at x=2
    ASSERT_NEAR_(result.meanCov.mean.x(), 2.0, 0.01);
    ASSERT_NEAR_(result.meanCov.mean.y(), 0.0, 0.01);
    ASSERT_NEAR_(result.meanCov.mean.z(), 0.0, 0.01);
}

void test_vector_of_points_conversion()
{
    std::vector<mrpt::math::TPoint3Df> pts;
    pts.push_back({1.0f, 2.0f, 3.0f});
    pts.push_back({4.0f, 5.0f, 6.0f});
    pts.push_back({7.0f, 8.0f, 9.0f});

    std::vector<float> xs, ys, zs;
    vector_of_points_to_xyz(pts, xs, ys, zs);

    ASSERT_EQUAL_(xs.size(), 3);
    ASSERT_EQUAL_(ys.size(), 3);
    ASSERT_EQUAL_(zs.size(), 3);

    ASSERT_EQUAL_(xs[0], 1.0f);
    ASSERT_EQUAL_(ys[0], 2.0f);
    ASSERT_EQUAL_(zs[0], 3.0f);

    ASSERT_EQUAL_(xs[1], 4.0f);
    ASSERT_EQUAL_(ys[1], 5.0f);
    ASSERT_EQUAL_(zs[1], 6.0f);

    ASSERT_EQUAL_(xs[2], 7.0f);
    ASSERT_EQUAL_(ys[2], 8.0f);
    ASSERT_EQUAL_(zs[2], 9.0f);
}

void test_insufficient_points()
{
    // Test with less than 3 points - should throw exception
    std::vector<float> xs = {0, 1};
    std::vector<float> ys = {0, 0};
    std::vector<float> zs = {0, 0};

    bool exceptionThrown = false;
    try
    {
        auto result =
            estimate_points_eigen(xs.data(), ys.data(), zs.data(), std::nullopt, xs.size());
    }
    catch (const std::exception&)
    {
        exceptionThrown = true;
    }

    ASSERT_(exceptionThrown);
}

void test_covariance_matrix()
{
    // Create points with known covariance
    std::vector<float> xs, ys, zs;

    // Points spread more in X than Y
    for (int i = 0; i < 10; i++)
    {
        xs.push_back(static_cast<float>(i) * 2.0f);  // Spread of 18
        ys.push_back(static_cast<float>(i) * 0.5f);  // Spread of 4.5
        zs.push_back(0.0f);
    }

    auto result = estimate_points_eigen(xs.data(), ys.data(), zs.data(), std::nullopt, xs.size());

    // Check that covariance matrix is populated
    const auto& cov = result.meanCov.cov;

    // Variance in X should be larger than in Y
    ASSERT_GT_(cov(0, 0), cov(1, 1));

    // Variance in Z should be near zero
    ASSERT_LT_(cov(2, 2), 0.01);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_planar_points();
        std::cout << "test_planar_points: Success ✅" << std::endl;

        test_linear_points();
        std::cout << "test_linear_points: Success ✅" << std::endl;

        test_spherical_distribution();
        std::cout << "test_spherical_distribution: Success ✅" << std::endl;

        test_with_indices();
        std::cout << "test_with_indices: Success ✅" << std::endl;

        test_vector_of_points_conversion();
        std::cout << "test_vector_of_points_conversion: Success ✅" << std::endl;

        test_insufficient_points();
        std::cout << "test_insufficient_points: Success ✅" << std::endl;

        test_covariance_matrix();
        std::cout << "test_covariance_matrix: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
