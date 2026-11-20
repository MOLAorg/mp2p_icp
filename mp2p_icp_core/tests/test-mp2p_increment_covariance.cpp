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
 * @file   test-mp2p_increment_covariance.cpp
 * @brief  IncrementCovarianceModel reproduces the training code's covariances
 *         (golden vectors from real scans) and rejects malformed model files.
 */

#include <mp2p_icp/IncrementCovarianceModel.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
const std::string kSourceDir = MP2P_SOURCE_DIR;

void test_golden_vectors()
{
    mp2p_icp::IncrementCovarianceModel model;
    model.load(kSourceDir + "/models/husky_ouster_gicp_se3_v1.yaml");
    ASSERT_(model.loaded());

    std::ifstream f(kSourceDir + "/tests/data/increment_covariance_golden.txt");
    ASSERT_(f.is_open());
    std::string line;
    size_t      n      = 0;
    double      maxRel = 0;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream  ss(line);
        std::vector<double> v;
        double              x = 0;
        while (ss >> x)
        {
            v.push_back(x);
        }
        ASSERT_EQUAL_(v.size(), 21U + 2U + 3U + 3U + 1U + 21U);

        mp2p_icp::IncrementCovarianceModel::Input in;
        size_t                                    k = 0;
        for (int i = 0; i < 6; i++)
        {
            for (int j = i; j < 6; j++)
            {
                in.measurement_information(i, j) = v[k];
                in.measurement_information(j, i) = v[k];
                k++;
            }
        }
        in.iterations = static_cast<size_t>(v[k++]);
        in.quality    = v[k++];
        const mrpt::math::TVector3D t(v[k], v[k + 1], v[k + 2]);
        k += 3;
        mrpt::math::CVectorFixedDouble<3> w;
        w[0] = v[k];
        w[1] = v[k + 1];
        w[2] = v[k + 2];
        k += 3;
        in.increment =
            mrpt::poses::CPose3D::FromRotationAndTranslation(mrpt::poses::Lie::SO<3>::exp(w), t);
        in.dt = v[k++];

        const auto Q = model.predict(in);
        for (int i = 0; i < 6; i++)
        {
            for (int j = i; j < 6; j++)
            {
                const double expected = v[k++];
                const double scale    = std::sqrt(Q(i, i) * Q(j, j));
                const double rel      = std::abs(Q(i, j) - expected) / scale;
                maxRel                = std::max(maxRel, rel);
            }
        }
        n++;
    }
    std::cout << "golden vectors: " << n << " scans, max relative error " << maxRel << "\n";
    ASSERT_(n >= 10);
    ASSERT_LT_(maxRel, 1e-9);
}

void test_rejects_bad_files()
{
    mp2p_icp::IncrementCovarianceModel model;
    model.load(kSourceDir + "/models/husky_ouster_gicp_se3_v1.yaml");
    const std::string desc = model.description();

    bool threw = false;
    try
    {
        model.load(mrpt::containers::yaml::FromText("format: something_else\nversion: 1\n"));
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    ASSERT_(threw);
    // A failed load keeps the previous model:
    ASSERT_(model.loaded());
    ASSERT_EQUAL_(model.description(), desc);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_golden_vectors();
        test_rejects_bad_files();
        std::cout << "Test successful.\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
