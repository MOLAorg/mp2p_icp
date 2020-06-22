/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   covariance.cpp
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */

#include <mp2p_icp/covariance.h>
#include <mrpt/math/num_jacobian.h>

#include <Eigen/Dense>

using namespace mp2p_icp;

mrpt::math::CMatrixDouble66 mp2p_icp::covariance(
    const Pairings&     finalPairings,
    const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& p)
{
    // If we don't have pairings, we can't provide an estimation:
    if (finalPairings.paired_points.empty())
    {
        mrpt::math::CMatrixDouble66 cov;
        cov.setDiagonal(1e6);
        return cov;
    }

    mrpt::math::CMatrixDouble61 xInitial;
    xInitial[0] = finalAlignSolution.x();
    xInitial[1] = finalAlignSolution.y();
    xInitial[0] = finalAlignSolution.x();
    xInitial[3] = finalAlignSolution.yaw();
    xInitial[4] = finalAlignSolution.pitch();
    xInitial[5] = finalAlignSolution.roll();

    mrpt::math::CMatrixDouble61 xIncrs;
    for (int i = 0; i < 3; i++) xIncrs[i] = p.finDif_xyz;
    for (int i = 0; i < 3; i++) xIncrs[3 + i] = p.finDif_angles;

    struct LambdaParams
    {
    };

    LambdaParams lmbParams;

    MRPT_TODO("Refactor this to reuse code in the optimal_tf files?");

    auto errorLambda = [&](const mrpt::math::CMatrixDouble61& x,
                           const LambdaParams&,
                           mrpt::math::CVectorDouble& errors) {
        mrpt::poses::CPose3D p;
        p.setFromValues(x[0], x[1], x[2], x[3], x[4], x[5]);

        const size_t nErrors = 3 * finalPairings.paired_points.size();
        ASSERT_(nErrors > 0);
        errors.resize(nErrors);

        size_t errorIdx = 0;

        for (const auto& pt2pt : finalPairings.paired_points)
        {
            double gx, gy, gz;
            p.composePoint(
                pt2pt.other_x, pt2pt.other_y, pt2pt.other_z, gx, gy, gz);

            errors[errorIdx++] = pt2pt.this_x - gx;
            errors[errorIdx++] = pt2pt.this_y - gy;
            errors[errorIdx++] = pt2pt.this_z - gz;
        }

        MRPT_TODO("Take into account lines and planes for the covariance too");

        // make sure all error values have been filled up:
        ASSERT_EQUAL_(errorIdx, nErrors);
    };

    Eigen::MatrixXd jacob;
    mrpt::math::estimateJacobian(
        xInitial,
        std::function<void(
            const mrpt::math::CMatrixDouble61&, const LambdaParams&,
            mrpt::math::CVectorDouble&)>(errorLambda),
        xIncrs, lmbParams, jacob);

    mrpt::math::CMatrixDouble66 hessian;
    hessian = jacob.transpose() * jacob;

    return hessian.inverse_LLt();
}

// other ideas?
// See: http://censi.mit.edu/pub/research/2007-icra-icpcov-slides.pdf
