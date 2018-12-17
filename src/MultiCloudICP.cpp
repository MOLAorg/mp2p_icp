/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MultiCloudICP.cpp
 * @brief  ICP registration for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */

#include <mp2_icp/MultiCloudICP.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>
#include <Eigen/Dense>

using namespace mp2_icp;

void MultiCloudICP::align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    MRPT_START

    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.size(), pcs2.size());
    ASSERT_(!pcs1.empty());
    const auto nLayers = pcs1.size();
    ASSERT_(nLayers >= 1);

    // Reset output:
    result = Results();

    // Matching params:
    mrpt::maps::TMatchingParams mp;
    // Distance threshold
    mp.maxDistForCorrespondence = p.thresholdDist;
    // Angular threshold
    mp.maxAngularDistForCorrespondence = p.thresholdAng;
    mp.onlyKeepTheClosest              = true;
    mp.onlyUniqueRobust                = false;
    mp.decimation_other_map_points     = p.corresponding_points_decimation;

    // For decimation: cycle through all possible points, even if we decimate
    // them, in such a way that different points are used in each iteration.
    mp.offset_other_map_points = 0;

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (size_t layer = 0; layer < nLayers; layer++)
    {
        pointcount1 += pcs1[layer]->size();
        pointcount2 += pcs2[layer]->size();
        ASSERT_(
            (pcs1[layer]->size() == 0 && pcs2[layer]->size() == 0) ||
            (pcs1[layer]->size() > 0 && pcs2[layer]->size() > 0));
    }

    ASSERT_(pointcount1 > 0);
    ASSERT_(pointcount2 > 0);

    // ------------------------------------------------------
    //					The ICP loop
    // ------------------------------------------------------
    mrpt::poses::CPose3D solution = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    mrpt::poses::CPose3D prev_solution = solution;
    for (; result.nIterations < p.maxIterations; result.nIterations++)
    {
        std::vector<mrpt::maps::TMatchingExtraResults> mres(nLayers);

        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

        // the global list of pairings, including all layers:
        mrpt::tfest::TMatchingPairList pairings;

        // Find correspondences for each point cloud "layer":
        for (size_t layer = 0; layer < nLayers; layer++)
        {
            const auto &m1 = pcs1[layer], &m2 = pcs2[layer];
            ASSERT_(m1);
            ASSERT_(m2);

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(m2.get(), solution, mpl, mp, mres[layer]);

            // merge lists:
            pairings.insert(pairings.end(), mpl.begin(), mpl.end());

        }  // end for each "layer"

        // Ratio of points with a valid pairing:
        result.goodness = p.corresponding_points_decimation * pairings.size() /
                          static_cast<double>(pointcount1);

        if (pairings.empty())
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            break;
        }

        // Compute the estimated pose, using Horn's method:
        mrpt::poses::CPose3DQuat estPoseQuat;
        double                   transf_scale;
        mrpt::tfest::se3_l2(
            pairings, estPoseQuat, transf_scale,
            true /* DO force rigid transformation (scale=1) */);
        solution = mrpt::poses::CPose3D(estPoseQuat);

        // If matching has not changed, we are done:
        const auto deltaSol = solution - prev_solution;
        const mrpt::math::CVectorFixed<double, 6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.head<3>().norm();
        const double delta_rot = dSol.tail<3>().norm();

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        // Shuffle decimated points for next iter:
        if (++mp.offset_other_map_points >= p.corresponding_points_decimation)
            mp.offset_other_map_points = 0;

        prev_solution = solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Store output:
    result.optimal_tf.mean = solution;
    MRPT_TODO("covariance of the estimation");

    MRPT_END
}
