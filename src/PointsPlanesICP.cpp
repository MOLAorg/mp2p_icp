/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointsPlanesICP.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2_icp/PointsPlanesICP.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>
#include <Eigen/Dense>

using namespace mp2_icp;

void PointsPlanesICP::align_OLAE(
    const PointsPlanesICP::pointcloud_t& pcs1,
    const PointsPlanesICP::pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (const auto& kv1 : pcs1.point_layers)
    {
        // Ignore this layer?
        if (p.pt2pt_layers.count(kv1.first) == 0) continue;

        pointcount1 += kv1.second->size();
        pointcount2 += pcs2.point_layers.at(kv1.first)->size();
    }
    ASSERT_(pointcount1 > 0 || !pcs1.planes.empty());
    ASSERT_(pointcount2 > 0 || !pcs2.planes.empty());

    // ------------------------------------------------------
	// The mp2_icp ICP loop
    // ------------------------------------------------------
    auto solution      = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    auto prev_solution = solution;

    // Prepare params for "find pairings" for each layer:
    std::map<std::string, mrpt::maps::TMatchingParams> mps;

    // Find largest point cloud:
    std::string layerOfLargestPc;
    std::size_t pointCountLargestPc = 0;

    for (const auto& kv1 : pcs1.point_layers)
    {
        const bool is_layer_of_planes = (kv1.first == "plane_centroids"s);

        mrpt::maps::TMatchingParams& mp = mps[kv1.first];

        if (!is_layer_of_planes)
        {
            if (p.pt2pt_layers.count(kv1.first) == 0) continue;

            const auto& m1 = kv1.second;
            ASSERT_(m1);

            if (m1->size() > pointCountLargestPc)
            {
                pointCountLargestPc = m1->size();
                layerOfLargestPc    = kv1.first;
            }

            // Matching params for point-to-point:
            // Distance threshold
            mp.maxDistForCorrespondence = p.thresholdDist;

            // Angular threshold
            mp.maxAngularDistForCorrespondence = p.thresholdAng;
            mp.onlyKeepTheClosest              = true;
            mp.onlyUniqueRobust                = false;
            mp.decimation_other_map_points     = std::max(
                1U,
                static_cast<unsigned>(m1->size() / (1.0 * p.maxPairsPerLayer)));

            // For decimation: cycle through all possible points, even if we
            // decimate them, in such a way that different points are used
            // in each iteration.
            mp.offset_other_map_points = 0;
        }
        else
        {
            // Matching params for plane-to-plane (their centroids only at
            // this point):
            // Distance threshold: + extra since  plane centroids must not
            // show up at the same location
            mp.maxDistForCorrespondence = p.thresholdDist + 2.0;
            // Angular threshold
            mp.maxAngularDistForCorrespondence = 0.;
            mp.onlyKeepTheClosest              = true;
            mp.decimation_other_map_points     = 1;
        }
    }

    std::map<std::string, mrpt::maps::TMatchingExtraResults> mres;

    for (; result.nIterations < p.maxIterations; result.nIterations++)
    {
        // the global list of pairings:
        OLAE_Match_Input pairings;

        // Correspondences for each point layer:
        // ---------------------------------------
        // Find correspondences for each point cloud "layer":
        for (const auto& kv1 : pcs1.point_layers)
        {
            const auto &m1 = kv1.second, &m2 = pcs2.point_layers.at(kv1.first);
            ASSERT_(m1);
            ASSERT_(m2);

            const bool is_layer_of_planes = (kv1.first == "plane_centroids"s);

            // Ignore this layer?
            if (!is_layer_of_planes && p.pt2pt_layers.count(kv1.first) == 0)
                continue;

            auto& mp = mps.at(kv1.first);
            // Measure angle distances from the current estimate:
            mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(
                m2.get(), solution, mpl, mp, mres[kv1.first]);

            // Shuffle decimated points for next iter:
            if (++mp.offset_other_map_points >= mp.decimation_other_map_points)
                mp.offset_other_map_points = 0;

            // merge lists:
            // handle specially the plane-to-plane matching:
            if (!is_layer_of_planes)
            {
                // layer weight:
                const double lyWeight = p.pt2pt_layers.at(kv1.first);

                // A standard layer: point-to-point correspondences:
                pairings.paired_points.insert(
                    pairings.paired_points.end(), mpl.begin(), mpl.end());

                // and their weights:
                pairings.point_weights.emplace_back(mpl.size(), lyWeight);
            }
            else
            {
                // Plane-to-plane correspondence:

                // We have pairs of planes whose centroids are quite close.
                // Check their normals too:
                for (const auto& pair : mpl)
                {
                    // 1) Check fo pairing sanity:
                    ASSERTDEB_(pair.this_idx < pcs1.planes.size());
                    ASSERTDEB_(pair.other_idx < pcs2.planes.size());

                    const auto& p1 = pcs1.planes[pair.this_idx];
                    const auto& p2 = pcs2.planes[pair.other_idx];

                    const mrpt::math::TVector3D n1 = p1.plane.getNormalVector();
                    const mrpt::math::TVector3D n2 = p2.plane.getNormalVector();

                    // dot product to find the angle between normals:
                    const double dp = n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
                    const double n2n_ang = std::acos(dp);

                    // 2) append to list of plane pairs:
                    MRPT_TODO("Set threshold parameter");
                    if (n2n_ang < mrpt::DEG2RAD(5.0))
                    {
                        // Accept pairing:
                        pairings.paired_planes.emplace_back(p1, p2);
                    }
                }
            }
        }

        if (pairings.empty())
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            result.goodness          = 0;
            break;
        }

        // Compute the optimal pose, using the OLAE method
        // (Optimal linear attitude estimator)
        // ------------------------------------------------
#if 1
        OLAE_Match_Result res;

        // Weights: translation => trust points; attitude => trust planes
        pairings.weights.translation.planes = 0.0;
        pairings.weights.translation.points = 1.0;
        pairings.weights.attitude.planes    = p.relative_weight_planes_attitude;
        pairings.weights.attitude.points    = 1.0;

        pairings.use_robust_kernel = p.use_kernel;
        MRPT_TODO("make param");
        // pairings.robust_kernel_param = mrpt::DEG2RAD(0.05);
        // pairings.robust_kernel_scale = 1500.0;

        if (pairings.paired_points.size() >= 3)
        {
            // Skip ill-defined problems if the no. of points is too small.
            // There's no check for this inside olae_match() because it also
            // handled lines, planes, etc. but we don't want to rely on that for
            // this application.
            olae_match(pairings, res);
        }

        solution = mrpt::poses::CPose3D(res.optimal_pose);
#else
        mrpt::poses::CPose3DQuat estPoseQuat;
        double                   transf_scale;
        mrpt::tfest::se3_l2(
            pairings.paired_points, estPoseQuat, transf_scale,
            true /* DO force rigid transformation (scale=1) */);
        solution = mrpt::poses::CPose3D(estPoseQuat);
#endif

        // If matching has not changed, we are done:
        const auto deltaSol = solution - prev_solution;
        const mrpt::math::CVectorFixed<double, 6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.head<3>().norm();
        const double delta_rot = dSol.tail<3>().norm();

#if 0
        std::cout << "Dxyz: " << std::abs(delta_xyz)
                  << " Drot:" << std::abs(delta_rot)
                  << " p: " << solution.asString() << "\n";
#endif

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        prev_solution = solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

        // Ratio of points with a valid pairing:
#if 1
    if (!layerOfLargestPc.empty())
        result.goodness = mres.at(layerOfLargestPc).correspondencesRatio;
#else
    {
        mrpt::maps::TMatchingParams mp;

        // Matching params for point-to-point:
        // Distance threshold
        mp.maxDistForCorrespondence = 0.25f;
        mp.maxAngularDistForCorrespondence = 0;
        mp.onlyKeepTheClosest = true;
        mp.onlyUniqueRobust = false;
        mp.decimation_other_map_points = 60;

        const auto m1 = pcs1.point_layers.at("raw");
        const auto m2 = pcs2.point_layers.at("raw");

        mrpt::tfest::TMatchingPairList mpl;
        mrpt::maps::TMatchingExtraResults res;

        m1->determineMatching3D(m2.get(), result.optimal_tf.mean, mpl, mp, res);

        result.goodness = res.correspondencesRatio;
    }
#endif

    // Store output:
    result.optimal_tf.mean = solution;
    MRPT_TODO("covariance of the estimation");

    MRPT_END
}

static auto vector_rot_Z_90d_CCW(const mrpt::math::TVector3D& v)
{
    return mrpt::math::TVector3D(-v.y, v.x, v.z);
}

// Core of the OLAE algorithm.
// Factored out here so it can be re-evaluated if the solution is near the Gibbs
// vector singularity (|Phi|~= \pi)
// (Refer to technical report for details)
static std::tuple<Eigen::Matrix3d, Eigen::Vector3d> olae_build_linear_system(
    const PointsPlanesICP::OLAE_Match_Input& in,
    const mrpt::math::TPoint3D& ct_other, const mrpt::math::TPoint3D& ct_this,
    const bool do_relinearize_singularity)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Build the linear system: M g = v
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nLines + nPlanes;

    // weight of points, block by block:
    ASSERT_(!in.point_weights.empty());
    auto        cur_point_block_weights = in.point_weights.begin();
    std::size_t cur_point_block_start   = 0;

    // Normalized weights for attitude "waXX":
    double waPoints, waLines, waPlanes;
    {
        const auto wPt = in.weights.attitude.points,
                   wLi = in.weights.attitude.lines,
                   wPl = in.weights.attitude.planes;

        ASSERTMSG_(
            wPt + wLi + wPl > .0,
            "All, point, line, plane attidude weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPoints + wLi * nLines + wPl * nPlanes);
        waPoints     = wPt * k;
        waLines      = wLi * k;
        waPlanes     = wPl * k;
    }

    // Accumulator of robust kernel terms to normalize at the end:
    double robust_w_sum = .0;

    // Terms contributed by points & vectors have now the uniform form of
    // unit vectors:

    for (std::size_t i = 0; i < nAllMatches; i++)
    {
        // Get "bi" (this/global) & "ri" (other/local) vectors:
        TVector3D bi, ri;
        double    wi = .0;

        // Points, lines, planes, are all stored in sequence:
        if (i < nPoints)
        {
            // point-to-point pairing:  normalize(point-centroid)
            const auto& p = in.paired_points[i];
            wi            = waPoints;

            if (i >= cur_point_block_start + cur_point_block_weights->first)
            {
                ASSERT_(cur_point_block_weights != in.point_weights.end());
                ++cur_point_block_weights;  // move to next block
                cur_point_block_start = i;
            }
            wi *= cur_point_block_weights->second;
            // (solution will be normalized via robust_w_sum a the end)

            bi = TVector3D(p.this_x, p.this_y, p.this_z) - ct_this;
            ri = TVector3D(p.other_x, p.other_y, p.other_z) - ct_other;
            const auto bi_n = bi.norm(), ri_n = ri.norm();
            ASSERT_ABOVE_(bi_n, 1e-8);
            ASSERT_ABOVE_(ri_n, 1e-8);
            // (Note: ideally, both norms should be equal if noiseless and a
            // real pairing )
            bi *= 1.0 / bi_n;
            ri *= 1.0 / ri_n;
        }
        else if (i < nPoints + nLines)
        {
            // line-to-line pairing:
            wi = waLines;

            const auto idxLine = i - nPoints;
            MRPT_TODO("handle lines");
            THROW_EXCEPTION("handle lines");
        }
        else
        {
            // plane-to-plane pairing:
            wi = waPlanes;

            const auto idxPlane = i - (nPoints + nLines);
            bi = in.paired_planes[idxPlane].p_this.plane.getNormalVector();
            ri = in.paired_planes[idxPlane].p_other.plane.getNormalVector();

            ASSERTDEB_BELOW_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_BELOW_(std::abs(ri.norm() - 1.0), 0.01);
        }

        // If we are in a second stage, let's relinearize around a rotation
        // of +PI/2 along +Z, to avoid working near the Gibbs vector
        // singularity:
        if (do_relinearize_singularity)
        {
            // Rotate:
            ri = vector_rot_Z_90d_CCW(ri);
        }

        // Robust kernel:
        if (in.use_robust_kernel)
        {
            const double A = in.robust_kernel_param;
            const double B = in.robust_kernel_scale;
            const double ang =
                std::acos(ri.x * bi.x + ri.y * bi.y + ri.z * bi.z);
            double f = 1.0;
            if (ang > A) f = 1.0 / (1.0 + B * mrpt::square(ang - A));
            wi *= f;
        }

        ASSERT_(wi > .0);
        robust_w_sum += wi;

        // M+=(1/2)* ([s_i]_{x})^2
        // with: s_i = b_i + r_i
        const double sx = bi.x + ri.x, sy = bi.y + ri.y, sz = bi.z + ri.z;

        /* ([s_i]_{x})^2 is:
         *
         *  ⎡    2     2                          ⎤
         *  ⎢- sy  - sz      sx⋅sy        sx⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                 2     2             ⎥
         *  ⎢   sx⋅sy     - sx  - sz      sy⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                              2     2⎥
         *  ⎣   sx⋅sz        sy⋅sz     - sx  - sy ⎦
         */
        const double c00 = -sy * sy - sz * sz;
        const double c11 = -sx * sx - sz * sz;
        const double c22 = -sx * sx - sy * sy;
        const double c01 = sx * sy;
        const double c02 = sx * sz;
        const double c12 = sy * sz;

        // clang-format off
        const auto dM = (Eigen::Matrix3d() <<
           c00, c01, c02,
           c01, c11, c12,
           c02, c12, c22 ).finished();
        // clang-format on

        /* v-= [b_i]_{x}  r_i
         *  Each term is:
         *  ⎡by⋅rz - bz⋅ry ⎤
         *  ⎢              ⎥
         *  ⎢-bx⋅rz + bz⋅rx⎥
         *  ⎢              ⎥
         *  ⎣bx⋅ry - by⋅rx ⎦
         */

        // clang-format off
        const auto dV = (Eigen::Vector3d() <<
           (bi.y * ri.z - bi.z * ri.y),
           (-bi.x * ri.z + bi.z * ri.x),
           (bi.x * ri.y - bi.y * ri.x) ).finished();
        // clang-format on

        M += wi * dM;
        v -= wi * dV;
    }

    if (robust_w_sum)
    {
        M *= (1.0 / robust_w_sum);
        v *= (1.0 / robust_w_sum);
    }

    // The missing (1/2) from the formulas above:
    M *= 0.5;

    return {M, v};
    //
    MRPT_END
}

// "Markley, F. L., & Mortari, D. (1999). How to estimate attitude from
// vector observations."
static double olae_estimate_Phi(const double M_det, std::size_t n)
{
    return std::acos((M_det / (n == 2 ? -1.0 : -1.178)) - 1.);
}

// See .h docs, and associated technical report.
void PointsPlanesICP::olae_match(
    const OLAE_Match_Input& in, OLAE_Match_Result& result)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Note on notation: we are search the relative transformation of
    // the "other" frame wrt to "this", i.e. "this"="global",
    // "other"="local":
    //   p_this = pose \oplus p_other
    //   p_A    = pose \oplus p_B      --> pB = p_A \ominus pose

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nLines + nPlanes;

    // Reset output to defaults:
    result = OLAE_Match_Result();

    // Normalize weights for each feature type and for each target (attitude
    // / translation):
    ASSERT_(in.weights.attitude.points >= .0);
    ASSERT_(in.weights.attitude.lines >= .0);
    ASSERT_(in.weights.attitude.planes >= .0);
    ASSERT_(in.weights.translation.points >= .0);
    ASSERT_(in.weights.translation.planes >= .0);

    // Normalized weights for centroid "wcXX":
    double wcPoints, wcPlanes;
    {
        const auto wPt = in.weights.translation.points,
                   wPl = in.weights.translation.planes;

        ASSERTMSG_(
            wPt + wPl > .0,
            "Both, point and plane translation weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPoints + wPl * nPlanes);
        wcPoints     = wPt * k;
        wcPlanes     = wPl * k;
    }

    // Compute the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    {
        TPoint3D ct_other_pt(0, 0, 0), ct_this_pt(0, 0, 0);

        for (const auto& pair : in.paired_points)
        {
            ct_this_pt += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other_pt += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
        }
        ct_other_pt *= wcPoints;
        ct_this_pt *= wcPoints;

        // Add plane centroids to the computation of centroids as well:
        TPoint3D ct_other_pl(0, 0, 0), ct_this_pl(0, 0, 0);
        if (wcPlanes > 0)
        {
            for (const auto& pair : in.paired_planes)
            {
                ct_this_pl += pair.p_this.centroid;
                ct_other_pl += pair.p_other.centroid;
            }
            ct_this_pl *= wcPlanes;
            ct_other_pl *= wcPlanes;
        }

        // Normalize sum of centroids:
        ct_other = ct_other_pt + ct_other_pl;
        ct_this  = ct_this_pt + ct_this_pl;
    }

    // Build the linear system: M g = v
    const auto [M, v] = olae_build_linear_system(
        in, ct_other, ct_this, false /*dont relinearize singularity*/);

    // We are finding this optimal rotation, as a Gibbs vector:
    Eigen::Vector3d optimal_rot;

    // Solve linear system for optimal rotation:
    const double Md = M.determinant();

    // Estimate |Phi|:
    const double estPhi1 = olae_estimate_Phi(Md, nAllMatches);

    // Threshold to decide whether to do a re-linearization:
    const bool do_relinearize = (estPhi1 > mrpt::DEG2RAD(150.0));

    if (do_relinearize)
    {
        // relinearize on a different orientation:
        const auto [M2, v2] = olae_build_linear_system(
            in, ct_other, ct_this, true /*DO relinearize singularity*/);

        // Find the optimal Gibbs vector:
        optimal_rot = M2.colPivHouseholderQr().solve(v2);
    }
    else
    {
        // Find the optimal Gibbs vector:
        optimal_rot = M.colPivHouseholderQr().solve(v);
    }

    // Convert to quaternion by normalizing q=[1, optim_rot]:
    {
        auto       x = optimal_rot[0], y = optimal_rot[1], z = optimal_rot[2];
        const auto r = 1.0 / std::sqrt(1.0 + x * x + y * y + z * z);
        x *= r;
        y *= r;
        z *= r;
        auto q = mrpt::math::CQuaternionDouble(r, -x, -y, -z);

        // Quaternion to 3x3 rot matrix:
        result.optimal_pose = mrpt::poses::CPose3D(q, .0, .0, .0);
    }
    // Undo transformation above:
    if (do_relinearize)
    {
        result.optimal_pose = result.optimal_pose +
                              mrpt::poses::CPose3D(0, 0, 0, M_PI * 0.5, 0, 0);
    }

    // Use centroids to solve for optimal translation:
    mrpt::math::TPoint3D pp;
    result.optimal_pose.composePoint(
        ct_other.x, ct_other.y, ct_other.z, pp.x, pp.y, pp.z);

    result.optimal_pose.x(ct_this.x - pp.x);
    result.optimal_pose.y(ct_this.y - pp.y);
    result.optimal_pose.z(ct_this.z - pp.z);

    MRPT_END
}

/** Gets a renderizable view of all planes */
void PointsPlanesICP::pointcloud_t::planesAsRenderizable(
    mrpt::opengl::CSetOfObjects& o, const PointsPlanesICP::render_params_t& p)
{
    MRPT_START

    const float pw = p.plane_half_width, pf = p.plane_grid_spacing;

    for (const auto& plane : planes)
    {
        auto gl_pl =
            mrpt::opengl::CGridPlaneXY::Create(-pw, pw, -pw, pw, .0, pf);
        gl_pl->setColor_u8(p.plane_color);
        mrpt::math::TPose3D planePose;
        plane.plane.getAsPose3DForcingOrigin(plane.centroid, planePose);
        gl_pl->setPose(planePose);
        o.insert(gl_pl);
    }

    MRPT_END
}

void PointsPlanesICP::p2p_match(
    const PointsPlanesICP::P2P_Match_Input& in,
    PointsPlanesICP::P2P_Match_Result&      result)
{
    using std::size_t;

    MRPT_START

    // Run Gauss-Newton steps, using SE(3) relinearization at the current
    // solution:
    result.optimal_pose = in.initial_guess;

    const auto nPt2Pt = in.paired_points.size();
    const auto nPt2Pl = in.paired_pt2pl.size();

    const auto nErrorTerms = nPt2Pt * 3 + nPt2Pl;

    Eigen::VectorXd err(nErrorTerms);
    Eigen::MatrixXd J(nErrorTerms, 6);

    double w_pt = 1.0, w_pl = 50.0;

    for (size_t iter = 0; iter < in.max_iterations; iter++)
    {
        const auto dDexpe_de =
            mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(result.optimal_pose);

        // Point-to-point:
        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            // Error:
            const auto&  p  = in.paired_points[idx_pt];
            const double lx = p.other_x, ly = p.other_y, lz = p.other_z;
            double       gx, gy, gz;
            result.optimal_pose.composePoint(lx, ly, lz, gx, gy, gz);
            err(idx_pt * 3 + 0) = gx - p.this_x;
            err(idx_pt * 3 + 1) = gy - p.this_y;
            err(idx_pt * 3 + 2) = gz - p.this_z;

            // Eval Jacobian:
            // clang-format off
            const Eigen::Matrix<double, 3, 12> J1 =
                (Eigen::Matrix<double, 3, 12>() <<
                   lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,  0,
                    0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,
                    0,  0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1
                 ).finished();
            // clang-format on

            J.block<3, 6>(idx_pt * 3, 0) = w_pt * J1 * dDexpe_de.asEigen();
        }

        // Point-to-plane:
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto& p = in.paired_pt2pl[idx_pl];

            const double lx = p.pt_other.x, ly = p.pt_other.y,
                         lz = p.pt_other.z;
            mrpt::math::TPoint3D g;
            result.optimal_pose.composePoint(lx, ly, lz, g.x, g.y, g.z);

            err(idx_pl + nPt2Pt * 3) = p.pl_this.plane.evaluatePoint(g);

            // Eval Jacobian:
            // clang-format off
            const Eigen::Matrix<double, 3, 12> J1 =
                (Eigen::Matrix<double, 3, 12>() <<
                   lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,  0,
                    0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,
                    0,  0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1
                 ).finished();
            // clang-format on

            const Eigen::Matrix<double, 1, 3> Jpl =
                (Eigen::Matrix<double, 1, 3>() << p.pl_this.plane.coefs[0],
                 p.pl_this.plane.coefs[1], p.pl_this.plane.coefs[2])
                    .finished();

            const Eigen::Matrix<double, 1, 6> Jb =
                Jpl * J1 * dDexpe_de.asEigen();

            J.block<1, 6>(idx_pl + nPt2Pt * 3, 0) = w_pl * Jb;
        }

        // 3) Solve Gauss-Newton:
        const Eigen::VectorXd             g = J.transpose() * err;
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> delta =
            -H.colPivHouseholderQr().solve(g);

        // 4) add SE(3) increment:
        const auto dE = mrpt::poses::Lie::SE<3>::exp(
            mrpt::math::CVectorFixed<double, 6>(delta));

        result.optimal_pose = result.optimal_pose + dE;

        //        std::cout << "[P2P GN] iter:" << iter << " err:" << err.norm()
        //                  << " delta:" << delta.transpose() << "\n";

        // Simple convergence test:
        if (delta.norm() < in.min_delta) break;

    }  // for each iteration
    MRPT_END
}

void PointsPlanesICP::align(
    const PointsPlanesICP::pointcloud_t& pcs1,
    const PointsPlanesICP::pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // ------------------------------------------------------
	// The mp2_icp ICP loop
    // ------------------------------------------------------
    auto solution      = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    auto prev_solution = solution;

    // Prepare params for "find pairings" for each layer:
    std::map<std::string, mrpt::maps::TMatchingParams> mps;

    //    icp_params.pt2pt_layers.insert("color_bright"s);
    //    icp_params.pt2pt_layers.insert("non_planar"s);
    //    icp_params.pt2pl_layer = "plane_points"s;

    for (const auto& kv1 : pcs1.point_layers)
    {
        const bool is_layer_of_planes = (kv1.first == p.pt2pl_layer);

        mrpt::maps::TMatchingParams& mp = mps[kv1.first];

        if (!is_layer_of_planes)
        {
            const auto& m1 = kv1.second;
            ASSERT_(m1);

            // Matching params for point-to-point:
            // Distance threshold
            mp.maxDistForCorrespondence = p.thresholdDist;

            // Angular threshold
            mp.maxAngularDistForCorrespondence = p.thresholdAng;
            mp.onlyKeepTheClosest              = true;
            mp.onlyUniqueRobust                = false;
            mp.decimation_other_map_points     = std::max(
                1U,
                static_cast<unsigned>(m1->size() / (1.0 * p.maxPairsPerLayer)));

            // For decimation: cycle through all possible points, even if we
            // decimate them, in such a way that different points are used
            // in each iteration.
            mp.offset_other_map_points = 0;
        }
        else
        {
            // Matching params for plane-to-plane (their centroids only at
            // this point):
            // Distance threshold: + extra since  plane centroids must not
            // show up at the same location
            mp.maxDistForCorrespondence = p.thresholdDist + 2.0;
            // Angular threshold
            mp.maxAngularDistForCorrespondence = 0.;
            mp.onlyKeepTheClosest              = true;
            mp.decimation_other_map_points     = 1;
        }
    }

    for (; result.nIterations < p.maxIterations; result.nIterations++)
    {
        std::map<std::string, mrpt::maps::TMatchingExtraResults> mres;

        // the global list of pairings:
        P2P_Match_Input pairings;

        // Correspondences for each point layer:
        // ---------------------------------------
        // Find correspondences for each point cloud "layer":
        for (const auto& ptLyWeight : p.pt2pt_layers)
        {
            const auto&  ptLy     = ptLyWeight.first;  // layer name
            const double lyWeight = ptLyWeight.second;  // layer weight

            const auto &m1 = pcs1.point_layers.at(ptLy),
                       &m2 = pcs2.point_layers.at(ptLy);
            ASSERT_(m1);
            ASSERT_(m2);

            auto& mp = mps.at(ptLy);
            // Measure angle distances from the current estimate:
            mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(m2.get(), solution, mpl, mp, mres[ptLy]);

            // Shuffle decimated points for next iter:
            if (++mp.offset_other_map_points >= mp.decimation_other_map_points)
                mp.offset_other_map_points = 0;

            // A standard layer: point-to-point correspondences:
            pairings.paired_points.insert(
                pairings.paired_points.end(), mpl.begin(), mpl.end());

            // and their weights:
            pairings.point_weights.emplace_back(mpl.size(), lyWeight);

        }  // end for each point layer

        // point-to-planes
        if (!p.pt2pl_layer.empty())
        {
            const auto &m1 = pcs1.point_layers.at("plane_centroids"),
                       &m2 = pcs2.point_layers.at(p.pt2pl_layer);
            ASSERT_(m1);
            ASSERT_(m2);

            auto& mp = mps.at(p.pt2pl_layer);
            // Measure angle distances from the current estimate:
            mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(
                m2.get(), solution, mpl, mp, mres[p.pt2pl_layer]);
            // Plane-to-plane correspondence:

            // We have pairs of planes whose centroids are quite close.
            // Check their normals too:
            for (const auto& pair : mpl)
            {
                // 1) Check fo pairing sanity:
                ASSERTDEB_(pair.this_idx < pcs1.planes.size());
                ASSERTDEB_(pair.other_idx < m2->size());

                const auto&           pl_this = pcs1.planes[pair.this_idx];
                mrpt::math::TPoint3Df pt_other;
                m2->getPoint(
                    pair.other_idx, pt_other.x, pt_other.y, pt_other.z);

                // 2) append to list of plane pairs:
                // if (1)
                {
                    // Accept pairing:
                    pairings.paired_pt2pl.emplace_back(pl_this, pt_other);
                }
            }
        }

        if (pairings.empty())
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            result.goodness          = 0;
            break;
        }

#if 0
		std::cout << "[mp2_icp pairings] " << pairings.paired_points.size()
                  << " pt2pt " << pairings.paired_pt2pl.size() << " pt2pl.\n";
#endif

        // Compute the optimal pose, using the OLAE method
        // (Optimal linear attitude estimator)
        // ------------------------------------------------
        P2P_Match_Result res;

        pairings.use_robust_kernel = p.use_kernel;
        MRPT_TODO("make this a parameter");
        pairings.max_iterations = 10;

        p2p_match(pairings, res);

        solution = mrpt::poses::CPose3D(res.optimal_pose);

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

        prev_solution = solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Ratio of points with a valid pairing:
    // Evaluate with the raw point clouds:
    {
        // Matching params for point-to-point:
        // Reuse those of the last stage, stored in "mp" above.
        mrpt::maps::TMatchingParams mp;
        mp.maxDistForCorrespondence        = p.thresholdDist;
        mp.maxAngularDistForCorrespondence = p.thresholdAng;
        mp.onlyKeepTheClosest              = true;
        mp.onlyUniqueRobust                = false;

        MRPT_TODO("Rethink this, make a param,...");
        mp.decimation_other_map_points = 100;

        mrpt::tfest::TMatchingPairList    mpl;
        mrpt::maps::TMatchingExtraResults mres;

        pcs1.point_layers.at("raw")->determineMatching3D(
            pcs2.point_layers.at("raw").get(), solution, mpl, mp, mres);

        result.goodness = mres.correspondencesRatio;
    }

    // Store output:
    result.optimal_tf.mean = solution;
    MRPT_TODO("covariance of the estimation");

    MRPT_END
}
