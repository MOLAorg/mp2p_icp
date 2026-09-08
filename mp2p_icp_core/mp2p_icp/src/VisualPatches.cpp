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
 * @file   VisualPatches.cpp
 * @brief  Photometric patches anchored to map points ("virtual patches").
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 */

#include <mp2p_icp/VisualPatches.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <ostream>

#include "visual_patch_terms.h"

using namespace mp2p_icp;

namespace
{
/** Optional diagnostic: append the mean-normalized residual of sampled patches
 *  to a TSV file, one row per patch, so the spatial correlation INSIDE a patch
 *  can be measured.
 *
 *  Enabled only if MP2P_ICP_VISUAL_RESIDUAL_FILE is set, so it costs one cached
 *  lookup when unused. It exists because a patch does not supply as many
 *  independent measurements as it has pixels, and the chi-square per degree of
 *  freedom that sets this term's weight is wrong by exactly that factor.
 *
 *  Not thread-safe by design: it is for single-threaded diagnostic runs.
 */
std::atomic<uint64_t> residualCallCounter{0};

std::ostream* residualStream()
{
    static std::unique_ptr<std::ofstream> s_file = []() -> std::unique_ptr<std::ofstream>
    {
        const char* path = ::getenv("MP2P_ICP_VISUAL_RESIDUAL_FILE");
        if (!path || !path[0])
        {
            return {};
        }
        auto f = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::app);
        if (!f->is_open() || !f->good())
        {
            return {};
        }
        return f;
    }();
    if (s_file && !s_file->good())
    {
        s_file.reset();
    }
    return s_file ? s_file.get() : nullptr;
}

/// Only every N-th solver call is dumped: every patch of every inner iteration
/// would be millions of rows for one mission, and the statistic converges long
/// before that.
constexpr uint64_t kResidualDumpDecimation = 97;

/** Forward projection of a normalized image-plane point through the declared
 *  distortion model. Returns the distorted normalized coordinates. */
void distortNormalized(const mrpt::img::TCamera& cam, double xn, double yn, double& xd, double& yd)
{
    switch (cam.distortion)
    {
        case mrpt::img::DistortionModel::none:
        {
            xd = xn;
            yd = yn;
            break;
        }
        case mrpt::img::DistortionModel::plumb_bob:
        {
            const double k1 = cam.dist[0];
            const double k2 = cam.dist[1];
            const double p1 = cam.dist[2];
            const double p2 = cam.dist[3];
            const double k3 = cam.dist[4];

            const double r2  = xn * xn + yn * yn;
            const double rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));

            xd = xn * rad + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
            yd = yn * rad + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;
            break;
        }
        case mrpt::img::DistortionModel::kannala_brandt:
        {
            // MRPT stores the 4 fisheye coefficients as [k1 k2 * * k3 k4]:
            const double k1 = cam.dist[0];
            const double k2 = cam.dist[1];
            const double k3 = cam.dist[4];
            const double k4 = cam.dist[5];

            const double r = std::sqrt(xn * xn + yn * yn);
            if (r < 1e-9)
            {
                xd = xn;
                yd = yn;
                break;
            }
            const double th  = std::atan(r);
            const double th2 = th * th;
            const double thd = th * (1.0 + th2 * (k1 + th2 * (k2 + th2 * (k3 + th2 * k4))));
            const double s   = thd / r;
            xd               = xn * s;
            yd               = yn * s;
            break;
        }
    }
}

/** Bilinear sample of an 8-bit grayscale image. Callers must have checked that
 *  the whole 2x2 neighborhood is inside the image. */
inline double bilinear(const uint8_t* const* rows, double u, double v)
{
    const int    x0 = static_cast<int>(std::floor(u));
    const int    y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;

    const double i00 = rows[y0][x0];
    const double i01 = rows[y0][x0 + 1];
    const double i10 = rows[y0 + 1][x0];
    const double i11 = rows[y0 + 1][x0 + 1];

    return (1 - ay) * ((1 - ax) * i00 + ax * i01) + ay * ((1 - ax) * i10 + ax * i11);
}

}  // namespace

std::optional<mrpt::img::TPixelCoordf> mp2p_icp::projectToPixel(
    const mrpt::img::TCamera& cam, const mrpt::math::TPoint3D& p, double minDepth)
{
    if (p.z < minDepth)
    {
        return {};
    }
    double xd = 0;
    double yd = 0;
    distortNormalized(cam, p.x / p.z, p.y / p.z, xd, yd);

    mrpt::img::TPixelCoordf px;
    px.x = static_cast<float>(cam.fx() * xd + cam.cx());
    px.y = static_cast<float>(cam.fy() * yd + cam.cy());
    return px;
}

void mp2p_icp::VisualPatchTerm::buildPyramid(uint32_t levels)
{
    image_pyramid.clear();
    if (levels == 0)
    {
        return;
    }
    image_pyramid.push_back(image);
    for (uint32_t l = 1; l < levels; l++)
    {
        const auto& prev = image_pyramid.back();
        if (prev.getWidth() < 32 || prev.getHeight() < 32)
        {
            break;
        }
        image_pyramid.push_back(prev.scaleHalf(mrpt::img::IMG_INTERP_LINEAR));
    }
}

VisualPatchAccumStats mp2p_icp::accumulate_visual_patches(
    const VisualPatchTerm& term, const mrpt::poses::CPose3D& pose, Eigen::Matrix<double, 6, 6>& H,
    Eigen::Matrix<double, 6, 1>& g, unsigned int level)
{
    MRPT_START

    VisualPatchAccumStats stats;

    if (term.patches.empty())
    {
        return stats;
    }

    ASSERTMSG_(term.image.getChannelCount() == 1, "VisualPatchTerm::image must be grayscale");
    ASSERTMSG_(term.sigma_intensity > 0, "VisualPatchTerm::sigma_intensity must be >0");

    // The requested level, or the finest one actually available.
    const mrpt::img::CImage& img =
        (level < term.image_pyramid.size()) ? term.image_pyramid[level] : term.image;
    if (level >= term.image_pyramid.size())
    {
        level = 0;
    }
    // Pixel coordinates at this level are the full-resolution ones scaled down;
    // the affine warp is a similarity in pixel units and so is unchanged by it.
    const double levelScale = 1.0 / static_cast<double>(1u << level);

    const int    W    = static_cast<int>(img.getWidth());
    const int    Ht   = static_cast<int>(img.getHeight());
    const int    half = static_cast<int>(term.half_size);
    const size_t nPix = static_cast<size_t>(2 * half + 1) * static_cast<size_t>(2 * half + 1);

    // Row pointers once: the sampling below is the inner loop of this block.
    std::vector<const uint8_t*> rows(static_cast<size_t>(Ht));
    for (int y = 0; y < Ht; y++)
    {
        rows[static_cast<size_t>(y)] = img.ptrLine<uint8_t>(static_cast<unsigned int>(y));
    }
    const uint8_t* const* rowPtr = rows.data();

    // Camera pose in the GLOBAL frame for the current iterate, and the inverse
    // extrinsic, which maps the LOCAL frame into the camera frame.
    const mrpt::poses::CPose3D T_gc  = pose + term.camera_pose_on_local;
    const mrpt::poses::CPose3D E_inv = -term.camera_pose_on_local;
    const Eigen::Matrix3d      R_cl  = E_inv.getRotationMatrix().asEigen();

    const double invSigma  = 1.0 / term.sigma_intensity;
    const double pixWeight = term.weight * invSigma * invSigma;

    const uint64_t residualCall  = residualCallCounter++;
    std::ostream*  residualOut   = residualStream();
    const bool     dumpResiduals = residualOut && (residualCall % kResidualDumpDecimation) == 0;

    // Working buffers, reused across patches.
    std::vector<double> cur(nPix);
    std::vector<double> gx(nPix);
    std::vector<double> gy(nPix);

    for (const auto& patch : term.patches)
    {
        if (level >= patch.ref_patches.size() || patch.ref_patches[level].size() != nPix)
        {
            stats.rejected++;
            continue;
        }
        const std::vector<float>& refPatch = patch.ref_patches[level];

        // 1) The anchor, in the current camera frame, and its projection.
        const auto p_c = T_gc.inverseComposePoint(patch.pt_global);
        const auto px0 = projectToPixel(term.camera, p_c, term.min_depth);
        if (!px0)
        {
            stats.rejected++;
            continue;
        }
        // One extra pixel each way is read by the central-difference gradient.
        const double m = term.border_margin + half + 2;
        // Everything from here on is in the pixel units of the level in use.
        const double u0 = px0->x * levelScale;
        const double v0 = px0->y * levelScale;
        if (u0 < m || v0 < m || u0 > W - 1 - m || v0 > Ht - 1 - m)
        {
            stats.rejected++;
            continue;
        }

        // 2) Affine warp reference -> current, from the patch plane and the two
        //    viewpoints. Two metric tangent vectors are built on the plane,
        //    sized to about `half` pixels in the reference view, and both views
        //    project them; the warp is then the map between the two offsets.
        //    Doing it this way needs only the forward projection, so lens
        //    distortion is handled without ever inverting it.
        const auto            Rr = patch.ref_camera_pose.getRotationMatrix();
        mrpt::math::TVector3D camX(Rr(0, 0), Rr(1, 0), Rr(2, 0));
        mrpt::math::TVector3D camY(Rr(0, 1), Rr(1, 1), Rr(2, 1));
        mrpt::math::TVector3D camZ(Rr(0, 2), Rr(1, 2), Rr(2, 2));

        const auto p_r = patch.ref_camera_pose.inverseComposePoint(patch.pt_global);
        if (p_r.z < term.min_depth)
        {
            stats.rejected++;
            continue;
        }

        // Plane normal: the stored one, or fronto-parallel to the reference.
        mrpt::math::TVector3D n = patch.normal_global;
        if (n.norm() < 1e-6)
        {
            n = camZ;
        }
        else
        {
            n *= 1.0 / n.norm();
        }
        const double nz = n.x * camZ.x + n.y * camZ.y + n.z * camZ.z;
        if (std::abs(nz) < 0.15)
        {
            // The plane is seen nearly edge-on from the reference view: the
            // warp is unbounded, so the patch carries no usable texture.
            stats.rejected++;
            continue;
        }

        const double favg  = 0.5 * (term.camera.fx() + term.camera.fy());
        const double scale = p_r.z * half / favg;

        // Shear the camera axes onto the plane along the viewing direction, so
        // that both tangents lie on it exactly.
        const double nx_ = n.x * camX.x + n.y * camX.y + n.z * camX.z;
        const double ny_ = n.x * camY.x + n.y * camY.y + n.z * camY.z;

        const mrpt::math::TVector3D a1 = (camX - camZ * (nx_ / nz)) * scale;
        const mrpt::math::TVector3D a2 = (camY - camZ * (ny_ / nz)) * scale;

        const mrpt::math::TPoint3D P1 = patch.pt_global + a1;
        const mrpt::math::TPoint3D P2 = patch.pt_global + a2;

        const auto r0 = projectToPixel(term.camera, p_r, term.min_depth);
        const auto r1 = projectToPixel(
            term.camera, patch.ref_camera_pose.inverseComposePoint(P1), term.min_depth);
        const auto r2 = projectToPixel(
            term.camera, patch.ref_camera_pose.inverseComposePoint(P2), term.min_depth);
        const auto c1 = projectToPixel(term.camera, T_gc.inverseComposePoint(P1), term.min_depth);
        const auto c2 = projectToPixel(term.camera, T_gc.inverseComposePoint(P2), term.min_depth);

        if (!r0 || !r1 || !r2 || !c1 || !c2)
        {
            stats.rejected++;
            continue;
        }

        Eigen::Matrix2d B;
        B << r1->x - r0->x, r2->x - r0->x, r1->y - r0->y, r2->y - r0->y;
        Eigen::Matrix2d C;
        C << c1->x - px0->x, c2->x - px0->x, c1->y - px0->y, c2->y - px0->y;

        if (std::abs(B.determinant()) < 1e-6)
        {
            stats.rejected++;
            continue;
        }
        const Eigen::Matrix2d A = C * B.inverse();

        // Reject wild warps: a patch magnified far beyond its captured
        // resolution carries no information the reference actually holds.
        if (!A.allFinite() || A.norm() > 20.0)
        {
            stats.rejected++;
            continue;
        }

        // 3) Sample the current image over the warped patch grid, with the
        //    gradients needed by the Jacobian.
        bool   inside  = true;
        double meanCur = 0;
        double meanGx  = 0;
        double meanGy  = 0;
        size_t k       = 0;
        for (int dy = -half; dy <= half && inside; dy++)
        {
            for (int dx = -half; dx <= half; dx++, k++)
            {
                // The warp is a similarity in pixel units, so it is the same
                // matrix at every level: a level-l offset maps to a level-l
                // offset unchanged.
                const double u = u0 + A(0, 0) * dx + A(0, 1) * dy;
                const double v = v0 + A(1, 0) * dx + A(1, 1) * dy;
                if (u < 2 || v < 2 || u > W - 3 || v > Ht - 3)
                {
                    inside = false;
                    break;
                }
                cur[k] = bilinear(rowPtr, u, v);
                gx[k]  = 0.5 * (bilinear(rowPtr, u + 1, v) - bilinear(rowPtr, u - 1, v));
                gy[k]  = 0.5 * (bilinear(rowPtr, u, v + 1) - bilinear(rowPtr, u, v - 1));
                meanCur += cur[k];
                meanGx += gx[k];
                meanGy += gy[k];
            }
        }
        if (!inside)
        {
            stats.rejected++;
            continue;
        }
        const double invN = 1.0 / static_cast<double>(nPix);
        meanCur *= invN;
        meanGx *= invN;
        meanGy *= invN;

        double meanRef = 0;
        for (const float r : refPatch)
        {
            meanRef += r;
        }
        meanRef *= invN;

        // 4) Mean-normalized residuals, and the per-patch robust weight.
        double sumSqr = 0;
        for (size_t i = 0; i < nPix; i++)
        {
            const double r = (cur[i] - meanCur) - (refPatch[i] - meanRef);
            sumSqr += r * r;
        }
        const double rmsSigmas = std::sqrt(sumSqr * invN) * invSigma;
        if (!std::isfinite(rmsSigmas) || rmsSigmas > term.max_rms_sigmas)
        {
            stats.rejected++;
            continue;
        }
        const double robust =
            rmsSigmas <= term.huber_delta ? 1.0 : term.huber_delta / std::max(1e-12, rmsSigmas);

        // 5) Pose Jacobian of the anchor's projection, shared by every pixel of
        //    the patch since the warp is held fixed within one iteration.
        //
        //    q = T^-1 * P is the anchor in the LOCAL frame; with T <- T*Exp(eps)
        //    it moves as dq/deps = [-I | [q]_x], and p_c = R_cl*q + t_cl.
        const auto      q = pose.inverseComposePoint(patch.pt_global);
        Eigen::Matrix3d qx;
        qx << 0, -q.z, q.y, q.z, 0, -q.x, -q.y, q.x, 0;

        Eigen::Matrix<double, 3, 6> dpc_deps;
        dpc_deps.block<3, 3>(0, 0) = -R_cl;
        dpc_deps.block<3, 3>(0, 3) = R_cl * qx;

        // d(pixel)/d(p_c), by central differences on the projection: the
        // distortion models above are closed-form but their analytic Jacobians
        // are not, and this costs six extra projections per patch.
        Eigen::Matrix<double, 2, 3> dpx_dpc;
        {
            const double eps = 1e-4 * std::max(1.0, std::abs(p_c.z));
            bool         ok  = true;
            for (int axis = 0; axis < 3 && ok; axis++)
            {
                mrpt::math::TPoint3D pp = p_c;
                mrpt::math::TPoint3D pm = p_c;
                pp[static_cast<size_t>(axis)] += eps;
                pm[static_cast<size_t>(axis)] -= eps;

                const auto up = projectToPixel(term.camera, pp, term.min_depth);
                const auto um = projectToPixel(term.camera, pm, term.min_depth);
                if (!up || !um)
                {
                    ok = false;
                    break;
                }
                dpx_dpc(0, axis) = (up->x - um->x) / (2 * eps);
                dpx_dpc(1, axis) = (up->y - um->y) / (2 * eps);
            }
            if (!ok)
            {
                stats.rejected++;
                continue;
            }
        }

        // The projection Jacobian is in FULL-RESOLUTION pixels, while the image
        // gradients below are per pixel OF THE LEVEL IN USE. Without this
        // factor a coarse level takes steps 2^level too short.
        const Eigen::Matrix<double, 2, 6> J_uv = levelScale * (dpx_dpc * dpc_deps);

        // 6) Accumulate. The mean subtraction is part of the residual, so its
        //    own derivative (the mean gradient) belongs in the Jacobian.
        const double w = pixWeight * robust;
        k              = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++, k++)
            {
                const double r = (cur[k] - meanCur) - (refPatch[k] - meanRef);

                Eigen::Matrix<double, 1, 2> gradRow;
                gradRow << gx[k] - meanGx, gy[k] - meanGy;

                const Eigen::Matrix<double, 1, 6> Ji = gradRow * J_uv;

                H.noalias() += w * Ji.transpose() * Ji;
                g.noalias() += w * Ji.transpose() * r;
                stats.chi2 += w * r * r;
            }
        }
        stats.used++;

        if (dumpResiduals)
        {
            for (size_t i = 0; i < nPix; i++)
            {
                *residualOut << ((cur[i] - meanCur) - (refPatch[i] - meanRef))
                             << (i + 1 == nPix ? '\n' : '\t');
            }
        }
    }

    return stats;

    MRPT_END
}
