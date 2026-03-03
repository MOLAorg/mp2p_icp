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
 * @file   Generator_view_vector_impl.h
 * @brief  Internal declarations of per-SIMD-level view-vector kernels.
 * @author Jose Luis Blanco Claraco
 * @date   Mar 2026
 *
 * This header is internal to mp2p_icp_filters and should NOT be installed.
 */
#pragma once

#include <cstddef>  // size_t

namespace mp2p_icp_filters::internal
{
/**
 * Fills view_x/y/z[0..N-1] with unit vectors from each point p[i] towards
 * the sensor origin (sx, sy, sz). All pointers must be valid and non-null.
 *
 * The scalar version is always compiled into Generator.cpp.
 * The SSE2/AVX versions are compiled into their own translation units with the
 * appropriate -msse2 / -mavx compiler flags.
 */
void addViewVectors_scalar(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz);

#if defined(MP2P_HAS_SSE2)
void addViewVectors_sse2(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz);
#endif

#if defined(MP2P_HAS_AVX)
void addViewVectors_avx(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz);
#endif

}  // namespace mp2p_icp_filters::internal
