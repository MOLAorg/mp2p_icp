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
 * @file   Generator_view_vector_sse2.cpp
 * @brief  SSE2/SSE kernel for view-vector field generation.
 * @note   This file is compiled with -msse2 (GCC/Clang) or no extra flag
 *         (MSVC, where SSE2 is on by default for x86_64).
 */

#include "Generator_view_vector_impl.h"

#if defined(MP2P_HAS_SSE2)

#include <emmintrin.h>  // SSE2
#include <xmmintrin.h>  // SSE (rsqrtps)

#include <cmath>

namespace mp2p_icp_filters::internal
{
void addViewVectors_sse2(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz)
{
    const __m128 sx4   = _mm_set1_ps(sx);
    const __m128 sy4   = _mm_set1_ps(sy);
    const __m128 sz4   = _mm_set1_ps(sz);
    const __m128 half4 = _mm_set1_ps(0.5f);
    const __m128 c1p54 = _mm_set1_ps(1.5f);
    // eps² guards against zero-length view vectors
    const __m128 eps24 = _mm_set1_ps(1e-20f);

    size_t i = 0;
    for (; i + 4 <= N; i += 4)
    {
        const __m128 dx = _mm_sub_ps(sx4, _mm_loadu_ps(px + i));
        const __m128 dy = _mm_sub_ps(sy4, _mm_loadu_ps(py + i));
        const __m128 dz = _mm_sub_ps(sz4, _mm_loadu_ps(pz + i));

        __m128 norm_sq =
            _mm_add_ps(_mm_add_ps(_mm_mul_ps(dx, dx), _mm_mul_ps(dy, dy)), _mm_mul_ps(dz, dz));

        norm_sq = _mm_max_ps(norm_sq, eps24);

        // Approximate reciprocal sqrt + 1 Newton-Raphson step:
        //   y  = rsqrt_approx(x)
        //   y' = y · (1.5 - 0.5 · x · y²)
        const __m128 y0       = _mm_rsqrt_ps(norm_sq);
        const __m128 inv_norm = _mm_mul_ps(
            y0, _mm_sub_ps(c1p54, _mm_mul_ps(half4, _mm_mul_ps(norm_sq, _mm_mul_ps(y0, y0)))));

        _mm_storeu_ps(vx + i, _mm_mul_ps(dx, inv_norm));
        _mm_storeu_ps(vy + i, _mm_mul_ps(dy, inv_norm));
        _mm_storeu_ps(vz + i, _mm_mul_ps(dz, inv_norm));
    }

    // Scalar tail
    for (; i < N; i++)
    {
        const float dx      = sx - px[i];
        const float dy      = sy - py[i];
        const float dz      = sz - pz[i];
        const float nsq     = dx * dx + dy * dy + dz * dz;
        const float inv_nrm = (nsq > 1e-20f) ? (1.0f / std::sqrt(nsq)) : 0.0f;
        vx[i]               = dx * inv_nrm;
        vy[i]               = dy * inv_nrm;
        vz[i]               = dz * inv_nrm;
    }
}

}  // namespace mp2p_icp_filters::internal

#endif  // MP2P_HAS_SSE2
