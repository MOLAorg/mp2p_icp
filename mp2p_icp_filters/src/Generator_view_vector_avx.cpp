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
 * @file   Generator_view_vector_avx.cpp
 * @brief  AVX kernel for view-vector field generation.
 * @note   This file is compiled with -mavx (GCC/Clang) or /arch:AVX (MSVC).
 */

#include "Generator_view_vector_impl.h"

#if defined(MP2P_HAS_AVX)

#include <immintrin.h>  // AVX

#include <cmath>

namespace mp2p_icp_filters::internal
{
void addViewVectors_avx(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz)
{
    const __m256 sx8   = _mm256_set1_ps(sx);
    const __m256 sy8   = _mm256_set1_ps(sy);
    const __m256 sz8   = _mm256_set1_ps(sz);
    const __m256 half8 = _mm256_set1_ps(0.5f);
    const __m256 c1p58 = _mm256_set1_ps(1.5f);
    // eps² guards against zero-length view vectors
    const __m256 eps28 = _mm256_set1_ps(1e-20f);

    size_t i = 0;
    for (; i + 8 <= N; i += 8)
    {
        const __m256 dx = _mm256_sub_ps(sx8, _mm256_loadu_ps(px + i));
        const __m256 dy = _mm256_sub_ps(sy8, _mm256_loadu_ps(py + i));
        const __m256 dz = _mm256_sub_ps(sz8, _mm256_loadu_ps(pz + i));

        // norm_sq = dx²+dy²+dz²
        __m256 norm_sq = _mm256_add_ps(
            _mm256_add_ps(_mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy)), _mm256_mul_ps(dz, dz));

        norm_sq = _mm256_max_ps(norm_sq, eps28);

        // Approximate reciprocal sqrt + 1 Newton-Raphson step:
        //   y  = rsqrt_approx(x)
        //   y' = y · (1.5 - 0.5 · x · y²)
        const __m256 y0       = _mm256_rsqrt_ps(norm_sq);
        const __m256 inv_norm = _mm256_mul_ps(
            y0, _mm256_sub_ps(
                    c1p58, _mm256_mul_ps(half8, _mm256_mul_ps(norm_sq, _mm256_mul_ps(y0, y0)))));

        _mm256_storeu_ps(vx + i, _mm256_mul_ps(dx, inv_norm));
        _mm256_storeu_ps(vy + i, _mm256_mul_ps(dy, inv_norm));
        _mm256_storeu_ps(vz + i, _mm256_mul_ps(dz, inv_norm));
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

#endif  // MP2P_HAS_AVX
