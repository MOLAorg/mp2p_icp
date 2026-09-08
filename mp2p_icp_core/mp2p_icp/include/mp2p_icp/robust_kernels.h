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
 * @file   robust_kernels.h
 * @brief  Robust kernel types and functions, for common use in all solvers
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2023
 */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>
#include <functional>

namespace mp2p_icp
{
/** Type to select a robust kernel. Support mrpt::typemeta::TEnumType
 *  to get/set from strings.
 */
enum class RobustKernel : uint8_t
{
    /// None: plain least-squares
    None = 0,

    /// Generalized GemanMcClure kernel (Zhang97ivc, Agarwal15phd).
    GemanMcClure,

    /// Cauchy kernel (Lee2013IROS).
    Cauchy,
};

using robust_sqrt_weight_func_t = std::function<double(double /*errSqr*/)>;

/**
 * Creates a functor with the IRLS weight function w(x)=ρ'(x)/x of a given
 * kernel, or an empty functor if non-robust kernel is selected. Callers
 * multiply their per-factor information (not their residual) by it, hence the
 * weight itself and not its square root, despite the historical type name.
 *
 * All kernels here are normalized to w(0)=1, so the kernel does not rescale a
 * block with respect to terms that are not kernel-weighted (e.g. a pose
 * prior), and `kernelParam` is the residual scale at which down-weighting sets
 * in: it is given, and read here, in plain residual units, even though the
 * functor takes the SQUARED residual as its argument.
 *
 * Implemented as `inline` to try to make the compiler to optimize.
 *
 * @param kernel Selected kernel type.
 * @param kernelParam Parameter of the kernel.
 * @return A functor.
 */
inline robust_sqrt_weight_func_t create_robust_kernel(
    const RobustKernel kernel, const double kernelParam)
{
    const double kernelParamSqr = mrpt::square(kernelParam);

    switch (kernel)
    {
        case RobustKernel::None:
            return {};  // empty
            break;

        case RobustKernel::GemanMcClure:
            /**
             * We must return the weight function:
             *
             *   w(x)=( ∂ρ(x)/∂x )/x = ( c²/(c²+x²) )²
             *
             * with the loss function ρ(x) = (c²/2)·x²/(c²+x²),
             * i.e. the Cauchy weight squared, and w(0)=1 as for every other
             * kernel here.
             */
            return [kernelParamSqr](double errorSqr) -> double
            { return mrpt::square(kernelParamSqr / (errorSqr + kernelParamSqr)); };

        case RobustKernel::Cauchy:
            /**
             * We must return the weight function:
             *
             *   w(x)=( ∂ρ(x)/∂x )/x = c²/(x²+c²)
             *
             * with the loss function ρ(x) = 0.5 c² log(1+x²/c²)
             *
             */
            return [kernelParamSqr](double errorSqr) -> double
            { return (kernelParamSqr) / (errorSqr + kernelParamSqr); };

        default:
            throw std::invalid_argument("Unknown kernel type");
    };
};

}  // namespace mp2p_icp

// This allows reading/writing the enum type to strings, e.g. in YAML files.
MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp, mp2p_icp::RobustKernel)
MRPT_FILL_ENUM(RobustKernel::None);
MRPT_FILL_ENUM(RobustKernel::GemanMcClure);
MRPT_FILL_ENUM(RobustKernel::Cauchy);
MRPT_ENUM_TYPE_END()
