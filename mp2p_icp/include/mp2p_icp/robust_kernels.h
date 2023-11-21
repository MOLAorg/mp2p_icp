/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
 * Creates a functor with the sqrt of the weight function of a given
 * kernel, or an empty functor if non-robust kernel is selected.
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
             * We must return the sqrt of the weight function:
             *
             *   sqrt(w(x))=( ∂ρ(x)/∂x )/x = c²/(e²+c)²
             *
             * with the loss function ρ(x) = (x²/2)/(c²+x²)
             */
            return [kernelParamSqr, kernelParam](double errorSqr) -> double {
                return (kernelParamSqr) / mrpt::square(errorSqr + kernelParam);
            };

        case RobustKernel::Cauchy:
            /**
             * We must return the sqrt of the weight function:
             *
             *   sqrt(w(x))=( ∂ρ(x)/∂x )/x = c²/(e²+c²)
             *
             * with the loss function ρ(x) = 0.5 c² log(1+x²/c²)
             *
             */
            return [kernelParamSqr, kernelParam](double errorSqr) -> double {
                return (kernelParamSqr) / (errorSqr + kernelParamSqr);
            };

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
