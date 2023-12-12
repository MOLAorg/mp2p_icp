/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/Parameterizable.h>

#include <stdexcept>

using namespace mp2p_icp;

void ParameterSource::attach(Parameterizable& obj)
{
    for (auto& p : obj.declaredParameters())  //
        attachedDeclParameters_.insert(&p);
}

void ParameterSource::realize()
{
    // Here comes the beef:
    // 1) Compile uncompiled expressions,
    // 2) Eval them all and store the results in their target.

    // 1) compile?
    for (auto& p : attachedDeclParameters_)
    {
        if (p->is_constant) continue;
        if (p->compiled.has_value()) continue;  // already done:

        auto& expr = p->compiled.emplace();
        expr.compile(p->expression, variables_);
    }

    // 2) Evaluate and store:
    for (auto& p : attachedDeclParameters_)
    {
        if (p->is_constant) continue;

        const double val = p->compiled->eval();

        std::visit(
            [val](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<T, std::monostate>)
                {
                    throw std::runtime_error(
                        "[ParameterSource] Attached parameter target is "
                        "monostate!");
                }
                else if constexpr (std::is_same_v<T, double*>)
                {
                    *arg = val;
                }
                else if constexpr (std::is_same_v<T, float*>)
                {
                    *arg = static_cast<float>(val);
                }
                else if constexpr (std::is_same_v<T, uint32_t*>)
                {
                    *arg = static_cast<uint32_t>(val);
                }
            },
            p->target);
    }
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, double& target)
{
    internal::InfoPerParam& ipm = declParameters_.emplace_back();

    ipm.expression = value;
    ipm.target     = &target;

    // Try to evaluate the expression now:
    // If it does not contain unknown variables, it will remain
    // constant and will need not updates from "realize()":
    try
    {
        mrpt::expr::CRuntimeCompiledExpression e;
        e.compile(value);
        // Yes, it was successful: mark as constant and store value:
        ipm.is_constant = true;
        using T         = std::decay_t<decltype(target)>;
        // Store result:
        target       = static_cast<T>(e.eval());
        ipm.compiled = std::move(e);
    }
    catch (const std::exception&)
    {
        // We probably need variables, not defined yet.
        // Ignore this exception at this moment. Will trigger if
        // raised again in realize() after defining the variables.
    }
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, float& target)
{
    internal::InfoPerParam& ipm = declParameters_.emplace_back();

    ipm.expression = value;
    ipm.target     = &target;
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, uint32_t& target)
{
    internal::InfoPerParam& ipm = declParameters_.emplace_back();

    ipm.expression = value;
    ipm.target     = &target;
}
