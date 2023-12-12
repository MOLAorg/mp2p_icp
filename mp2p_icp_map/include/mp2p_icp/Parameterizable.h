/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/expr/CRuntimeCompiledExpression.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <variant>
#include <vector>

namespace mp2p_icp
{
namespace internal
{
struct InfoPerParam
{
    std::string expression;
    /// Compiled expression
    std::optional<mrpt::expr::CRuntimeCompiledExpression>    compiled;
    std::variant<std::monostate, double*, float*, uint32_t*> target;
};
}  // namespace internal

class ParameterSource;
class Parameterizable;

/** Users of derived classes must declare an instance of this type, then attach
 * instances of derived classes to it, then optionally update variables via
 * updateVariable(), then call realize() for the changes to take effect.
 *
 * \ingroup mp2p_icp_grp
 */
class ParameterSource
{
   public:
    ParameterSource() = default;

    void attach(Parameterizable& obj);

    void updateVariable(const std::string& variable, double value)
    {
        variables_[variable] = value;
    }

    void realize();

   private:
    // Attached clients.
    std::map<std::string, double>     variables_;
    std::set<internal::InfoPerParam*> attachedDeclParameters_;
};

/** Common base for classes allowing dynamic parameters as given by formulas,
 * possibly as functions of externally-provided variables.
 *
 * \ingroup mp2p_icp_grp
 */
class Parameterizable
{
   public:
    void attachToParameterSource(ParameterSource& source)
    {
        source.attach(*this);
    }

    auto&       declaredParameters() { return declParameters_; }
    const auto& declaredParameters() const { return declParameters_; }

   protected:
    /** To be called at initialization by derived classes that read a parameter
     *  from a YAML or any other text source.
     *  If the text does not contain any undefined variable, it will be
     * evaluated straight away into `target`. Otherwise, its value will be
     * left to the moment this object is attached to a ParameterSource and
     * the user calls ParameterSource::realize().
     */
    void parseAndDeclareParameter(const std::string& value, double& target);

    /// \overload
    void parseAndDeclareParameter(const std::string& value, float& target);
    /// \overload
    void parseAndDeclareParameter(const std::string& value, uint32_t& target);

   private:
    /// List of declared parameters:
    std::vector<internal::InfoPerParam> declParameters_;
};

/** Attach a vector of objects to the given source */
template <typename T>
inline void AttachToParameterSource(
    std::vector<std::shared_ptr<T>>& setObjects, ParameterSource& source)
{
    for (auto& objPtr : setObjects)
    {
        auto o = std::dynamic_pointer_cast<Parameterizable>(objPtr);
        if (!o) continue;
        o->attachToParameterSource(source);
    }
}

/** Attach a single object to the given source.
 *  Provided for syntax consistency between single and vectors of objects.
 */
inline void AttachToParameterSource(Parameterizable& o, ParameterSource& source)
{
    o.attachToParameterSource(source);
}

#define DECLARE_PARAMETER_IN_OPT(__yaml, __variable, __object)        \
    __object.mp2p_icp::Parameterizable::parseAndDeclareParameter(     \
        __yaml.getOrDefault(#__variable, std::to_string(__variable)), \
        __variable);

#define DECLARE_PARAMETER_OPT(__yaml, __variable) \
    DECLARE_PARAMETER_IN_OPT(__yaml, __variable, (*this))

#define DECLARE_PARAMETER_IN_REQ(__yaml, __variable, __object)            \
    if (!__yaml.has(#__variable))                                         \
        throw std::invalid_argument(mrpt::format(                         \
            "Required parameter `%s` not an existing key in dictionary.", \
            #__variable));                                                \
    __object.mp2p_icp::Parameterizable::parseAndDeclareParameter(         \
        __yaml[#__variable].as<std::string>(), __variable);

#define DECLARE_PARAMETER_REQ(__yaml, __variable) \
    DECLARE_PARAMETER_IN_REQ(__yaml, __variable, (*this))

}  // namespace mp2p_icp
