/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/expr/CRuntimeCompiledExpression.h>

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
    bool                                                     is_constant        = false;
    bool                                                     has_been_evaluated = false;
};
}  // namespace internal

class ParameterSource;
class Parameterizable;

/** Users of derived classes must declare an instance of this type, then attach
 * instances of derived classes to it, then optionally update variables via
 * updateVariable(), then call realize() for the changes to take effect.
 *
 * \ingroup mp2p_icp_map_grp
 */
class ParameterSource
{
   public:
    ParameterSource() = default;

    void attach(Parameterizable& obj);

    /** Updates a variable value. Remember to call realize() after updating all
     *  variables for the changes to take effect */
    void updateVariable(const std::string& variable, double value) { variables_[variable] = value; }

    /** Like updateVariable(), accepting several pairs of names-values */
    void updateVariables(const std::vector<std::pair<std::string, double>>& nameValuePairs)
    {
        for (const auto& [name, value] : nameValuePairs)
        {
            updateVariable(name, value);
        }
    }

    void realize();

    std::string printVariableValues() const;

    /** Returns a copy of the current variable values */
    auto getVariableValues() const -> std::map<std::string, double> { return variables_; }

   private:
    // Attached clients.
    std::map<std::string, double>     variables_;
    std::set<internal::InfoPerParam*> attachedDeclParameters_;
};

/** Common base for classes allowing dynamic parameters as given by formulas,
 * possibly as functions of externally-provided variables.
 *
 * \ingroup mp2p_icp_map_grp
 */
class Parameterizable
{
    friend class ParameterSource;

   public:
    /**
     * Each parameterizable object can be attached to one source at a given time
     */
    virtual void attachToParameterSource(ParameterSource& source) { source.attach(*this); }

    auto&       declaredParameters() { return declParameters_; }
    const auto& declaredParameters() const { return declParameters_; }

    ParameterSource*       attachedSource() { return attachedSource_; }
    const ParameterSource* attachedSource() const { return attachedSource_; }

    /** Throws if any parameter is uninitialized or realized() has not been
     * called in the attached ParameterSource.
     * All parameters can be reset so realize() needs to be called again
     * by manually calling unrealizeParameters().
     */
    void checkAllParametersAreRealized() const;

    /// Mark all non-constant parameters as non-evaluated again.
    void unrealizeParameters();

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
    ParameterSource*                    attachedSource_ = nullptr;

    template <typename T>
    void parseAndDeclareParameter_impl(const std::string& value, T& target);
};

/** Attach a vector of objects to the given source */
template <typename T>
inline void AttachToParameterSource(
    std::vector<std::shared_ptr<T>>& setObjects, ParameterSource& source)
{
    for (auto& objPtr : setObjects)
    {
        auto o = std::dynamic_pointer_cast<Parameterizable>(objPtr);
        if (!o)
        {
            continue;
        }
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

#define DECLARE_PARAMETER_IN_OPT(__yaml, __variable, __object)    \
    __object.mp2p_icp::Parameterizable::parseAndDeclareParameter( \
        (__yaml).getOrDefault(#__variable, std::to_string(__variable)), __variable);

#define DECLARE_PARAMETER_OPT(__yaml, __variable) \
    DECLARE_PARAMETER_IN_OPT(__yaml, __variable, (*this))

#define DECLARE_PARAMETER_IN_REQ(__yaml, __variable, __object)                               \
    if (!(__yaml).has(#__variable))                                                          \
        throw std::invalid_argument(                                                         \
            mrpt::format(                                                                    \
                "Required parameter `%s` not an existing key in dictionary.", #__variable)); \
    (__object).mp2p_icp::Parameterizable::parseAndDeclareParameter(                          \
        (__yaml)[#__variable].as<std::string>(), __variable);

#define DECLARE_PARAMETER_REQ(__yaml, __variable) \
    DECLARE_PARAMETER_IN_REQ(__yaml, __variable, (*this))

}  // namespace mp2p_icp
