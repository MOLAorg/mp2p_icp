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
 * @file   FilterByExpression.h
 * @brief  Generic filter by expressions depending on cloud fields
 * @author Jose Luis Blanco Claraco
 * @date   Dec 26, 2025
 */
#include <mp2p_icp_filters/FilterByExpression.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(FilterByExpression, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByExpression::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, [[maybe_unused]] FilterByExpression& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, expression);
    MCP_LOAD_OPT(c, output_layer_not_passed);
    MCP_LOAD_OPT(c, output_layer_passed);

    ASSERTMSG_(
        !output_layer_not_passed.empty() || !output_layer_passed.empty(),
        "At least one 'output_layer_not_passed' or "
        "'output_layer_passed' must be provided.");
}

FilterByExpression::FilterByExpression()
{
    mrpt::system::COutputLogger::setLoggerName("FilterByExpression");
}

void FilterByExpression::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c, *this);
    // We don't compile here because we don't know the fields until we see the cloud
}

void FilterByExpression::update_expression_bindings(const mrpt::maps::CPointsMap& pc) const
{
#if MRPT_VERSION >= 0x020f04  // 2.15.4
    // Create a set of current field names for comparison
    std::set<std::string> current_fields;
    for (const auto& l :
         {pc.getPointFieldNames_double(), pc.getPointFieldNames_float(),
          pc.getPointFieldNames_uint16(), pc.getPointFieldNames_uint8()})
    {
        for (const auto& f : l)
        {
            current_fields.insert(std::string(f));
        }
    }

    // If schema matches cache and it's already compiled, skip
    if (current_fields == cached_field_names_ && expr_.is_compiled())
    {
        return;
    }

    // Reset values and rebuild map
    var_values_.clear();
    for (const auto& fieldName : current_fields)
    {
        var_values_[fieldName] = 0.0;
    }

    // Compile
    expr_.compile(params.expression, var_values_, "FilterByExpression (" + FilterBase::name + ")");
    cached_field_names_ = std::move(current_fields);
#else
    (void)pc;
    THROW_EXCEPTION("This class requires MRPT>=2.15.4");
#endif
}

void FilterByExpression::filter(mp2p_icp::metric_map_t& inOut) const
{
#if MRPT_VERSION >= 0x020f04  // 2.15.4

    checkAllParametersAreRealized();

    // In:
    const auto pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    update_expression_bindings(pc);

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPassed = GetOrCreatePointLayer(
        inOut, params.output_layer_passed, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outPassed)
    {
        outPassed->reserve(outPassed->size() + pc.size() / 2);
    }

    mrpt::maps::CPointsMap::Ptr outFailed = GetOrCreatePointLayer(
        inOut, params.output_layer_not_passed, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);
    if (outFailed)
    {
        outFailed->reserve(outFailed->size() + pc.size() / 2);
    }

    // Prepare Output Contexts
    std::optional<mrpt::maps::CPointsMap::InsertCtx> ctxP, ctxF;
    if (outPassed)
    {
        outPassed->registerPointFieldsFrom(pc);
        ctxP = outPassed->prepareForInsertPointsFrom(pc);
    }
    if (outFailed)
    {
        outFailed->registerPointFieldsFrom(pc);
        ctxF = outFailed->prepareForInsertPointsFrom(pc);
    }

    // --- High Efficiency Pointer Extraction ---
    const size_t N = pc.size();

    // Map each field name to its corresponding typed buffer pointer to avoid lookups in the loop
    struct FieldInterface
    {
        double* var_target = nullptr;
        // We use a lambda to fetch the value and convert to double
        std::function<double(size_t)> fetcher;
    };
    std::vector<FieldInterface> active_fields;

    // Assign the correct fetcher based on the field type defined in CPointsMap.h
    for (const auto& field : pc.getPointFieldNames_double())
    {
        std::string fName(field);
        if (var_values_.count(fName) == 0)
        {
            continue;
        }
        FieldInterface& fi = active_fields.emplace_back();
        fi.var_target      = &var_values_[fName];
        const auto& vec    = pc.getPointsBufferRef_double_field(fName);
        fi.fetcher         = [vec](size_t i) { return (*vec)[i]; };
    }
    for (const auto& field : pc.getPointFieldNames_float())
    {
        std::string fName(field);
        if (var_values_.count(fName) == 0)
        {
            continue;
        }
        FieldInterface& fi = active_fields.emplace_back();
        fi.var_target      = &var_values_[fName];
        const auto& vec    = pc.getPointsBufferRef_float_field(fName);
        fi.fetcher         = [vec](size_t i) { return static_cast<double>((*vec)[i]); };
    }
    for (const auto& field : pc.getPointFieldNames_uint16())
    {
        std::string fName(field);
        if (var_values_.count(fName) == 0)
        {
            continue;
        }
        FieldInterface& fi = active_fields.emplace_back();
        fi.var_target      = &var_values_[fName];
        const auto& vec    = pc.getPointsBufferRef_uint16_field(fName);
        fi.fetcher         = [vec](size_t i) { return static_cast<double>((*vec)[i]); };
    }
    for (const auto& field : pc.getPointFieldNames_uint8())
    {
        std::string fName(field);
        if (var_values_.count(fName) == 0)
        {
            continue;
        }
        FieldInterface& fi = active_fields.emplace_back();
        fi.var_target      = &var_values_[fName];
        const auto& vec    = pc.getPointsBufferRef_uint8_field(fName);
        fi.fetcher         = [vec](size_t i) { return static_cast<double>((*vec)[i]); };
    }

    // --- Main Processing Loop ---
    size_t passCount    = 0;
    size_t notPassCount = 0;
    for (size_t i = 0; i < N; ++i)
    {
        // Update all discovered fields
        for (auto& field : active_fields)
        {
            *field.var_target = field.fetcher(i);
        }

        if (expr_.eval() > 0.0)
        {
            passCount++;
            if (outPassed)
            {
                outPassed->insertPointFrom(i, *ctxP);
            }
        }
        else
        {
            notPassCount++;
            if (outFailed)
            {
                outFailed->insertPointFrom(i, *ctxF);
            }
        }
    }
    MRPT_LOG_DEBUG_FMT(
        "FilterByExpression: %zu points passed filter, %zu points did not pass, for expresion: %s",
        passCount, notPassCount, params.expression.c_str());
#else
    (void)inOut;
    THROW_EXCEPTION("This class requires MRPT>=2.15.4");
#endif
}