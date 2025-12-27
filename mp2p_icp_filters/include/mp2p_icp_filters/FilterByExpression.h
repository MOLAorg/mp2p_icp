/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   FilterByExpression.h
 * @brief  Generic filter by expressions depending on cloud fields
 * @author Jose Luis Blanco Claraco
 * @date   Dec 26, 2025
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/**
 * A programmable filter that evaluates a mathematical or logical expression
 * for each point to decide into which output layer it should be moved.
 *
 * Parameters:
 * - expression: A string formula (e.g., "ring > 10", "x^2 + y^2 < 4.0")
 * - output_layer_passed: (Optional) Layer for points where expression > 0
 * - output_layer_not_passed: (Optional) Layer for points where expression <= 0
 *
 * Variables available in expressions:
 * - x, y, z: Spatial coordinates
 * - intensity, ring,... or any other custom field in the input clouds.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterByExpression : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterByExpression, mp2p_icp_filters)

   public:
    FilterByExpression();

    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterByExpression& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;
        std::string expression;
        std::string output_layer_passed;
        std::string output_layer_not_passed;
    };
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

   private:
    // Re-compiles the expression if the set of fields has changed
    void update_expression_bindings(const mrpt::maps::CPointsMap& pc) const;

    mutable mrpt::expr::CRuntimeCompiledExpression expr_;

    // This map holds the double values that ExprTk "points" to.
    // Use a map so addresses remain stable after insertion.
    mutable std::map<std::string, double> var_values_;

    // Cached set of field names to detect schema changes
    mutable std::set<std::string> cached_field_names_;
};
}  // namespace mp2p_icp_filters