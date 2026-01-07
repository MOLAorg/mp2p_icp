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
 * @file   FilterRenameLayer.h
 * @brief  Renames a layer within a metric map.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 7, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/**
 * @brief Renames a layer within a metric_map_t.
 *
 * This filter takes a layer (which can be of any class) and changes its key
 * in the map to a new name. If a layer with the output name already exists, it is overwritten.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterRenameLayer : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterRenameLayer, mp2p_icp_filters)
   public:
    FilterRenameLayer() = default;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** The current name of the layer to be renamed. */
        std::string input_layer;

        /** The new name for the layer. */
        std::string output_layer;

        /** Can be set false to silently ignore missing input layers */
        bool fail_if_input_layer_does_not_exist = true;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters