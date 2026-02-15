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
 * @file   FilterClear.h
 * @brief  Clears (empties) a given metric map layer
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/maps/CMetricMap.h>

namespace mp2p_icp_filters
{
/** Clears (empties) a given metric map layer by calling its clear() method.
 *
 * This filter calls the virtual clear() method of any mrpt::maps::CMetricMap,
 * which removes all points/features from the map while keeping the layer itself
 * in the metric_map_t structure.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterClear : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterClear, mp2p_icp_filters)
   public:
    FilterClear();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** The layer to clear */
        std::string target_layer;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
