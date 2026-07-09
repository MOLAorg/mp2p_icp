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
 * @file   FilterAbsoluteTimestamp.h
 * @brief  Creates a new double-precision field with absolute timestamps.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 6, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/**
 * @brief Creates a new per-point channel (default name: "timestamp_abs") of
 * type double.
 *
 * This filter reads the relative timestamp channel "t" (float) and adds the
 * reference "zero time" of the cloud (retrieved from the attached processing
 * context's velocity buffer) to compute an absolute UNIX timestamp for every
 * point.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterAbsoluteTimestamp : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterAbsoluteTimestamp, mp2p_icp_filters)
   public:
    FilterAbsoluteTimestamp();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** The layer to process. */
        std::string pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Name of the new output field. */
        std::string output_field_name = "timestamp_abs";
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters