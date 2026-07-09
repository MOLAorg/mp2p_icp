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
 * @file   FilterRemovePointCloudField.h
 * @brief  Unregisters (removes) a custom point cloud field
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Unregisters (removes) a custom point cloud field from a CGenericPointsMap layer.
 *
 * This filter removes a previously registered custom field of any supported type
 * (float, double, uint16_t, uint8_t) from a point cloud layer. The field is
 * completely removed, freeing the associated memory.
 *
 * Note: This filter only works with layers containing mrpt::maps::CGenericPointsMap
 * or derived classes. Other map types will be silently skipped.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterRemovePointCloudField : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterRemovePointCloudField, mp2p_icp_filters)
   public:
    FilterRemovePointCloudField();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** The point cloud layer to process */
        std::string pointcloud_layer = "raw";

        /** One or more field names to remove (e.g., "intensity", "ring", "timestamp_abs") */
        std::vector<std::string> field_names;

        /** Whether to throw an exception if any field does not exist.
         * If false, missing fields are silently ignored. */
        bool throw_on_missing_field = true;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
