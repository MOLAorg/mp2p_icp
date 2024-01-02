/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterNormalizeIntensity.h
 * @brief  Normalizes the intensity channel of a point cloud layer
 * @author Jose Luis Blanco Claraco
 * @date   Dec 19, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Normalizes the intensity channel of a point cloud layer, such as intensity
 * values end up in the range [0,1].
 *
 * No new layer is created, data is directly updated in the same input/output
 * layer.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterNormalizeIntensity : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterNormalizeIntensity, mp2p_icp_filters)
   public:
    FilterNormalizeIntensity();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string pointcloud_layer;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
