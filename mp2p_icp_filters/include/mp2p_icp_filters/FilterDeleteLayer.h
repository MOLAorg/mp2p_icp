/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDeleteLayer.h
 * @brief  Removes a given point layer
 * @author Jose Luis Blanco Claraco
 * @date   Sep 14, 2021
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Removes a given point layer.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDeleteLayer : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDeleteLayer, mp2p_icp_filters)
   public:
    FilterDeleteLayer();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string pointcloud_layer_to_remove;

    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
