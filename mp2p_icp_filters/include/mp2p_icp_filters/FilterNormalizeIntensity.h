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

#include <mutex>
#include <optional>

namespace mp2p_icp_filters
{
/** Normalizes the intensity channel of a point cloud layer, such as intensity
 * values end up in the range [0,1].
 *
 * No new layer is created, data is directly updated in the same input/output
 * layer.
 *
 * If the parameter `remember_intensity_range` is `true` (default=`false`),
 * the filter object will have "memory" and remember the minimum and maximum
 * intensities observed in past clouds. Otherwise, each cloud will have its
 * own extreme values so they will be always normalized to [0, 1].
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

        bool remember_intensity_range = false;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    mutable std::optional<float> minI_, maxI_;
    mutable std::mutex           minMaxMtx_;
};

/** @} */

}  // namespace mp2p_icp_filters
