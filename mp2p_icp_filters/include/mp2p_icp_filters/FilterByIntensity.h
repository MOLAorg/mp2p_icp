/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByIntensity.h
 * @brief  Thresholds an input cloud by intensity values.
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Thresholds an input cloud by intensity values.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterByIntensity : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterByIntensity, mp2p_icp_filters)
   public:
    FilterByIntensity();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer;

        /** If non-empty, points with `intensity < low_threshold` will be stored
         * here. */
        std::string output_layer_low_intensity;

        /** If non-empty, points with `intensity > high_threshold` will be
         * stored here. */
        std::string output_layer_high_intensity;

        /** If non-empty, points with
         * `intensity ∈ [low_threshold, high_threshold]` will be stored here. */
        std::string output_layer_mid_intensity;

        float low_threshold  = 0.10f;
        float high_threshold = 0.90f;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
