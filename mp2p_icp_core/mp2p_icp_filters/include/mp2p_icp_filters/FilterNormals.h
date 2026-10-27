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
#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/**
 * @brief Estimates per-point surface normals, in place, as the smallest
 * eigenvector of the k nearest neighbors of each point.
 *
 * Adds the "normal_x", "normal_y", "normal_z" fields to the input layer, which
 * must support extra fields (e.g. mrpt::maps::CGenericPointsMap). If the layer
 * has the "view_x/y/z" fields (see Generator), normals are flipped to point
 * towards the sensor, i.e. outside of solid bodies. Points with too few
 * neighbors get a zero normal.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterNormals : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterNormals, mp2p_icp_filters)
   public:
    FilterNormals() = default;

    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterNormals& parent);

        /** The layer to which normals are added */
        std::string pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Number of neighbors (including the point itself) for the fit. */
        uint32_t knn = 10;

        /** Neighbors farther than this [m] are ignored. */
        float max_search_distance = 1.0f;

        /** When TBB is enabled, the grainsize for parallel processing. */
        size_t parallelization_grain_size = 1024UL;
    };

    Parameters params;

   protected:
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters
