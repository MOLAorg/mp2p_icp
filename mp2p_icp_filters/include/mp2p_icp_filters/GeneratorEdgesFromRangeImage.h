/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GeneratorEdgesFromRangeImage.h
 * @brief  Generator of edge points from organized point clouds
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2023
 */

#pragma once

#include <mp2p_icp_filters/Generator.h>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/** Generator of edge points from organized point clouds
 *
 */
class GeneratorEdgesFromRangeImage : public mp2p_icp_filters::Generator
{
    DEFINE_MRPT_OBJECT(GeneratorEdgesFromRangeImage, mp2p_icp_filters)

   public:
    GeneratorEdgesFromRangeImage() = default;

    /** \name Generator API
     *  @{ */

    // See derived class docs
    void initialize(const mrpt::containers::yaml& cfg_block) override;

    struct ParametersEdges
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /// Target layer name for detected plane points. May be empty if not
        /// needed.
        std::string planes_target_layer;

        int32_t score_threshold = 10;
    };

    ParametersEdges paramsEdges_;

    /** @} */

   protected:
    // To be overrided in derived classes, if implemented:
    bool filterRotatingScan(
        const mrpt::obs::CObservationRotatingScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const override;

    bool filterScan3D(
        const mrpt::obs::CObservation3DRangeScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const override;
};

/** @} */

}  // namespace mp2p_icp_filters
