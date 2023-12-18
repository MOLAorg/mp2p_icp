/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GeneratorEdgesFromCurvature.h
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
class GeneratorEdgesFromCurvature : public mp2p_icp_filters::Generator
{
    DEFINE_MRPT_OBJECT(GeneratorEdgesFromCurvature, mp2p_icp_filters)

   public:
    GeneratorEdgesFromCurvature() = default;

    /** \name Generator API
     *  @{ */

    // See derived class docs
    void initialize(const mrpt::containers::yaml& cfg_block) override;

    struct ParametersEdges
    {
        void load_from_yaml(
            const mrpt::containers::yaml& c,
            GeneratorEdgesFromCurvature&  parent);

        float max_cosine          = 0.5f;
        float min_point_clearance = 0.10f;
    };

    ParametersEdges paramsEdges_;

    void process(
        const mrpt::obs::CObservation& input_raw, mp2p_icp::metric_map_t& inOut,
        const std::optional<mrpt::poses::CPose3D>& robotPose =
            std::nullopt) const override;

    /** @} */

   protected:
    // To be overrided in derived classes, if implemented:
    bool filterRotatingScan(
        const mrpt::obs::CObservationRotatingScan& pc,
        mp2p_icp::metric_map_t&                    out,
        const std::optional<mrpt::poses::CPose3D>& robotPose =
            std::nullopt) const override;
};

/** @} */

}  // namespace mp2p_icp_filters
