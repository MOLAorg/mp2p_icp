/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterAdjustTimestamps.h
 * @brief  Normalizes point cloud timestamps
 * @author Jose Luis Blanco Claraco
 * @date   Jun 19, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mp2p_icp_filters
{
/** Enum to select the pointcloud timestamp normalization criterion in
 *  FilterAdjustTimestamps
 *
 * \ingroup mp2p_icp_filters_grp
 */
enum class TimestampAdjustMethod : uint8_t
{
    /** Adjust such as the earliest timestamp is 0, wih succesive ones
       representing real ellapsed seconds. */
    EarliestIsZero = 0,
    /** Adjust such as the middle timestamp is 0, with the rest being positive
       and negative ellapsed seconds.  */
    MiddleIsZero,
    /** Normalize all timestamps such as they are in the range [0,1] */
    Normalize,
};

/** Modifies the per-point timestamps of a map layer according to one of a set
 * of criteria (see TimestampAdjustMethod).
 *
 * An optional time offset to be added on top of that adjustment is available
 * via Parameters::time_offset
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterAdjustTimestamps : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterAdjustTimestamps, mp2p_icp_filters)
   public:
    FilterAdjustTimestamps();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterAdjustTimestamps& parent);

        std::string pointcloud_layer;

        /** Whether to skip throwing an exception if the input layer does not
         * contain timestamps.
         */
        bool silently_ignore_no_timestamps = false;

        /** Additional time offset, useful when synchronizing several sensors */
        double time_offset = .0;

        /** The criterion to adjust timestamps. */
        TimestampAdjustMethod method = TimestampAdjustMethod::MiddleIsZero;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */
}  // namespace mp2p_icp_filters

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mp2p_icp_filters, mp2p_icp_filters::TimestampAdjustMethod)
MRPT_FILL_ENUM(TimestampAdjustMethod::EarliestIsZero);
MRPT_FILL_ENUM(TimestampAdjustMethod::MiddleIsZero);
MRPT_FILL_ENUM(TimestampAdjustMethod::Normalize);
MRPT_ENUM_TYPE_END()
