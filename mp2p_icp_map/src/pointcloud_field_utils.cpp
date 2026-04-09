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
 * @file   pointcloud_field_utils.cpp
 * @brief  Raises warnings on missing fields that would be filled with zeros.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 9, 2026
 */

#include <mp2p_icp/pointcloud_field_utils.h>

bool mp2p_icp::warn_on_field_padding_mismatch(
    const mrpt::maps::CPointsMap& src, const mrpt::maps::CPointsMap& dst,
    const mrpt::system::COutputLogger& logger)
{
    bool any = false;

    // Helper: check one family of fields.
    // dst_names: fields registered in dst for this type
    // src_has_fn: returns true if src carries a non-empty buffer for the name
    auto check = [&](const auto& dst_names_org, auto src_has_fn, const char* type_label)
    {
        // make this compatible with different MRPT versions using std::string_view vs std::string
        std::vector<std::string> dst_names;
        for (const auto& mame : dst_names_org)
        {
            dst_names.emplace_back(std::string(mame));
        }

        for (const auto& name : dst_names)
        {
            if (!src_has_fn(name))
            {
                logger.logFmt(
                    mrpt::system::LVL_WARN,
                    "warn_on_field_padding_mismatch: destination map has field '%s' (%s) but "
                    "source map does not: points inserted from source will be "
                    "zero-padded for this field.",
                    name.c_str(), type_label);
                any = true;
            }
        }
    };

    // float fields (excluding x/y/z which are always present)
    check(
        dst.getPointFieldNames_float_except_xyz(),
        [&](const std::string& name) -> bool
        {
            const auto* v = src.getPointsBufferRef_float_field(name);
            return v && !v->empty();
        },
        "float");

    // double fields
    check(
        dst.getPointFieldNames_double(),
        [&](const std::string& name) -> bool
        {
            const auto* v = src.getPointsBufferRef_double_field(name);
            return v && !v->empty();
        },
        "double");

    // uint16 fields
    check(
        dst.getPointFieldNames_uint16(),
        [&](const std::string& name) -> bool
        {
            const auto* v = src.getPointsBufferRef_uint16_field(name);
            return v && !v->empty();
        },
        "uint16");

    // uint8 fields
    check(
        dst.getPointFieldNames_uint8(),
        [&](const std::string& name) -> bool
        {
            const auto* v = src.getPointsBufferRef_uint8_field(name);
            return v && !v->empty();
        },
        "uint8");

    return any;
}
