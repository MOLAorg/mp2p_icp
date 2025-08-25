/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   test-mp2p_map_serialization.cpp
 * @brief  Unit tests for metric_map_t serialization
 * @author Jose Luis Blanco Claraco
 * @date   Jun 2, 2025
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/serialization/CArchive.h>

namespace
{
void unit_test_1()
{
    mp2p_icp::metric_map_t  map_in;
    mrpt::io::CMemoryStream memStream;
    auto                    arch = mrpt::serialization::archiveFrom(memStream);
    arch << map_in;
    memStream.Seek(0);

    mp2p_icp::metric_map_t map_out;
    arch >> map_out;

    ASSERT_(map_out.empty());
    ASSERT_(map_out.metadata.isNullNode() || map_out.metadata.empty());
}

void unit_test_2()
{
    mp2p_icp::metric_map_t map_in;
    {
        auto pts = std::make_shared<mrpt::maps::CSimplePointsMap>();
        pts->insertPoint(1.0, 2.0, 3.0);
        pts->insertPoint(4.0, 5.0, 6.0);
        map_in.layers["raw"] = pts;

        map_in.metadata["name"] = "Test Map";
        map_in.metadata["spot"] = "Alabama";
    }

    mrpt::io::CMemoryStream memStream;
    auto                    arch = mrpt::serialization::archiveFrom(memStream);
    arch << map_in;
    memStream.Seek(0);

    mp2p_icp::metric_map_t map_out;
    arch >> map_out;

    ASSERT_(!map_out.empty());
    ASSERT_(!map_out.metadata.empty());
    ASSERT_EQUAL_(map_in.contents_summary(), map_out.contents_summary());
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        unit_test_1();
        unit_test_2();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
