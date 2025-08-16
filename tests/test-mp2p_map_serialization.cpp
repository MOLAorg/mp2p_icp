/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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
