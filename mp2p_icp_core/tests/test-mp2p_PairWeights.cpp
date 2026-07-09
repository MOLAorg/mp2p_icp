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
 * @file   test-mp2p_PairWeights.cpp
 * @brief  Unit test for PairWeights
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/PairWeights.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mp2p_icp;

namespace
{
void test_default_construction()
{
    PairWeights pw;

    ASSERT_EQUAL_(pw.pt2pt, 1.0);
    ASSERT_EQUAL_(pw.pt2ln, 1.0);
    ASSERT_EQUAL_(pw.pt2pl, 1.0);
    ASSERT_EQUAL_(pw.ln2ln, 1.0);
    ASSERT_EQUAL_(pw.pl2pl, 1.0);
}

void test_load_from_yaml()
{
    mrpt::containers::yaml cfg;
    cfg["pt2pt"] = 2.0;
    cfg["pt2ln"] = 3.0;
    cfg["pt2pl"] = 4.0;
    cfg["ln2ln"] = 5.0;
    cfg["pl2pl"] = 6.0;

    PairWeights pw;
    pw.load_from(cfg);

    ASSERT_EQUAL_(pw.pt2pt, 2.0);
    ASSERT_EQUAL_(pw.pt2ln, 3.0);
    ASSERT_EQUAL_(pw.pt2pl, 4.0);
    ASSERT_EQUAL_(pw.ln2ln, 5.0);
    ASSERT_EQUAL_(pw.pl2pl, 6.0);
}

void test_save_to_yaml()
{
    PairWeights pw;
    pw.pt2pt = 2.5;
    pw.pt2ln = 3.5;
    pw.pt2pl = 4.5;
    pw.ln2ln = 5.5;
    pw.pl2pl = 6.5;

    mrpt::containers::yaml cfg;
    pw.save_to(cfg);

    ASSERT_EQUAL_(cfg["pt2pt"].as<double>(), 2.5);
    ASSERT_EQUAL_(cfg["pt2ln"].as<double>(), 3.5);
    ASSERT_EQUAL_(cfg["pt2pl"].as<double>(), 4.5);
    ASSERT_EQUAL_(cfg["ln2ln"].as<double>(), 5.5);
    ASSERT_EQUAL_(cfg["pl2pl"].as<double>(), 6.5);
}

void test_serialization()
{
    PairWeights pw1;
    pw1.pt2pt = 1.5;
    pw1.pt2ln = 2.5;
    pw1.pt2pl = 3.5;
    pw1.ln2ln = 4.5;
    pw1.pl2pl = 5.5;

    // Serialize
    mrpt::io::CMemoryStream buf;
    auto                    arch_out = mrpt::serialization::archiveFrom(buf);
    pw1.serializeTo(arch_out);

    // Deserialize
    buf.Seek(0);
    auto        arch_in = mrpt::serialization::archiveFrom(buf);
    PairWeights pw2;
    pw2.serializeFrom(arch_in);

    ASSERT_EQUAL_(pw2.pt2pt, 1.5);
    ASSERT_EQUAL_(pw2.pt2ln, 2.5);
    ASSERT_EQUAL_(pw2.pt2pl, 3.5);
    ASSERT_EQUAL_(pw2.ln2ln, 4.5);
    ASSERT_EQUAL_(pw2.pl2pl, 5.5);
}

void test_partial_yaml_load()
{
    // Test loading only some fields
    mrpt::containers::yaml cfg;
    cfg["pt2pt"] = 10.0;
    cfg["pt2ln"] = 20.0;
    cfg["pt2pl"] = 30.0;
    cfg["ln2ln"] = 40.0;
    cfg["pl2pl"] = 50.0;

    PairWeights pw;
    pw.load_from(cfg);

    ASSERT_EQUAL_(pw.pt2pt, 10.0);
    ASSERT_EQUAL_(pw.pt2ln, 20.0);
    ASSERT_EQUAL_(pw.pt2pl, 30.0);
    ASSERT_EQUAL_(pw.ln2ln, 40.0);
    ASSERT_EQUAL_(pw.pl2pl, 50.0);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_default_construction();
        std::cout << "test_default_construction: Success ✅" << std::endl;

        test_load_from_yaml();
        std::cout << "test_load_from_yaml: Success ✅" << std::endl;

        test_save_to_yaml();
        std::cout << "test_save_to_yaml: Success ✅" << std::endl;

        test_serialization();
        std::cout << "test_serialization: Success ✅" << std::endl;

        test_partial_yaml_load();
        std::cout << "test_partial_yaml_load: Success ✅" << std::endl;

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: ❌\n" << e.what() << std::endl;
        return 1;
    }
}
