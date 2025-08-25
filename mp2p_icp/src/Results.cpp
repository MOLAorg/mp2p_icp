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

#include <mp2p_icp/Results.h>
#include <mrpt/serialization/CArchive.h>

#include <ostream>

using namespace mp2p_icp;

static const uint8_t SERIALIZATION_VERSION = 0;

void Results::serializeTo(mrpt::serialization::CArchive& out) const
{
    out.WriteAs<uint8_t>(SERIALIZATION_VERSION);
    out << optimal_tf << optimalScale << nIterations;
    out << static_cast<uint8_t>(terminationReason);
    out << quality;
    finalPairings.serializeTo(out);
}
void Results::serializeFrom(mrpt::serialization::CArchive& in)
{
    const auto readVersion = in.ReadAs<uint8_t>();

    ASSERT_EQUAL_(readVersion, SERIALIZATION_VERSION);

    in >> optimal_tf >> optimalScale >> nIterations;
    terminationReason = static_cast<IterTermReason>(in.ReadAs<uint8_t>());
    in >> quality;
    finalPairings.serializeFrom(in);
}

mrpt::serialization::CArchive& mp2p_icp::operator<<(
    mrpt::serialization::CArchive& out, const Results& obj)
{
    obj.serializeTo(out);
    return out;
}

mrpt::serialization::CArchive& mp2p_icp::operator>>(mrpt::serialization::CArchive& in, Results& obj)
{
    obj.serializeFrom(in);
    return in;
}

void Results::print(std::ostream& o) const
{
    o << "- optimalPoseLocalWrtGlobal: " << optimal_tf.mean
      << "\n"
         "- quality: "
      << 100 * quality
      << " %\n"
         "- iterations: "
      << nIterations
      << "\n"
         "- terminationReason: "
      << mrpt::typemeta::TEnumType<mp2p_icp::IterTermReason>::value2name(terminationReason) << "\n"
      << "- finalPairings: " << finalPairings.contents_summary() << "\n";
}
