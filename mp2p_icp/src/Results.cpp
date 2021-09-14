/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/Results.h>
#include <mrpt/serialization/CArchive.h>

using namespace mp2p_icp;

static const uint8_t SERIALIZATION_VERSION = 0;

void Results::serializeTo(mrpt::serialization::CArchive& out) const
{
    out.WriteAs<uint8_t>(SERIALIZATION_VERSION);
    out << optimal_tf << optimalScale << nIterations << terminationReason;
    finalPairings.serializeTo(out);
}
void Results::serializeFrom(mrpt::serialization::CArchive& in)
{
    const auto readVersion = in.ReadAs<uint8_t>();

    ASSERT_EQUAL_(readVersion, SERIALIZATION_VERSION);

    in >> optimal_tf >> optimalScale >> nIterations >> terminationReason;
    finalPairings.serializeFrom(in);
}

mrpt::serialization::CArchive& mp2p_icp::operator<<(
    mrpt::serialization::CArchive& out, const Results& obj)
{
    out.WriteAs<uint8_t>(0);
    out << obj.finalPairings << obj.nIterations << obj.optimalScale
        << obj.optimal_tf << obj.quality << obj.terminationReason;

    return out;
}

mrpt::serialization::CArchive& mp2p_icp::operator>>(
    mrpt::serialization::CArchive& in, Results& obj)
{
    in.ReadAs<uint8_t>();

    in >> obj.finalPairings >> obj.nIterations >> obj.optimalScale >>
        obj.optimal_tf >> obj.quality >> obj.terminationReason;

    return in;
}
