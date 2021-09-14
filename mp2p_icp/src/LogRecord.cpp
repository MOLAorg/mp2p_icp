/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/LogRecord.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>

IMPLEMENTS_MRPT_OBJECT(LogRecord, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t LogRecord::serializeGetVersion() const { return 0; }
void    LogRecord::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << pcGlobal << pcLocal;
    out << initialGuessLocalWrtGlobal;
    out << icpParameters;
    out << icpResult;
    out << iterationsDetails;
}
void LogRecord::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> pcGlobal >> pcLocal >> initialGuessLocalWrtGlobal >>
                icpParameters >> icpResult >> iterationsDetails;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

bool LogRecord::save_to_file(const std::string& fileName) const
{
    auto f = mrpt::io::CFileGZOutputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch << *this;

    return true;
}

bool LogRecord::load_from_file(const std::string& fileName)
{
    auto f = mrpt::io::CFileGZInputStream(fileName);
    if (!f.is_open()) return false;

    auto arch = mrpt::serialization::archiveFrom(f);
    arch >> *this;

    return true;
}

static const uint8_t DIPI_SERIALIZATION_VERSION = 0;

void LogRecord::DebugInfoPerIteration::serializeTo(
    mrpt::serialization::CArchive& out) const
{
    out.WriteAs<uint8_t>(DIPI_SERIALIZATION_VERSION);

    out << optimalPose << pairings;
}
void LogRecord::DebugInfoPerIteration::serializeFrom(
    mrpt::serialization::CArchive& in)
{
    const auto readVersion = in.ReadAs<uint8_t>();
    ASSERT_EQUAL_(readVersion, DIPI_SERIALIZATION_VERSION);
}

mrpt::serialization::CArchive& mrpt::serialization::operator<<(
    mrpt::serialization::CArchive&          out,
    const LogRecord::DebugInfoPerIteration& obj)
{
    obj.serializeTo(out);
    return out;
}

mrpt::serialization::CArchive& mrpt::serialization::operator>>(
    mrpt::serialization::CArchive& in, LogRecord::DebugInfoPerIteration& obj)
{
    obj.serializeFrom(in);
    return in;
}
