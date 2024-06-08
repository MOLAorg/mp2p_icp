/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
    if (pcGlobal)
    {
        out.WriteAs<bool>(true);
        out << *pcGlobal;
    }
    else { out.WriteAs<bool>(false); }
    if (pcLocal)
    {
        out.WriteAs<bool>(true);
        out << *pcLocal;
    }
    else { out.WriteAs<bool>(false); }
    out << initialGuessLocalWrtGlobal;
    out << icpParameters;
    out << icpResult;
    out << iterationsDetails;
}
void LogRecord::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    *this = LogRecord();

    switch (version)
    {
        case 0:
        {
            if (in.ReadAs<bool>())
            {
                pcGlobal = metric_map_t::Create();
                in >> const_cast<metric_map_t&>(*pcGlobal);
            }
            if (in.ReadAs<bool>())
            {
                pcLocal = metric_map_t::Create();
                in >> const_cast<metric_map_t&>(*pcLocal);
            }

            in >> initialGuessLocalWrtGlobal >> icpParameters >> icpResult >>
                iterationsDetails;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

bool LogRecord::save_to_file(const std::string& fileName) const
{
    try
    {
        auto f = mrpt::io::CFileGZOutputStream(fileName);
        if (!f.is_open()) return false;

        auto arch = mrpt::serialization::archiveFrom(f);
        arch << *this;

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[LogRecord::save_to_file] Error: " << e.what();
        return false;
    }
}

bool LogRecord::load_from_file(const std::string& fileName)
{
    try
    {
        auto f = mrpt::io::CFileGZInputStream(fileName);
        if (!f.is_open()) return false;

        auto arch = mrpt::serialization::archiveFrom(f);
        arch >> *this;

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[LogRecord::save_to_file] Error: " << e.what();
        return false;
    }
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
    in >> optimalPose >> pairings;
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
