/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LogRecord.h
 * @brief  A record of the inputs and outputs of an ICP run.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 15, 2021
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/serialization/CSerializable.h>

#include <optional>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** Details on an ICP run, loadable from the GUI tool mp2p-icp-log-viewer.
 *
 */
class LogRecord : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(LogRecord, mp2p_icp)

   public:
    LogRecord()  = default;
    ~LogRecord() = default;

    /** @name Data fields
     * @{ */

    /** The ICP input global and local point clouds: */
    metric_map_t::ConstPtr pcGlobal, pcLocal;

    mrpt::math::TPose3D initialGuessLocalWrtGlobal;

    mp2p_icp::Parameters icpParameters;
    mp2p_icp::Results    icpResult;

    struct DebugInfoPerIteration
    {
        mrpt::poses::CPose3D optimalPose;  //!< for this ICP iteration
        Pairings             pairings;

        void serializeTo(mrpt::serialization::CArchive& out) const;
        void serializeFrom(mrpt::serialization::CArchive& in);

        DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::LogRecord::DebugInfoPerIteration)
    };
    using iteration_idx_t   = std::size_t;
    using IterationsDetails = std::map<iteration_idx_t, DebugInfoPerIteration>;

    std::optional<IterationsDetails> iterationsDetails;

    /** @} */

    /** @name Methods
     * @{ */

    /** Saves the record object to a file, using MRPT serialization and
     *  using on-the-fly GZIP compression.
     * \return true on success.
     */
    bool save_to_file(const std::string& fileName) const;

    /** Loads the record object from a file. See \save_to_file()
     * \return true on success.
     */
    bool load_from_file(const std::string& fileName);

    /** Static method alternative to load_from_file().
     *  Throws on any error.
     *
     * \return The loaded object.
     */
    static LogRecord LoadFromFile(const std::string& fileName)
    {
        LogRecord lr;
        lr.load_from_file(fileName);
        return lr;
    }

    /** @} */
};

/** @} */

}  // namespace mp2p_icp

namespace mrpt::serialization
{
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive&                    out,
    const mp2p_icp::LogRecord::DebugInfoPerIteration& obj);

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive&              in,
    mp2p_icp::LogRecord::DebugInfoPerIteration& obj);

}  // namespace mrpt::serialization
