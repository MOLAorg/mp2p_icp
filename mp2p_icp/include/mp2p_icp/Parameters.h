/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mp2p_icp/WeightParameters.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>  // DEG2RAD()
#include <mrpt/serialization/CSerializable.h>

#include <cstdint>
#include <functional>
#include <string>

namespace mp2p_icp
{
class metric_map_t;  // Frwd decl

/** ICP parameters.
 * \sa ICP_Base
 * \ingroup mp2p_icp_grp
 */
struct Parameters : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(Parameters, mp2p_icp)

   public:
    /** @name Termination criteria
        @{ */
    /** Maximum number of ICP iterations to run. */
    uint32_t maxIterations{40};

    /** If the correction in all translation coordinates (X,Y,Z) is below
     * this threshold (in meters), iterations are terminated (Default:1e-6)
     */
    double minAbsStep_trans{5e-4};

    /** If the correction in all rotation coordinates (yaw,pitch,roll) is
     * below this threshold (in radians), iterations are terminated
     * (Default:1e-6) */
    double minAbsStep_rot{1e-4};
    /** @} */

    /** @name Debugging and logging
        @{ */

    /** If true, debug files useful to inspect how ICP works internally will be
     * generated and saved to disk for posterior inspection with a GUI.
     *
     * The same mp2p_icp::LogRecord object saved to disk will be also returned
     * by ICP::align().
     *
     * \sa debugFileNameFormat, saveIterationDetails
     */
    bool generateDebugFiles = false;

    /** If enabled, the intermediary pairings and SE(3) solution for each
     *  ICP step will be also stored in the mp2p_icp::LogRecord to help
     *  investigating how ICP made progress.
     */
    bool saveIterationDetails = false;

    /** If set to N>1, only 1 out of N ICP iterations will be kept.
     *  Applicable if saveIterationDetails is true.
     *  Useful to save tons of disk space for large datasets (!).
     */
    uint32_t decimationIterationDetails = 10;

    /** If set to N>1, only 1 out of N log files will be actually generated.
     *  Useful to save tons of disk space for large datasets (!).
     */
    uint32_t decimationDebugFiles = 1;

    /** Generated files format, if generateDebugFiles is true. */
    std::string debugFileNameFormat =
        "icp-run-$UNIQUE_ID-local-$LOCAL_ID$LOCAL_LABEL-"
        "global-$GLOBAL_ID$GLOBAL_LABEL.icplog";

    /** Function to apply to the local and global maps before saving the map to
     * a log file. Useful to apply deletion filters to save space and time.
     */
    std::function<void(mp2p_icp::metric_map_t&)> functor_before_logging_local,
        functor_before_logging_global;

    bool debugPrintIterationProgress = false;

    /** Quality checkpoints: if the quality is smaller than the given threshold
     * at the prescribed iteration, ICP will be aborted. */
    std::map<uint32_t, double> quality_checkpoints = {{50U, 0.05}, {100U, 0.10}};

    /** @} */

    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;
};

}  // namespace mp2p_icp
