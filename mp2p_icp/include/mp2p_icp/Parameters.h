/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mp2p_icp/WeightParameters.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>  // DEG2RAD()
#include <mrpt/serialization/CSerializable.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace mp2p_icp
{
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
     * \sa debugFileNameFormat
     */
    bool generateDebugFiles = false;

    /** Generated files format, if generateDebugFiles is true. */
    std::string debugFileNameFormat =
        "icp-run-$UNIQUE_ID-local-$LOCAL_ID$LOCAL_LABEL-"
        "global-$GLOBAL_ID$GLOBAL_LABEL.icplog";

    bool debugPrintIterationProgress = false;

    /** @} */

    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;
};

}  // namespace mp2p_icp
