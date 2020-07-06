/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mp2p_icp/WeightParameters.h>
#include <mrpt/containers/Parameters.h>
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

    /** Max. number of pairings per layer (point-to-point, plane-to-plane...).
     * Decimation of the point cloud being registered against the reference
     * one. The speed-up comes from a decimation of the number of KD-tree
     * queries, the most expensive step in ICP
     */
    uint32_t maxPairsPerLayer{500};

    /** If the correction in all translation coordinates (X,Y,Z) is below
     * this threshold (in meters), iterations are terminated (Default:1e-6)
     */
    double minAbsStep_trans{5e-4};

    /** If the correction in all rotation coordinates (yaw,pitch,roll) is
     * below this threshold (in radians), iterations are terminated
     * (Default:1e-6) */
    double minAbsStep_rot{1e-4};
    /** @} */

    /** [Only for ICP_GaussNewton] Maximum number of iterations trying to solve
     * for the optimal pose, within each ICP iteration.
     */
    uint32_t maxInnerLoopIterations{6};

    /** Weight and robust kernel parameters associated with the low-level
     * optimal pose estimation algorithms */
    WeightParameters pairingsWeightParameters;

    void load_from(const mrpt::containers::Parameters& p);
    void save_to(mrpt::containers::Parameters& p) const;
};

}  // namespace mp2p_icp
