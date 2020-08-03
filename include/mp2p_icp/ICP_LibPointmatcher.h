/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_LibPointmatcher.h
 * @brief  ICP wrapper on libpointmatcher
 * @author Jose Luis Blanco Claraco
 * @date   May 31, 2020
 */
#pragma once

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/IterTermReason.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/rtti/CObject.h>

#include <vector>

namespace mp2p_icp
{
/** ICP wrapper on libpointmatcher
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_LibPointmatcher : public ICP
{
    DEFINE_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp);

   public:
    void align(
        const pointcloud_t& pc1, const pointcloud_t& pc2,
        const mrpt::math::TPose3D& initialGuessM2wrtM1, const Parameters& p,
        Results& result) override;

    struct ParametersLibpointmatcher
    {
        double       RandomSamplingDataPointsFilter_prob = 1.0;
        unsigned int SurfaceNormalDataPointsFilter_knn   = 10;
        unsigned int KDTreeMatcher_knn                   = 1;
        std::string  outlierFilter = "VarTrimmedDistOutlierFilter";
        std::map<std::string, double> outlierParams = {
            {"minRatio", 0.05},
            {"maxRatio", 0.95},
            {"lambda", 2.35},
        };
        std::string errorMinimizer = "PointToPlaneErrorMinimizer";
    };

    ParametersLibpointmatcher parametersLibpointmatcher;

    /** Returns true if mp2p_icp was built with libpointmatcher support. */
    static bool methodAvailable();

   private:
#if defined(MP2P_HAS_LIBPOINTMATCHER)
#endif
};
}  // namespace mp2p_icp
