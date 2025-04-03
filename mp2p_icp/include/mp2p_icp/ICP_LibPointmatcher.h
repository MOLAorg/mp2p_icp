/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/metricmap.h>
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
    DEFINE_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp)

   public:
    void initialize_derived(const mrpt::containers::yaml& params) override;

    void align(
        const metric_map_t& pcLocal, const metric_map_t& pcGlobal,
        const mrpt::math::TPose3D& initialGuessLocalWrtGlobal, const Parameters& p, Results& result,
        const std::optional<mrpt::poses::CPose3DPDFGaussianInf>& prior = std::nullopt,
        const mrpt::optional_ref<LogRecord>& outputDebugInfo           = std::nullopt) override;

    struct ParametersLibpointmatcher
    {
        double                        RandomSamplingDataPointsFilter_prob = 1.0;
        unsigned int                  SurfaceNormalDataPointsFilter_knn   = 10;
        unsigned int                  KDTreeMatcher_knn                   = 1;
        std::string                   outlierFilter = "VarTrimmedDistOutlierFilter";
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
    std::string pm_icp_yaml_settings_;
#if defined(MP2P_HAS_LIBPOINTMATCHER)

#endif
};
}  // namespace mp2p_icp
