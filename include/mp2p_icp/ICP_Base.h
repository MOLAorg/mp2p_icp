/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Base.h
 * @brief  Virtual interface for ICP algorithms. Useful for RTTI class searches.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mp2p_icp/IterTermReason.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/optimal_tf_common.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <cstdint>
#include <functional>  //reference_wrapper
#include <memory>

namespace mp2p_icp
{
/** Common interface for ICP algorithms.
 * The main API entry point is align().
 *
 * \sa This class can be used as parent class in RTTI queries to find available
 * ICP algorithms.
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_Base : public mrpt::system::COutputLogger, public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(ICP_Base)

   public:
    /** Register two point clouds (possibly after having been preprocessed to
     * extract features, etc.) and returns the relative pose of pc2 with respect
     * to pc1.
     */
    virtual void align(
        const pointcloud_t& pc1, const pointcloud_t& pc2,
        const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
        Results& result);

   protected:
    struct ICP_State
    {
        ICP_State(const pointcloud_t& pcs1, const pointcloud_t& pcs2)
            : pc1(pcs1), pc2(pcs2)
        {
        }

        const pointcloud_t&                                      pc1;
        const pointcloud_t&                                      pc2;
        std::map<std::string, mrpt::maps::TMatchingParams>       mps;
        std::map<std::string, mrpt::maps::TMatchingExtraResults> mres;
        std::string      layerOfLargestPc;
        WeightedPairings currentPairings;
        // Current best transform:
        mrpt::poses::CPose3D current_solution;
        double               current_scale{1.0};
    };

    struct ICP_iteration_result
    {
        bool                 success{false};
        mrpt::poses::CPose3D new_solution;
        double               new_scale{1.0};
        // TODO: Outliers info
    };

    /** Used internally by ICP implementations to find correspondences between
     * two pointclouds. */
    WeightedPairings commonFindPairings(ICP_State& s, const Parameters& p);

    /** Implemented by specific ICP algorithms, to be run at each ICP iteration.
     * It must search for matchings given the current pose estimate, and
     * evaluate the next new pose, if enough data is available possible.
     */
    virtual void impl_ICP_iteration(
        ICP_State& state, const Parameters& p, ICP_iteration_result& out) = 0;

    /** Populates state.mps according to pointcloud stats and to input
     * parameters */
    void prepareMatchingParams(ICP_State& state, const Parameters& p) const;
};
}  // namespace mp2p_icp
