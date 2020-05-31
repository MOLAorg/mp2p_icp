/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
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

#include <mp2p_icp/ICP_Base.h>
#include <mp2p_icp/IterTermReason.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/rtti/CObject.h>

#include <vector>

#if defined(MP2P_HAS_LIBPOINTMATCHER)
#endif

namespace mp2p_icp
{
/** ICP registration for pointclouds split in different "layers"
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_LibPointmatcher : public ICP_Base
{
    DEFINE_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp);

   public:
    void align(
        const pointcloud_t& pc1, const pointcloud_t& pc2,
        const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
        Results& result) override;

   protected:
    // See base class docs
    void impl_ICP_iteration(
        [[maybe_unused]] ICP_State& s, [[maybe_unused]] const Parameters& p,
        [[maybe_unused]] ICP_iteration_result& out) override
    {
        // Not used in this class
    }

   private:
#if defined(MP2P_HAS_LIBPOINTMATCHER)
#endif
};
}  // namespace mp2p_icp
