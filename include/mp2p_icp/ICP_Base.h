/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/pointcloud.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <cstdint>
#include <memory>

namespace mp2p_icp
{
/** Virtual interface for ICP algorithms. Useful for RTTI class searches.
 *
 * The main API entry point is align().
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_Base : public mrpt::system::COutputLogger, public mrpt::rtti::CObject
{
    DEFINE_MRPT_OBJECT(ICP_Base)

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
};
}  // namespace mp2p_icp
