/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Horn_MultiCloud.h
 * @brief  ICP registration for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */
#pragma once

#include <mp2p_icp/ICP_Base.h>
#include <mp2p_icp/IterTermReason.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/rtti/CObject.h>
#include <vector>

namespace mp2p_icp
{
/** ICP registration for pointclouds split in different "layers"
 *
 * \ingroup mp2p_icp_grp
 */
class ICP_Horn_MultiCloud : public ICP_Base
{
	DEFINE_MRPT_OBJECT(ICP_Horn_MultiCloud, mp2p_icp);

   protected:
    // See base class docs
    void impl_ICP_iteration(
        ICP_State& s, const Parameters& p, ICP_iteration_result& out) override;
};

}  // namespace mp2p_icp
