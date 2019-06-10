/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Base.cpp
 * @brief  Virtual interface for ICP algorithms. Useful for RTTI class searches.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/ICP_Base.h>

IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(ICP_Base, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void ICP_Base::align(
    [[maybe_unused]] const pointcloud_t&        pc1,
    [[maybe_unused]] const pointcloud_t&        pc2,
    [[maybe_unused]] const mrpt::math::TPose3D& init_guess_m2_wrt_m1,
    [[maybe_unused]] const Parameters& p, [[maybe_unused]] Results& result)
{
    THROW_EXCEPTION("ICP_Base::align() not implemented in derived class");
}
