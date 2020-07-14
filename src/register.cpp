/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  RTTI registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/ICP_GaussNewton.h>
#include <mp2p_icp/ICP_Horn_MultiCloud.h>
#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mp2p_icp/ICP_OLAE.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/Matcher_Points_InlierRatio.h>
#include <mp2p_icp/QualityEvaluator_PairedRatio.h>
#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mp2p_icp/QualityEvaluator_Voxels.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/core/initializer.h>

MRPT_INITIALIZER(register_mp2p_icp)
{
    using mrpt::rtti::registerClass;

    registerClass(CLASS_ID(mp2p_icp::pointcloud_t));

    registerClass(CLASS_ID(mp2p_icp::ICP_Base));
    registerClass(CLASS_ID(mp2p_icp::ICP_OLAE));
    registerClass(CLASS_ID(mp2p_icp::ICP_GaussNewton));
    registerClass(CLASS_ID(mp2p_icp::ICP_Horn_MultiCloud));
    registerClass(CLASS_ID(mp2p_icp::ICP_LibPointmatcher));

    registerClass(CLASS_ID(mp2p_icp::Matcher));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Points_DistanceThreshold));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Points_InlierRatio));

    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_PairedRatio));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_RangeImageSimilarity));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_Voxels));
}
