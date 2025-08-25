/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   register.cpp
 * @brief  RTTI registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mp2p_icp/LogRecord.h>
#include <mp2p_icp/Matcher_Adaptive.h>
#include <mp2p_icp/Matcher_Point2Line.h>
#include <mp2p_icp/Matcher_Point2Plane.h>
#include <mp2p_icp/Matcher_Points_DistanceThreshold.h>
#include <mp2p_icp/Matcher_Points_InlierRatio.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/QualityEvaluator_PairedRatio.h>
#include <mp2p_icp/QualityEvaluator_RangeImageSimilarity.h>
#include <mp2p_icp/QualityEvaluator_Voxels.h>
#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/Solver_Horn.h>
#include <mp2p_icp/Solver_OLAE.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/initializer.h>

/** \defgroup mp2p_icp_grp mp2p_icp library
 * Main library for ICP components and pipelines.
 */

MRPT_INITIALIZER(register_mp2p_icp)
{
    using mrpt::rtti::registerClass;

    registerClass(CLASS_ID(mp2p_icp::ICP));
    registerClass(CLASS_ID(mp2p_icp::ICP_LibPointmatcher));

    registerClass(CLASS_ID(mp2p_icp::Solver));
    registerClass(CLASS_ID(mp2p_icp::Solver_OLAE));
    registerClass(CLASS_ID(mp2p_icp::Solver_GaussNewton));
    registerClass(CLASS_ID(mp2p_icp::Solver_Horn));

    registerClass(CLASS_ID(mp2p_icp::Matcher));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Points_DistanceThreshold));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Points_InlierRatio));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Point2Line));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Point2Plane));
    registerClass(CLASS_ID(mp2p_icp::Matcher_Adaptive));

    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_PairedRatio));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_RangeImageSimilarity));
    registerClass(CLASS_ID(mp2p_icp::QualityEvaluator_Voxels));

    registerClass(CLASS_ID(mp2p_icp::LogRecord));
    registerClass(CLASS_ID(mp2p_icp::Parameters));
}
