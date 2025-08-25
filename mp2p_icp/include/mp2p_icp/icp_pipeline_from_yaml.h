/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   icp_pipeline_from_yaml.h
 * @brief  Loads and setup an ICP pipeline from a YAML configuration file
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2020
 */
#pragma once

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameters.h>
#include <mrpt/containers/yaml.h>

namespace mp2p_icp
{
/** Sets up an ICP pipeline from a YAML configuration file.
 *
 * The basic structure of the YAML configuration block is (see also example YAML
 * files):
 * \code
 * class_name: mp2p_icp::ICP
 *
 * # See: mp2p_icp::Parameter
 * params:
 *   maxIterations: 100
 *   # ...
 *
 * solvers:
 *   - class: mp2p_icp::Solver_GaussNewton
 *     params:
 *       maxIterations: 10
 *
 * # Sequence of one or more pairs (class, params) defining mp2p_icp::Matcher
 * # instances to pair geometric entities between pointclouds.
 * matchers:
 *   - class: mp2p_icp::Matcher_Points_DistanceThreshold
 *     params:
 *       threshold: 0.20
 *       maxLocalPointsPerLayer: 500
 *
 * quality:
 *   - class: mp2p_icp::QualityEvaluator_PairedRatio
 *     params:
 *      thresholdDistance: 0.1
 * \endcode
 *
 * \ingroup mp2p_icp_grp
 */
std::tuple<mp2p_icp::ICP::Ptr, mp2p_icp::Parameters> icp_pipeline_from_yaml(
    const mrpt::containers::yaml&       config,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

}  // namespace mp2p_icp
