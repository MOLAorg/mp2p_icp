/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver.h
 * @brief  Virtual base class for optimal alignment solvers (one step in ICP).
 * @author Jose Luis Blanco Claraco
 * @date   August 3, 2020
 */
#pragma once

#include <mp2p_icp/OptimalTF_Result.h>
#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/WeightParameters.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <any>
#include <optional>

namespace mp2p_icp
{
class Solver;

/** Defines the context of a solver within the successive ICP iterations.
 *
 * \ingroup mp2p_icp_grp
 */
struct SolverContext
{
    SolverContext() = default;

    std::optional<mrpt::poses::CPose3D> guessRelativePose;
    std::optional<mrpt::poses::CPose3D> currentCorrectionFromInitialGuess;
    std::optional<mrpt::poses::CPose3D> lastIcpStepIncrement;
    // room for optional solver-specific context:
    mutable std::map<const Solver*, std::map<std::string, std::any>>
        perSolverPersistentData;

    std::optional<uint32_t> icpIteration;
};

/** Virtual base class for optimal alignment solvers (one step in ICP).
 *
 * Each "solver" implementation takes a list of correspondences, and returns the
 * optimal (in some sense) relative SE(3) pose minimizing some particular error
 * function.
 *
 * \ingroup mp2p_icp_grp
 */
class Solver : public mrpt::system::COutputLogger,
               public mrpt::rtti::CObject,
               public mp2p_icp::Parameterizable
{
    DEFINE_VIRTUAL_MRPT_OBJECT(Solver)

   public:
    /** Check each derived class to see required and optional parameters. */
    virtual void initialize(const mrpt::containers::yaml& params);

    /** Finds correspondences between the two point clouds.
     * "out" is not cleared, but new pairings added to it.
     *
     * \return true if the method was actually invoked (due to the filter in
     * runFromIteration and runUpToIteration) and valid solution was found.
     */
    virtual bool optimal_pose(
        const Pairings& pairings, OptimalTF_Result& out,
        const SolverContext& sc) const;

    uint32_t runFromIteration = 0;
    uint32_t runUpToIteration = 0;  //!< 0: no limit

    double runUntilTranslationCorrectionSmallerThan = 0;

    /** Can be used to disable one of a set of solvers in a pipeline */
    bool enabled = true;

   protected:
    virtual bool impl_optimal_pose(
        const Pairings& pairings, OptimalTF_Result& out,
        const SolverContext& sc) const = 0;
};

}  // namespace mp2p_icp
