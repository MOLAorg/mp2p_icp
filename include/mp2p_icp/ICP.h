/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP.h
 * @brief  Generic ICP algorithm container.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mp2p_icp/IterTermReason.h>
#include <mp2p_icp/Matcher.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/QualityEvaluator.h>
#include <mp2p_icp/QualityEvaluator_PairedRatio.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/Solver.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdint>
#include <functional>  //reference_wrapper
#include <memory>

namespace mp2p_icp
{
/** Generic ICP algorithm container: builds a custom ICP pipeline by selecting
 * algorithm and parameter for each stage.
 *
 * The main API entry point is align().
 *
 * \todo Add pipeline picture.
 *
 * Several solvers may exists, but the output from the first one returning
 * "true" will be used. This is by design, to enable different solver algorithms
 * depending on the ICP iteration.
 *
 * \ingroup mp2p_icp_grp
 */
class ICP : public mrpt::system::COutputLogger, public mrpt::rtti::CObject
{
    DEFINE_MRPT_OBJECT(ICP, mp2p_icp)

   public:
    /** Register (align) two point clouds (possibly after having been
     * preprocessed to extract features, etc.) and returns the relative pose of
     * pc2 with respect to pc1.
     */
    virtual void align(
        const pointcloud_t& pc1, const pointcloud_t& pc2,
        const mrpt::math::TPose3D& initialGuessM2wrtM1, const Parameters& p,
        Results& result);

    /** @name Module: Solver instances
     * @{ */
    using solver_list_t = std::vector<mp2p_icp::Solver::Ptr>;

    /** Create and configure one or more "Solver" modules from YAML-like config
     *block. Config must be a *sequence* of one or more entries, each with a
     *`class` and a `params` dictionary entries.
     *
     * Example:
     *\code
     *- class: mp2p_icp::Solver_Horn
     *  params:
     *   # Parameters depend on the particular class
     *   # none
     *\endcode
     *
     * Alternatively, the objects can be directly created via solvers().
     */
    void initialize_solvers(const mrpt::containers::yaml& params);

    static void initialize_solvers(
        const mrpt::containers::yaml& params, ICP::solver_list_t& lst);

    const solver_list_t& solvers() const { return solvers_; }
    solver_list_t&       solvers() { return solvers_; }

    /** Runs a set of solvers. */
    static bool run_solvers(
        const solver_list_t& solvers, const Pairings& pairings,
        OptimalTF_Result& out, const WeightParameters& wp,
        const SolverContext& sc = {});

    /** @} */

    /** @name Module: Matcher instances
     * @{ */
    using matcher_list_t = std::vector<mp2p_icp::Matcher::Ptr>;

    /** Create and configure one or more "Match" modules from YAML-like config
     *block. Config must be a *sequence* of one or more entries, each with a
     *`class` and a `params` dictionary entries.
     *
     * Example:
     *\code
     *- class: mp2p_icp::Matcher_Points_DistanceThreshold
     *  params:
     *   # Parameters depend on the particular class
     *   threshold: 1.0
     *\endcode
     *
     * Alternatively, the objects can be directly created via matchers().
     */
    void initialize_matchers(const mrpt::containers::yaml& params);

    static void initialize_matchers(
        const mrpt::containers::yaml& params, ICP::matcher_list_t& lst);

    const matcher_list_t& matchers() const { return matchers_; }
    matcher_list_t&       matchers() { return matchers_; }

    /** Runs a set of matchers. */
    static Pairings run_matchers(
        const matcher_list_t& matchers, const pointcloud_t& pc1,
        const pointcloud_t& pc2, const mrpt::poses::CPose3D& pc2_wrt_pc1,
        const MatchContext& mc = {});

    /** @} */

    /** @name Module: QualityEvaluator instances
     * @{ */

    struct QualityEvaluatorEntry
    {
        QualityEvaluatorEntry(mp2p_icp::QualityEvaluator::Ptr o, double w)
            : obj(o), relativeWeight(w)
        {
        }

        mp2p_icp::QualityEvaluator::Ptr obj;
        double                          relativeWeight = 1.0;
    };
    using quality_eval_list_t = std::vector<QualityEvaluatorEntry>;

    /** Create and configure one or more  "QualityEvaluator" modules from
     *YAML-like config block. Config must be a *sequence* of one or more
     *entries, each with a `class` and a `params` dictionary entries.
     *
     * Example:
     *\code
     *- class: mp2p_icp::QualityEvaluator_PairedRatio
     *  weight: 1.0  # (Optional if only one quality evaluator is defined)
     *  params:
     *   # Parameters depend on the particular class
     *   # xxx: yyyy
     *\endcode
     *
     * Alternatively, the objects can be directly created via matchers().
     */
    void initialize_quality_evaluators(const mrpt::containers::yaml& params);

    static void initialize_quality_evaluators(
        const mrpt::containers::yaml& params, quality_eval_list_t& lst);

    const quality_eval_list_t& quality_evaluators() const
    {
        return quality_evaluators_;
    }
    quality_eval_list_t& quality_evaluators() { return quality_evaluators_; }

    static double evaluate_quality(
        const quality_eval_list_t& evaluators, const pointcloud_t& pcGlobal,
        const pointcloud_t& pcLocal, const mrpt::poses::CPose3D& localPose,
        const Pairings& finalPairings);

    /** @} */

   protected:
    solver_list_t       solvers_;
    matcher_list_t      matchers_;
    quality_eval_list_t quality_evaluators_ = {
        {QualityEvaluator_PairedRatio::Create(), 1.0}};

    struct ICP_State
    {
        ICP_State(const pointcloud_t& pcs1, const pointcloud_t& pcs2)
            : pc1(pcs1), pc2(pcs2)
        {
        }

        const pointcloud_t& pc1;
        const pointcloud_t& pc2;
        std::string         layerOfLargestPc;
        Pairings            currentPairings;
        OptimalTF_Result    currentSolution;
        uint32_t            currentIteration = 0;
    };
};
}  // namespace mp2p_icp
