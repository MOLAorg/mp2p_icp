/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_LibPointmatcher.cpp
 * @brief  ICP wrapper on libpointmatcher
 * @author Jose Luis Blanco Claraco
 * @date   May 31, 2020
 */

#include <mp2p_icp/ICP_LibPointmatcher.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>

#include <fstream>

#if defined(MP2P_HAS_LIBPOINTMATCHER)
#include <pointmatcher/PointMatcher.h>
#endif

IMPLEMENTS_MRPT_OBJECT(ICP_LibPointmatcher, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_LibPointmatcher::align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
#if defined(MP2P_HAS_LIBPOINTMATCHER)

    std::string icpImplConfigFile;  // = ...

    using PM = PointMatcher<float>;
    using DP = PM::DataPoints;

    // Load point clouds
    DP ptsFrom, ptsTo;

    // Create the default ICP algorithm
    PM::ICP icp;
    {
        // load YAML config
        std::ifstream ifs(icpImplConfigFile);
        if (!ifs.good())
        {
            THROW_EXCEPTION_FMT(
                "Cannot open config file %s", icpImplConfigFile.c_str());
        }
        icp.loadFromYaml(ifs);
    }

    int cloudDimension = ref.getEuclideanDim();
    ASSERT_EQUAL_(cloudDimension, 3U);

    PM::TransformationParameters translation =
        parseTranslation(initTranslation, cloudDimension);
    PM::TransformationParameters rotation =
        parseRotation(initRotation, cloudDimension);
    PM::TransformationParameters initTransfo = translation * rotation;

    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo))
    {
        cerr << endl
             << "Initial transformation is not rigid, identiy will be used"
             << endl;
        initTransfo = PM::TransformationParameters::Identity(
            cloudDimension + 1, cloudDimension + 1);
    }

    const DP initializedData = rigidTrans->compute(data, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(initializedData, ref);
    if (isVerbose)
        cout << "match ratio: "
             << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

    // Transform data to express it in ref
    DP data_out(initializedData);
    icp.transformations.apply(data_out, T);

    // Safe files to see the results
    ref.save(outputBaseFile + "_ref.vtk");
    data.save(outputBaseFile + "_data_in.vtk");
    data_out.save(outputBaseFile + "_data_out.vtk");
    if (isTransfoSaved)
    {
        ofstream transfoFile;
        string   initFileName     = outputBaseFile + "_init_transfo.txt";
        string   icpFileName      = outputBaseFile + "_icp_transfo.txt";
        string   completeFileName = outputBaseFile + "_complete_transfo.txt";

        transfoFile.open(initFileName.c_str());
        if (transfoFile.is_open())
        {
            transfoFile << initTransfo << endl;
            transfoFile.close();
        }
        else
        {
            cerr << "Unable to write the initial transformation file\n" << endl;
        }

        transfoFile.open(icpFileName.c_str());
        if (transfoFile.is_open())
        {
            transfoFile << T << endl;
            transfoFile.close();
        }
        else
        {
            cerr << "Unable to write the ICP transformation file\n" << endl;
        }

        transfoFile.open(completeFileName.c_str());
        if (transfoFile.is_open())
        {
            transfoFile << T * initTransfo << endl;
            transfoFile.close();
        }
        else
        {
            cerr << "Unable to write the complete transformation file\n"
                 << endl;
        }
    }
    else
    {
        if (isVerbose) cout << "ICP transformation:" << endl << T << endl;
    }

    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (const auto& kv1 : pcs1.point_layers)
    {
        // Ignore this layer?
        if (p.weight_pt2pt_layers.count(kv1.first) == 0) continue;

        pointcount1 += kv1.second->size();
        pointcount2 += pcs2.point_layers.at(kv1.first)->size();
    }
    ASSERT_(pointcount1 > 0 || !pcs1.planes.empty());
    ASSERT_(pointcount2 > 0 || !pcs2.planes.empty());

    // ------------------------------------------------------
    // Main ICP loop
    // ------------------------------------------------------
    ICP_State state(pcs1, pcs2);

    state.current_solution = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    auto prev_solution     = state.current_solution;

    // Prepare params for "find pairings" for each layer:
    prepareMatchingParams(state, p);

    for (result.nIterations = 0; result.nIterations < p.maxIterations;
         result.nIterations++)
    {
        ICP_iteration_result iter_result;

        // Call to algorithm-specific implementation of one ICP iteration:
        impl_ICP_iteration(state, p, iter_result);

        if (!iter_result.success)
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            result.goodness          = 0;
            break;
        }

        // Update to new solution:
        state.current_solution = iter_result.new_solution;

        // If matching has not changed, we are done:
        const auto deltaSol = state.current_solution - prev_solution;
        const mrpt::math::CVectorFixed<double, 6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.blockCopy<3, 1>(0, 0).norm();
        const double delta_rot = dSol.blockCopy<3, 1>(3, 0).norm();

#if 0
        std::cout << "Dxyz: " << std::abs(delta_xyz)
                  << " Drot:" << std::abs(delta_rot)
                  << " p: " << state.current_solution.asString() << "\n";
#endif

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        prev_solution = state.current_solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Ratio of points with a valid pairing:
    if (!state.layerOfLargestPc.empty())
        result.goodness =
            state.mres.at(state.layerOfLargestPc).correspondencesRatio;

    // Store output:
    result.optimal_tf.mean = state.current_solution;
    result.optimal_scale   = state.current_scale;
    MRPT_TODO("covariance of the estimation");
    // See: http://censi.mit.edu/pub/research/2007-icra-icpcov-slides.pdf

#else
    THROW_EXCEPTION("This method requires MP2P built against libpointmatcher")
#endif
    MRPT_END
}
