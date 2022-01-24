/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   icp_pipeline_from_yaml.cpp
 * @brief  Loads and setup an ICP pipeline from a YAML configuration file
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2020
 */

#include <mp2p_icp/icp_pipeline_from_yaml.h>

using namespace mp2p_icp;

std::tuple<mp2p_icp::ICP::Ptr, mp2p_icp::Parameters>
    mp2p_icp::icp_pipeline_from_yaml(
        const mrpt::containers::yaml&       icpParams,
        const mrpt::system::VerbosityLevel& vLevel)
{
    MRPT_START

    // ICP algorithm class:
    const std::string icpClassName = icpParams["class_name"].as<std::string>();

    auto icp = std::dynamic_pointer_cast<mp2p_icp::ICP>(
        mrpt::rtti::classFactory(icpClassName));
    if (!icp)
        THROW_EXCEPTION_FMT(
            "Could not instantiate ICP algorithm named '%s'",
            icpClassName.c_str());

    icp->setVerbosityLevel(vLevel);

    // Special derived-classes for library wrappers:
    bool isDerived = false;
    if (icpParams.has("derived"))
    {
        icp->initialize_derived(icpParams["derived"]);
        isDerived = true;
    }

    // ICP solver class:
    if (icpParams.has("solvers")) icp->initialize_solvers(icpParams["solvers"]);

    // ICP matchers class:
    if (icpParams.has("matchers"))
        icp->initialize_matchers(icpParams["matchers"]);

    // ICP quality class:
    ASSERT_(icpParams.has("quality"));
    icp->initialize_quality_evaluators(icpParams["quality"]);

    // ICP parameters:
    Parameters params;
    if (!isDerived)
    {
        ASSERT_(icpParams.has("params"));
        params.load_from(icpParams["params"]);
    }

    return {icp, params};
    MRPT_END
}
