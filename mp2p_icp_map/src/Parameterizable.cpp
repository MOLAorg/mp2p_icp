/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/Parameterizable.h>

using namespace mp2p_icp;

void ParameterSource::updateVariable(const std::string& variable, double value)
{
    //
}

void ParameterSource::realize()
{
    //
}

void Parameterizable::attachToParameterSource(ParameterSource& source)
{
    //
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, double& target)
{
    //
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, float& target)
{
    //
}

void Parameterizable::parseAndDeclareParameter(
    const std::string& value, uint32_t& target)
{
    //
}
