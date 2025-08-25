/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   load_plugin.h
 * @brief  Loads user-defined plugins (.so, .dll) with custom pipeline classes.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 12, 2025
 */
#pragma once

#include <mrpt/system/COutputLogger.h>

#include <string>

namespace mp2p_icp
{
/** Loads user-defined plugins (.so, .dll) with custom pipeline classes.
 *
 * The plugin must implement any of these virtual classes, and register it with
 * mrpt::rtti::registerClass():
 * - `mp2p_icp::Generator` for generators: observations to metric maps;
 * - `mp2p_icp::Matcher`
 * - `mp2p_icp::FilterBase`
 *
 * \param[in] moduleToLoad The path to the plugin library, either absolute or just the name, in
 *                         which case `LD_LIBRARY_PATH` will be searched.
 * \param[in] logger An optional instance of `mrpt::system::COutputLogger` to log error or debug
 *                   messages.
 *
 * \throws std::runtime_error if the plugin cannot be loaded.
 *
 * \ingroup mp2p_icp_grp
 */
void load_plugin(
    const std::string& moduleToLoad, const mrpt::system::COutputLogger* logger = nullptr);

}  // namespace mp2p_icp
