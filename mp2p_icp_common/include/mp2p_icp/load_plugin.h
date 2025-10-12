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
