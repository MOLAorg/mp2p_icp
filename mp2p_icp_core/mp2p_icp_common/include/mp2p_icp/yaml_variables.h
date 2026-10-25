/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   yaml_variables.h
 * @brief  Expansion of `${VAR}` / `${VAR|default}` variables in YAML files
 * @author Jose Luis Blanco Claraco
 * @date   Oct 25, 2026
 */
#pragma once

#include <mrpt/containers/yaml.h>

#include <map>
#include <string>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_common_grp
 * @{*/

/** Replaces all `${VAR}` and `${VAR|default}` tokens in a YAML text.
 *
 * Each variable is resolved from (first match wins):
 *  1. The environment variable `VAR`.
 *  2. An entry `VAR` in `variables`.
 *  3. The inline default after `|`, which may be empty (`${VAR|}`).
 *
 * If none applies, an exception is thrown. Only `CURRENT_YAML_FILE_PATH` is
 * looked up in `variables` before the environment.
 * Tokens within YAML comments (after a `#` that starts the line or follows a
 * blank, outside quotes) are not expanded, and substituted values are not
 * scanned again for more tokens.
 *
 * Expansion works on the raw text, so tokens may appear anywhere, including
 * within quoted strings or formulas, e.g. `"robot_x - ${RANGE|10.0}"`.
 *
 * This is a subset of the YAML pre-processing in MOLA's `mola_yaml` package,
 * which also supports file includes and shell commands.
 */
[[nodiscard]] std::string expand_yaml_variables(
    const std::string& text, const std::map<std::string, std::string>& variables = {});

/** Loads a YAML file, expanding its variables with expand_yaml_variables().
 *
 * In addition to `variables`, the variable `CURRENT_YAML_FILE_PATH` is
 * always the directory of the file, overriding any variable of the same name.
 *
 * \exception std::exception If the file cannot be read, a variable is
 * undefined and has no default, or the expanded text is not valid YAML.
 */
[[nodiscard]] mrpt::containers::yaml load_yaml_file(
    const std::string& fileName, const std::map<std::string, std::string>& variables = {});

/** @} */

}  // namespace mp2p_icp
