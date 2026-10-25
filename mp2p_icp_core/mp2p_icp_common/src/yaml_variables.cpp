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
 * @file   yaml_variables.cpp
 * @brief  Expansion of `${VAR}` / `${VAR|default}` variables in YAML files
 * @author Jose Luis Blanco Claraco
 * @date   Oct 25, 2026
 */

#include <mp2p_icp/yaml_variables.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <optional>
#include <sstream>
#include <utility>

namespace
{
constexpr const char* CURRENT_YAML_FILE_PATH = "CURRENT_YAML_FILE_PATH";

/** Returns the position of the `searchEndChar` closing a bracket already open
 *  before `pos`, taking nested `otherStartChar` into account, or npos. */
[[nodiscard]] std::string::size_type findClosing(
    size_t pos, const std::string& s, const char searchEndChar, const char otherStartChar) noexcept
{
    int depth = 1;
    for (const size_t len = s.size(); pos < len; ++pos)
    {
        const char ch = s[pos];
        if (ch == otherStartChar)
        {
            ++depth;
        }
        else if (ch == searchEndChar && --depth == 0)
        {
            return pos;
        }
    }
    return std::string::npos;
}

/** Splits `"name|default"` into `{"name", default}`. The default is nullopt
 *  without `|`, so `${VAR}` (required) differs from `${VAR|}` (empty). */
[[nodiscard]] std::pair<std::string, std::optional<std::string>> splitVerticalBar(
    const std::string& s)
{
    const auto pos = s.find('|');
    if (pos == std::string::npos)
    {
        return {s, std::nullopt};
    }
    return {s.substr(0, pos), s.substr(pos + 1)};
}

/** True if `pos` lies within a YAML comment: after a `#` that starts the line
 *  or follows a blank, outside quoted scalars. Quotes are tracked from the
 *  beginning of the line only. */
[[nodiscard]] bool positionIsInComment(const std::string& text, size_t pos) noexcept
{
    const auto   prevEol   = text.find_last_of("\r\n", pos);
    const size_t lineStart = (prevEol == std::string::npos) ? 0 : prevEol + 1;

    const auto startsToken = [&](size_t i)
    { return i == lineStart || std::strchr(" \t[{,", text[i - 1]) != nullptr; };

    char quote = 0;
    for (size_t i = lineStart; i < pos; ++i)
    {
        const char ch = text[i];
        if (quote == '"')
        {
            if (ch == '\\')
            {
                ++i;
            }
            else if (ch == '"')
            {
                quote = 0;
            }
        }
        else if (quote == '\'')
        {
            if (ch == '\'')
            {
                quote = 0;
            }
        }
        else if ((ch == '"' || ch == '\'') && startsToken(i))
        {
            quote = ch;
        }
        else if (ch == '#' && startsToken(i))
        {
            return true;
        }
    }
    return false;
}
}  // namespace

std::string mp2p_icp::expand_yaml_variables(
    const std::string& text, const std::map<std::string, std::string>& variables)
{
    std::string result;
    result.reserve(text.size());
    size_t pos = 0;

    while (true)
    {
        const auto tokenStart = text.find("${", pos);
        if (tokenStart == std::string::npos)
        {
            result.append(text, pos, std::string::npos);
            break;
        }

        result.append(text, pos, tokenStart - pos);

        // Keep comments verbatim, up to the end of the line:
        if (positionIsInComment(text, tokenStart))
        {
            const auto eol = text.find_first_of("\r\n", tokenStart);
            const auto end = (eol == std::string::npos) ? text.size() : eol;
            result.append(text, tokenStart, end - tokenStart);
            pos = end;
            continue;
        }

        const size_t exprStart = tokenStart + 2;
        const auto   exprEnd   = findClosing(exprStart, text, '}', '{');
        if (exprEnd == std::string::npos)
        {
            THROW_EXCEPTION_FMT(
                "Cannot find matching `}` for `${` at position %u",
                static_cast<unsigned int>(tokenStart));
        }

        const std::string varExpr = text.substr(exprStart, exprEnd - exprStart);
        pos                       = exprEnd + 1;

        const auto [varname, defaultValue] = splitVerticalBar(varExpr);

        if (const auto it = variables.find(varname);
            varname == CURRENT_YAML_FILE_PATH && it != variables.end())
        {
            result += it->second;
        }
        else if (const char* env = ::getenv(varname.c_str()); env != nullptr)
        {
            result += env;
        }
        else if (const auto it = variables.find(varname); it != variables.end())
        {
            result += it->second;
        }
        else if (defaultValue.has_value())
        {
            result += *defaultValue;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Undefined variable `${%s}`: define it as an environment variable, or give it "
                "a default value with `${%s|default}`",
                varname.c_str(), varname.c_str());
        }
    }

    return result;
}

mrpt::containers::yaml mp2p_icp::load_yaml_file(
    const std::string& fileName, const std::map<std::string, std::string>& variables)
{
    std::ifstream f(fileName);
    if (!f.is_open())
    {
        THROW_EXCEPTION_FMT("Cannot open YAML file: '%s'", fileName.c_str());
    }
    std::stringstream ss;
    ss << f.rdbuf();

    auto vars = variables;
    vars[CURRENT_YAML_FILE_PATH] =
        mrpt::system::extractFileDirectory(mrpt::system::toAbsolutePath(fileName));

    try
    {
        return mrpt::containers::yaml::FromText(expand_yaml_variables(ss.str(), vars));
    }
    catch (const std::exception& e)
    {
        THROW_EXCEPTION_FMT("Error loading YAML file '%s': %s", fileName.c_str(), e.what());
    }
}
