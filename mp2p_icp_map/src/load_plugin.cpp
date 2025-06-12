/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   load_plugin.cpp
 * @brief  Loads user-defined plugins (.so, .dll) with custom pipeline classes.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 12, 2025
 */

#include <mp2p_icp/load_plugin.h>
#include <mrpt/core/get_env.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <filesystem>
#include <string>
namespace fs = std::filesystem;

#if defined(__unix__)
#include <dlfcn.h>
#endif

namespace
{
void safe_add_to_list(const std::string& path, std::vector<std::string>& lst)
{
    if (mrpt::system::directoryExists(path))
    {
        lst.push_back(path);
    }
}

void from_env_var_to_list(
    const std::string& env_var_name, std::vector<std::string>& lst,
    const std::string& subStringPattern = {})
{
#if defined(_WIN32)
    const auto delim = std::string(";");
#else
    const auto delim = std::string(":");
#endif

    const auto               additionalPaths = mrpt::get_env<std::string>(env_var_name);
    std::vector<std::string> pathList;
    mrpt::system::tokenize(additionalPaths, delim, pathList);

    // Append to list:
    for (const auto& path : pathList)
    {
        if (!subStringPattern.empty() && path.find(subStringPattern) == std::string::npos)
        {
            continue;
        }
        safe_add_to_list(path, lst);
    }
}
}  // namespace

void mp2p_icp::load_plugin(
    const std::string& moduleToLoad, const mrpt::system::COutputLogger* logger)
{
    std::string absPath;

    if (!fs::path(moduleToLoad).is_relative())
    {
        // already absolute:
        ASSERT_FILE_EXISTS_(moduleToLoad);
        absPath = moduleToLoad;
    }
    else
    {
        // search for it:
        std::vector<std::string> lstPath;
        from_env_var_to_list("LD_LIBRARY_PATH", lstPath);

        for (const auto& p : lstPath)
        {
            const auto fp = mrpt::system::pathJoin({p, moduleToLoad});
            if (mrpt::system::fileExists(fp))
            {
                absPath = fp;
                break;
            }
        }
        if (absPath.empty())
        {
            THROW_EXCEPTION_FMT(
                "Could not find '%s' anywhere under the LD_LIBRARY_PATH paths",
                moduleToLoad.c_str());
        }
    }

#if defined(__unix__)
    // Check if already loaded?
    {
        void* handle = dlopen(absPath.c_str(), RTLD_NOLOAD);
        if (handle != nullptr)
        {
            return;  // skip. already loaded.
        }
    }
    void* handle = dlopen(absPath.c_str(), RTLD_LAZY);

#else
    HMODULE handle = LoadLibrary(absPath.c_str());
#endif
    if (handle == nullptr)
    {
        const char* err = dlerror();
        if (!err)
        {
            err = "(error calling dlerror())";
        }
        THROW_EXCEPTION(
            mrpt::format("Error loading module: `%s`\ndlerror(): `%s`", absPath.c_str(), err));
    }

    if (logger)
    {
        logger->logFmt(
            mrpt::system::LVL_DEBUG, "Successfully loaded plugin module: `%s`", absPath.c_str());
    }
}
