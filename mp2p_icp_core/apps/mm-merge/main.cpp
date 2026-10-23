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
 * @file   mm-merge/main.cpp
 * @brief  CLI tool to merge the layers of several metric map files into one
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// CLI flags:
struct Cli
{
    CLI::App cmd{"mm-merge"};

    std::vector<std::string> argInputs;
    std::string              argOutput;
    std::string              argOnConflict = "error";
    std::vector<std::string> argRenames;
    std::string              arg_plugins;

    Cli()
    {
        cmd.add_option(
               "-i,--input", argInputs,
               "Input .mm files to merge (at least two). Repeat the flag or list several files. "
               "Non-layer data (id, label) is taken from the first input.")
            ->required();
        cmd.add_option("-o,--output", argOutput, "Output .mm file to write to")->required();
        cmd.add_option(
               "--on-conflict", argOnConflict,
               "What to do if a layer name exists in more than one input: 'error' aborts listing "
               "the conflicting layers; 'rename' appends the suffix '_<N>' to the layer of the "
               "N-th input (1-based) and keeps both.")
            ->check(CLI::IsMember({"error", "rename"}))
            ->default_val("error");
        cmd.add_option(
            "--rename", argRenames,
            "Renames a layer of one input before merging, with format \"N:OLD_NAME|NEW_NAME\", "
            "N being the 1-based index of the input file. Can be repeated.");
        cmd.add_option(
            "-l,--load-plugins", arg_plugins,
            "One or more (comma separated) *.so files to load as plugins");
    }
};

void rename_layer(
    mp2p_icp::metric_map_t& mm, const std::string& oldName, const std::string& newName)
{
    auto node  = mm.layers.extract(oldName);
    node.key() = newName;
    mm.layers.insert(std::move(node));
}

void apply_explicit_renames(const Cli& cli, std::vector<mp2p_icp::metric_map_t>& maps)
{
    for (const auto& r : cli.argRenames)
    {
        const auto colonPos = r.find(':');
        const auto pipePos  = r.find('|');
        if (colonPos == std::string::npos || pipePos == std::string::npos || pipePos < colonPos)
        {
            THROW_EXCEPTION_FMT(
                "Invalid --rename argument '%s'. Expected format: \"N:OLD_NAME|NEW_NAME\"",
                r.c_str());
        }

        const auto idxStr  = r.substr(0, colonPos);
        const auto oldName = r.substr(colonPos + 1, pipePos - colonPos - 1);
        const auto newName = r.substr(pipePos + 1);

        std::size_t idx = 0;
        try
        {
            idx = std::stoul(idxStr);
        }
        catch (const std::exception&)
        {
            THROW_EXCEPTION_FMT("Invalid input index in --rename argument '%s'", r.c_str());
        }
        if (idx < 1 || idx > maps.size())
        {
            THROW_EXCEPTION_FMT(
                "Input index in --rename argument '%s' out of range [1,%zu]", r.c_str(),
                maps.size());
        }

        auto& mm = maps.at(idx - 1);
        if (mm.layers.count(oldName) == 0)
        {
            THROW_EXCEPTION_FMT(
                "--rename '%s': input #%zu has no layer named '%s'", r.c_str(), idx,
                oldName.c_str());
        }
        if (mm.layers.count(newName) != 0)
        {
            THROW_EXCEPTION_FMT(
                "--rename '%s': input #%zu already has a layer named '%s'", r.c_str(), idx,
                newName.c_str());
        }

        std::cout << "[mm-merge] Input #" << idx << ": renaming layer '" << oldName << "' -> '"
                  << newName << "'" << std::endl;
        rename_layer(mm, oldName, newName);
    }
}

void check_georeferencing(const Cli& cli, const std::vector<mp2p_icp::metric_map_t>& maps)
{
    const mp2p_icp::metric_map_t::Georeferencing* ref    = nullptr;
    std::size_t                                   refIdx = 0;

    for (std::size_t i = 0; i < maps.size(); i++)
    {
        const auto& g = maps[i].georeferencing;
        if (!g.has_value())
        {
            continue;
        }
        if (!ref)
        {
            ref    = &g.value();
            refIdx = i;
            continue;
        }
        if (g->geo_coord != ref->geo_coord || !(g->T_enu_to_map.mean == ref->T_enu_to_map.mean))
        {
            THROW_EXCEPTION_FMT(
                "Georeferencing of input #%zu ('%s') does not match that of input #%zu ('%s'). "
                "Only maps in the same georeferenced frame can be merged.",
                i + 1, cli.argInputs.at(i).c_str(), refIdx + 1, cli.argInputs.at(refIdx).c_str());
        }
    }
}

void merge_metadata(mrpt::containers::yaml& target, const mrpt::containers::yaml& src)
{
    if (!src.isMap() || src.asMap().empty())
    {
        return;
    }
    if (!target.isMap())
    {
        target = mrpt::containers::yaml::Map();
    }

    for (const auto& [k, v] : src.asMap())
    {
        const auto key = k.as<std::string>();
        if (target.has(key))
        {
            std::cout << "[mm-merge] Warning: metadata key '" << key
                      << "' exists in more than one input, keeping the first one." << std::endl;
            continue;
        }
        target[key] = mrpt::containers::yaml(v);
    }
}

void run_mm_merge(const Cli& cli)
{
    // Load plugins:
    if (!cli.arg_plugins.empty())
    {
        std::string errMsg;
        const auto& plugins = cli.arg_plugins;
        std::cout << "Loading plugin(s): " << plugins << std::endl;
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
        {
            throw std::runtime_error(errMsg);
        }
    }

    if (cli.argInputs.size() < 2)
    {
        THROW_EXCEPTION("At least two input files must be provided with -i/--input.");
    }

    std::vector<mp2p_icp::metric_map_t> maps(cli.argInputs.size());
    for (std::size_t i = 0; i < maps.size(); i++)
    {
        const auto& fil = cli.argInputs[i];
        std::cout << "[mm-merge] Reading input #" << i + 1 << " from: '" << fil << "'..."
                  << std::endl;
        if (!maps[i].load_from_file(fil))
        {
            THROW_EXCEPTION_FMT("Error reading map file: '%s'", fil.c_str());
        }
        std::cout << "[mm-merge] Input #" << i + 1 << ": " << maps[i].contents_summary()
                  << std::endl;
    }

    apply_explicit_renames(cli, maps);
    check_georeferencing(cli, maps);

    const bool autoRename = cli.argOnConflict == "rename";

    mp2p_icp::metric_map_t merged = std::move(maps.at(0));

    for (std::size_t i = 1; i < maps.size(); i++)
    {
        auto& mm = maps[i];

        // Resolve layer name conflicts:
        std::vector<std::string> conflicts;
        for (const auto& [name, _] : mm.layers)
        {
            if (merged.layers.count(name) != 0)
            {
                conflicts.push_back(name);
            }
        }

        if (!conflicts.empty() && !autoRename)
        {
            std::string names;
            for (const auto& name : conflicts)
            {
                names += (names.empty() ? "'" : ", '") + name + "'";
            }
            THROW_EXCEPTION_FMT(
                "Input #%zu ('%s') has layer names already present in previous inputs: %s. Use "
                "'--on-conflict rename' or '--rename' to resolve them.",
                i + 1, cli.argInputs[i].c_str(), names.c_str());
        }

        for (const auto& name : conflicts)
        {
            const auto newName = name + "_" + std::to_string(i + 1);
            if (merged.layers.count(newName) != 0 || mm.layers.count(newName) != 0)
            {
                THROW_EXCEPTION_FMT(
                    "Cannot auto-rename layer '%s' of input #%zu to '%s': name already in use. "
                    "Use '--rename' instead.",
                    name.c_str(), i + 1, newName.c_str());
            }
            std::cout << "[mm-merge] Input #" << i + 1 << ": renaming conflicting layer '" << name
                      << "' -> '" << newName << "'" << std::endl;
            rename_layer(mm, name, newName);
        }

        // No layer names in common now, so this only adds new layers, lines and planes:
        merged.merge_with(mm);

        merge_metadata(merged.metadata, mm.metadata);
        if (!merged.georeferencing.has_value())
        {
            merged.georeferencing = mm.georeferencing;
        }
    }

    std::cout << "[mm-merge] Merged map: " << merged.contents_summary() << std::endl;

    std::cout << "[mm-merge] Writing output map to: '" << cli.argOutput << "'..." << std::endl;
    if (!merged.save_to_file(cli.argOutput))
    {
        THROW_EXCEPTION_FMT("Error saving output map to file: '%s'", cli.argOutput.c_str());
    }
}
}  // namespace

int main(int argc, char** argv)
{
    Cli cli;

    CLI11_PARSE(cli.cmd, argc, argv);

    try
    {
        run_mm_merge(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
