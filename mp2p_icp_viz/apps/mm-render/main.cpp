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
 * @file   mm-render/main.cpp
 * @brief  CLI tool to render metric map layers into an image file, off-screen
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
CLI::App cmd{"mm-render"};

std::string              argMapFile;
std::string              argOutput;
std::vector<std::string> argLayers;
std::string              arg_plugins;
unsigned int             argWidth     = 1920;
unsigned int             argHeight    = 1080;
double                   argAzimuth   = -45.0;
double                   argElevation = 35.0;
std::optional<double>    argDistance;
std::vector<double>      argTarget;
double                   argFov = 45.0;
std::string              argBackground{"FFFFFF"};
std::string              argColormap;
float                    argPointSize = 1.0f;
bool                     argSSAO      = false;

const std::map<std::string, mrpt::img::TColormap> kColormaps = {
    {"jet", mrpt::img::cmJET},
    {"hot", mrpt::img::cmHOT},
    {"gray", mrpt::img::cmGRAYSCALE},
};

void load_plugins()
{
    std::string errMsg;
    // Most maps with non-MRPT layer types need this one; ignore it if missing:
    mrpt::system::loadPluginModules("libmola_metric_maps.so", errMsg);

    if (arg_plugins.empty())
    {
        return;
    }
    std::cout << "[mm-render] Loading plugin(s): " << arg_plugins << std::endl;
    if (!mrpt::system::loadPluginModules(arg_plugins, errMsg))
    {
        throw std::runtime_error(errMsg);
    }
}

mrpt::img::TColorf parse_hex_color(const std::string& hex)
{
    const unsigned long c = std::stoul(hex, nullptr, 16);
    return mrpt::img::TColorf(
        static_cast<float>((c >> 16) & 0xff) / 255.f, static_cast<float>((c >> 8) & 0xff) / 255.f,
        static_cast<float>(c & 0xff) / 255.f);
}

mrpt::viz::CSetOfObjects::Ptr render_layer(const mrpt::maps::CMetricMap::Ptr& layer)
{
    auto glObj = mrpt::viz::CSetOfObjects::Create();

    // Voxel maps: occupied voxels as cubes colored by height, which reads
    // better in a still image than the default uniform color.
    if (auto vx = std::dynamic_pointer_cast<mrpt::maps::CVoxelMap>(layer); vx)
    {
        auto ov = mrpt::viz::COctoMapVoxels::Create();
        ov->setVisualizationMode(mrpt::viz::COctoMapVoxels::COLOR_FROM_HEIGHT);
        ov->colorMap(argColormap.empty() ? mrpt::img::cmJET : kColormaps.at(argColormap));
        ov->enableLights(true);
        vx->getAsOctoMapVoxels(*ov);
        ov->showVoxels(mrpt::viz::VOXEL_SET_FREESPACE, false);
        glObj->insert(ov);
        return glObj;
    }

    mp2p_icp::render_params_point_layer_t rp;
    rp.pointSize = argPointSize;
    rp.colorMode.emplace();
    if (argColormap.empty())
    {
        rp.colorMode->keep_original_cloud_color = true;
        rp.force_alpha_channel                  = false;
    }
    else
    {
        rp.colorMode->colorMap          = kColormaps.at(argColormap);
        rp.colorMode->recolorizeByField = "z";
    }
    mp2p_icp::metric_map_t::get_visualization_map_layer(*glObj, rp, layer);
    return glObj;
}

void run_mm_render()
{
    load_plugins();

    ASSERT_FILE_EXISTS_(argMapFile);
    std::cout << "[mm-render] Reading input map from: '" << argMapFile << "'..." << std::endl;

    mp2p_icp::metric_map_t mm;
    if (!mm.load_from_file(argMapFile))
    {
        throw std::runtime_error("Error reading map file: " + argMapFile);
    }

    std::vector<std::string> layers = argLayers;
    if (layers.empty())
    {
        for (const auto& [name, _] : mm.layers)
        {
            layers.push_back(name);
        }
    }

    mrpt::viz::Scene                         scene;
    std::optional<mrpt::math::TBoundingBoxf> bbox;
    for (const auto& name : layers)
    {
        const auto it = mm.layers.find(name);
        if (it == mm.layers.end())
        {
            throw std::runtime_error("Layer not found in the map: '" + name + "'");
        }
        scene.insert(render_layer(it->second));

        const auto lyBox = it->second->boundingBox();
        bbox             = bbox ? bbox->unionWith(lyBox) : lyBox;
    }

    // Camera: by default, look at the center of the rendered layers from a
    // distance that fits the whole bounding box in view.
    mrpt::math::TPoint3Df target{0, 0, 0};
    float                 diag = 10.0f;
    if (bbox)
    {
        target = (bbox->min + bbox->max) * 0.5f;
        diag   = std::max(1.0f, static_cast<float>((bbox->max - bbox->min).norm()));
    }
    if (!argTarget.empty())
    {
        target = {
            static_cast<float>(argTarget.at(0)), static_cast<float>(argTarget.at(1)),
            static_cast<float>(argTarget.at(2))};
    }

    mrpt::viz::CCamera cam;
    cam.setAzimuthDegrees(static_cast<float>(argAzimuth));
    cam.setElevationDegrees(static_cast<float>(argElevation));
    cam.setPointingAt(target.x, target.y, target.z);
    if (argFov <= 0)
    {
        cam.setOrthogonal(true);
        // The orthographic view height is half the zoom distance:
        cam.setZoomDistance(static_cast<float>(argDistance.value_or(2.0 * diag)));
    }
    else
    {
        cam.setProjectiveFOVdeg(static_cast<float>(argFov));
        const double halfFov = 0.5 * mrpt::DEG2RAD(argFov);
        cam.setZoomDistance(
            static_cast<float>(argDistance.value_or(0.4 * diag / std::tan(halfFov))));
    }

    auto vp = scene.getViewport();
    vp->setCustomBackgroundColor(parse_hex_color(argBackground));
    vp->setViewportClipDistances(0.1f, 10.0f * diag + cam.getZoomDistance());
    if (argSSAO)
    {
        vp->enableSSAO(true);
    }

    mrpt::opengl::CFBORender render(argWidth, argHeight);
    render.setCamera(cam);
    mrpt::img::CImage img(argWidth, argHeight, mrpt::img::CH_RGB);
    render.render_RGB(scene, img);

    if (!img.saveToFile(argOutput))
    {
        throw std::runtime_error("Error writing image file: " + argOutput);
    }
    std::cout << "[mm-render] Saved " << argWidth << "x" << argHeight << " image to: '" << argOutput
              << "'" << std::endl;
}
}  // namespace

int main(int argc, char** argv)
{
    cmd.add_option("input", argMapFile, "Load this metric map file (*.mm)")->required();
    cmd.add_option("-o,--output", argOutput, "Output image file (*.png, *.jpg)")->required();
    cmd.add_option(
           "-y,--layers", argLayers,
           "Comma-separated list of layers to render (default: all layers)")
        ->delimiter(',');
    cmd.add_option(
        "-l,--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins. "
        "libmola_metric_maps.so is always tried.");
    cmd.add_option("--width", argWidth, "Image width, in pixels")->capture_default_str();
    cmd.add_option("--height", argHeight, "Image height, in pixels")->capture_default_str();
    cmd.add_option("--azimuth", argAzimuth, "Camera azimuth, in degrees")->capture_default_str();
    cmd.add_option("--elevation", argElevation, "Camera elevation, in degrees")
        ->capture_default_str();
    cmd.add_option(
        "--distance", argDistance,
        "Camera distance to the target point, in meters (default: fit the layers bounding box)");
    cmd.add_option(
           "--target", argTarget,
           "Camera target point 'x,y,z', in meters (default: center of the layers bounding "
           "box)")
        ->delimiter(',')
        ->expected(3);
    cmd.add_option("--fov", argFov, "Vertical field of view, in degrees. Use 0 for orthographic")
        ->capture_default_str();
    cmd.add_option("--background", argBackground, "Background color, as a hex RRGGBB string")
        ->capture_default_str();
    cmd.add_option(
           "--colormap", argColormap,
           "Recolor points and voxels by height with this colormap (default: keep the "
           "layer colors; voxel maps use 'jet')")
        ->check(CLI::IsMember({"jet", "hot", "gray"}));
    cmd.add_option("--point-size", argPointSize, "Point size, in pixels")->capture_default_str();
    cmd.add_flag("--ssao", argSSAO, "Enable screen-space ambient occlusion");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        run_mm_render();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exit due to exception:\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
