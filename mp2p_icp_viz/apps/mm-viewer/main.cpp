/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mm-viewer/main.cpp
 * @brief  GUI tool to visualize .mm (metric map) files
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2022
 */

// The goal is to visualize these guys:
#include <mp2p_icp/metricmap.h>
// using this:
#include <mrpt/imgui/CImGuiSceneView.h>

// other deps:
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_app_common/AsyncTask.h>
#include <imgui_app_common/ImGuiAppShell.h>
#include <imgui_app_common/SimpleFileDialog.h>
#include <imgui_internal.h>  // DockBuilder* API (default docking layout)
#include <imgui_stdlib.h>  // ImGui::InputText(std::string*)
#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mrpt/config.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/round.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // loadPluginModules()
#include <mrpt/system/string_utils.h>  // unitsFormat()
#include <mrpt/topography/conversions.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../libcfgpath/cfgpath.h"

namespace
{
constexpr const char* APP_NAME = "mm-viewer";

constexpr float TRAVELING_ZOOM2ROLL = 1e-4f;

// Small axis-corner gizmo viewports ("Map frame" / "ENU frame"), kept for parity with the old
// nanogui app. NOTE: with MRPT 2.x, `mrpt::imgui::CImGuiSceneView::render()` only renders the
// scene's "main" viewport, so these two extra viewports are created and kept up to date but are
// not currently visible on screen. MRPT 3.x is expected to support rendering arbitrary named
// viewports through the same mechanism, so this code is intentionally NOT removed -- it should
// start working again, unmodified, once this project moves to MRPT 3.x.
constexpr const char* FIRST_MINI_VIEW_NAME  = "small-view-1";
constexpr const char* SECOND_MINI_VIEW_NAME = "small-view-2";

const char* const kColorIntensityNames[]  = {"cmNONE", "cmHOT", "cmJET", "cmGRAYSCALE"};
const char* const kColorIntensityLabels[] = {"None", "Hot", "Jet", "Grayscale"};
constexpr int     kNumColorIntensity      = 4;

// =========== Declare supported cli switches ===========
CLI::App cmd{APP_NAME};

std::string              argMapFile;
std::string              arg_plugins;
std::string              arg_tumTrajectory;
std::vector<std::string> arg_add3dScenes;
std::string              arg_georefPolygon;

/** Extra viz layer loaded from a *.3dscene file, or from --georef-polygon. */
struct ExtraVizLayer
{
    std::string                      fileName;
    mrpt::opengl::CSetOfObjects::Ptr glObjects;
    bool                             visible = true;
};

/** Result of loadMapFileWorker(), running on a background thread: a self-contained value (no
 *  reference to AppState) so it is safe to build off the main/GL thread. Committed to AppState
 *  on the main thread once ready -- see pollMapLoad(). */
struct MapLoadResult
{
    bool                     success = false;
    std::string              errorMessage;
    std::string              mapFileName;
    mp2p_icp::metric_map_t   map;
    std::vector<std::string> layerNames;
    std::vector<std::string> knownPointFields;
};

/** All mutable application state, replacing the individual nanogui widget
 *  pointers of the previous implementation with plain value types (this is
 *  an immediate-mode GUI: widgets read/write these each frame). */
struct AppState
{
    mp2p_icp_viz::ImGuiAppShell  shell;
    mrpt::imgui::CImGuiSceneView sceneView;
    mrpt::opengl::Scene::Ptr     scene = mrpt::opengl::COpenGLScene::Create();

    mrpt::opengl::CSetOfObjects::Ptr glVizMap = mrpt::opengl::CSetOfObjects::Create();
    mrpt::opengl::CGridPlaneXY::Ptr  glGrid   = mrpt::opengl::CGridPlaneXY::Create();
    mrpt::opengl::CSetOfObjects::Ptr glENUCorner;
    mrpt::opengl::CSetOfObjects::Ptr glMapCorner;
    mrpt::opengl::CSetOfObjects::Ptr glTrajectory = mrpt::opengl::CSetOfObjects::Create();
    mrpt::opengl::CSetOfObjects::Ptr glVizObjects = mrpt::opengl::CSetOfObjects::Create();

    mp2p_icp::metric_map_t theMap;
    std::string            theMapFileName = "unnamed.mm";

    std::vector<std::string>    layerNames;
    std::vector<std::string>    knownPointFields;
    std::map<std::string, bool> layerVisible;

    std::vector<ExtraVizLayer> extraVizLayers;

    mrpt::poses::CPose3DInterpolator trajectory;

    // Camera travelling:
    mrpt::poses::CPose3DInterpolator camTravelling;
    std::optional<double>            camTravellingCurrentTime;
    std::vector<std::string>         camTravellingLabels;
    float                            animFPS             = 30.0f;
    float                            animProgress        = 0.0f;
    int                              travellingInterpIdx = 0;  // 0=Linear, 1=Spline
    float                            newKeyframeTime     = 0.0f;

    // View options (mirrors the old nanogui side panel):
    bool  applyGeoRef                = false;
    bool  viewOrtho                  = false;
    bool  view2D                     = false;
    bool  viewVoxelsAsPoints         = false;
    bool  viewVoxelsFreeSpace        = false;
    bool  colorizeMap                = true;
    bool  keepNativeCloudColors      = false;
    bool  autoBBoxOutliers           = true;
    float autoBBoxOutliersPercentile = 0.025f;
    bool  showGroundGrid             = true;
    bool  depthLogScale              = true;
    float pointSize                  = 2.0f;
    float trajectoryThicknessLog     = std::log(0.05f);
    float clipNear                   = 0.1f;
    float clipFar                    = 4.0f;
    float cameraFOV                  = 90.0f;

    int recolorizeByFieldIdx = 0;
    int colorIntensityIdx    = 2;  // cmJET
    int mouseUnitsIdx        = 0;  // map / enu / lat-lon

    bool doFitView = false;

    // Forces regeneration of the cached point-cloud OpenGL representation on the next
    // rebuild_3d_view() call, even if render_params_t happens to compare equal to the last
    // build (e.g. after loading a different map whose layers coincidentally share names,
    // visibility, and render options with the previous one).
    bool forceRebuildViz = true;

    std::string mouseCoordText = "Mouse pointing to: -";
    std::string cameraLookText = "Camera looking at: -";

    mp2p_icp_viz::SimpleFileDialog openDialog;
    mp2p_icp_viz::SimpleFileDialog exportDialog;

    // Async map loading (see loadMapFileWorker()/pollMapLoad()): keeps the file I/O off the
    // main/GL thread so the window manager doesn't consider the app "not responding" while a
    // large map file is being read.
    mp2p_icp_viz::AsyncTask<MapLoadResult> mapLoadTask;
    bool                                   isLoadingMap = false;
    std::string                            loadingFileName;

    // Bumped every time app.theMap is replaced (successfully or not). Lets an in-flight
    // vizBuildTask (started for a previous map) recognize its result is now stale once polled.
    int mapGeneration = 0;

    // Async point-cloud visualization building (see rebuild_3d_view()): same rationale, for
    // theMap.get_visualization(), which can also take a long time on large maps.
    mp2p_icp_viz::AsyncTask<mrpt::opengl::CSetOfObjects::Ptr> vizBuildTask;
    bool                                                      isBuildingViz          = false;
    int                                                       vizBuildTaskGeneration = -1;
};

AppState app;

bool load_plugins(const std::string& plugins)
{
    std::string errMsg;
    if (!mrpt::system::loadPluginModules(plugins, errMsg))
    {
        std::cerr << errMsg << std::endl;
        return false;
    }
    return true;
}

/** Reads a text file with one "lat,lon" (WGS84 degrees) vertex per line
 * (fields may be separated by a comma or by whitespace; blank lines and
 * lines starting with '#' are ignored).
 */
std::vector<mrpt::math::TPoint2D> readLatLonPolygonFile(const std::string& filePath)
{
    std::vector<mrpt::math::TPoint2D> latLonPoints;

    std::ifstream f(filePath);
    ASSERTMSG_(
        f.is_open(), mrpt::format("Cannot open georef polygon file: '%s'", filePath.c_str()));

    std::string line;
    while (std::getline(f, line))
    {
        for (char& c : line)
        {
            if (c == ',')
            {
                c = ' ';
            }
        }

        std::istringstream iss(line);
        std::string        firstTok;
        if (!(iss >> firstTok) || firstTok.empty() || firstTok[0] == '#')
        {
            continue;  // blank or comment line
        }

        double lat = 0;
        double lon = 0;
        std::istringstream(firstTok) >> lat;
        if (!(iss >> lon))
        {
            std::cerr << "Warning: malformed line in georef polygon file, skipping: \"" << line
                      << "\"\n";
            continue;
        }

        latLonPoints.emplace_back(lat, lon);
    }

    return latLonPoints;
}

/** Builds a closed polygon outline, in the metric map's local ("map") frame,
 * from a set of WGS84 lat/lon vertices, using the loaded map's
 * georeferencing information.
 */
mrpt::opengl::CSetOfObjects::Ptr buildGeorefPolygonLayer(
    const std::vector<mrpt::math::TPoint2D>&      latLonPoints,
    const mp2p_icp::metric_map_t::Georeferencing& georef)
{
    auto glLayer = mrpt::opengl::CSetOfObjects::Create();

    std::vector<mrpt::math::TPoint3D> mapPoints;
    mapPoints.reserve(latLonPoints.size());
    for (const auto& ll : latLonPoints)
    {
        const mrpt::topography::TGeodeticCoords geodeticPt(
            ll.x /*lat*/, ll.y /*lon*/, georef.geo_coord.height);

        mrpt::math::TPoint3D enuPt;
        mrpt::topography::geodeticToENU_WGS84(geodeticPt, enuPt, georef.geo_coord);

        mapPoints.push_back(georef.T_enu_to_map.mean.inverseComposePoint(enuPt));
    }

    if (mapPoints.size() >= 2)
    {
        auto glLines = mrpt::opengl::CSetOfLines::Create();
        glLines->setColor_u8(0xff, 0xd0, 0x00, 0xff);
        glLines->setLineWidth(3.0f);

        for (size_t i = 0; i < mapPoints.size(); i++)
        {
            const auto& p0 = mapPoints[i];
            const auto& p1 = mapPoints[(i + 1) % mapPoints.size()];
            glLines->appendLine(p0, p1);
        }
        glLayer->insert(glLines);
    }

    return glLayer;
}

void updateGuiAfterLoadingNewMap();

void rebuildCamTravellingLabels()
{
    app.camTravellingLabels.clear();
    for (size_t i = 0; i < app.camTravelling.size(); i++)
    {
        auto it = app.camTravelling.begin();
        std::advance(it, static_cast<std::ptrdiff_t>(i));

        app.camTravellingLabels.push_back(mrpt::format(
            "[%02u] t=%.02fs pose=%s", static_cast<unsigned int>(i),
            mrpt::Clock::toDouble(it->first), it->second.asString().c_str()));
    }
}

/** Does the actual (potentially slow: disk I/O, decompression, sanity checks) map loading work.
 *  Deliberately self-contained -- reads/writes only its local `res`, never `app.*` -- so it is
 *  safe to run on a background thread (see startMapLoad()/pollMapLoad()) instead of blocking the
 *  main/GL thread, which would otherwise starve glfwPollEvents() for large files and make the
 *  window manager conclude the app is "not responding". */
MapLoadResult loadMapFileWorker(const std::string& mapFile)
try
{
    MapLoadResult res;

    std::cout << "Loading map file: " << mapFile << std::endl;

    if (mrpt::system::extractFileExtension(mapFile) == "bin")
    {
        try
        {
            mrpt::io::CCompressedInputStream        f(mapFile);
            auto                                    arch = mrpt::serialization::archiveFrom(f);
            mrpt::serialization::CSerializable::Ptr obj  = arch.ReadObject();
            auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obj);
            if (!pts)
            {
                res.errorMessage =
                    "Error: .bin file did not deserialize to a CPointsMap-derived object" +
                    (obj ? std::string(" (got: ") + obj->GetRuntimeClass()->className + ")"
                         : std::string(" (null object)"));
                std::cerr << res.errorMessage << std::endl;
                return res;
            }
            const std::string layerName = mrpt::system::extractFileName(mapFile);
            res.map.layers[layerName]   = pts;
        }
        catch (const std::exception& e)
        {
            res.errorMessage = std::string("Error loading .bin file: ") + e.what();
            std::cerr << res.errorMessage << std::endl;
            return res;
        }
    }
    else
    {
        std::string loadErrorMsg;

        if (!res.map.load_from_file(mapFile, loadErrorMsg))
        {
            bool retry_was_successful = false;

            if (loadErrorMsg.find("which is not registered") != std::string::npos &&
                arg_plugins.empty())
            {
                std::cout
                    << "The map file requires plugins for missing C++ classes.\n"
                       "Trying to load 'libmola_metric_maps.so' and retrying.\n"
                       "Note that you can directly use '-l libmola_metric_maps.so' or any other "
                       "custom plugin next time.\n";

                if (!load_plugins("libmola_metric_maps.so"))
                {
                    res.errorMessage = "Failed to load plugin 'libmola_metric_maps.so'";
                    return res;
                }
                retry_was_successful = res.map.load_from_file(mapFile, loadErrorMsg);
            }

            if (!retry_was_successful)
            {
                res.errorMessage = "Error loading metric map from file!:\n" + loadErrorMsg;
                std::cerr << res.errorMessage << std::endl;
                return res;
            }
        }
    }

    res.mapFileName = mapFile;

    std::cout << "Loaded map: " << res.map.contents_summary() << std::endl;

    for (const auto& [name, map] : res.map.layers)
    {
        res.layerNames.push_back(name);
    }

    // Find point cloud field names:
    {
        std::set<std::string> fields;
        fields.insert("x");
        fields.insert("y");
        fields.insert("z");

        for (const auto& [name, map] : res.map.layers)
        {
            auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(map);
            if (!pts)
            {
                continue;
            }
            for (const auto& f : pts->getPointFieldNames_float())
            {
                fields.insert(std::string(f));
            }
            for (const auto& f : pts->getPointFieldNames_uint16())
            {
                fields.insert(std::string(f));
            }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            for (const auto& f : pts->getPointFieldNames_double())
            {
                fields.insert(std::string(f));
            }
            for (const auto& f : pts->getPointFieldNames_uint8())
            {
                fields.insert(std::string(f));
            }
#endif
        }

        if (fields.count("color_r") && fields.count("color_g") && fields.count("color_b"))
        {
            fields.erase("color_r");
            fields.erase("color_g");
            fields.erase("color_b");
            fields.insert("rgb");
        }
        if (fields.count("color_rf") && fields.count("color_gf") && fields.count("color_bf"))
        {
            fields.erase("color_rf");
            fields.erase("color_gf");
            fields.erase("color_bf");
            fields.insert("rgbf");
        }

        for (const auto& f : fields)
        {
            res.knownPointFields.push_back(f);
        }
    }

    // sanity checks:
    for (const auto& [name, map] : res.map.layers)
    {
        const auto* pc = mp2p_icp::MapToPointsMap(*map);
        if (!pc)
        {
            continue;  // not a point map
        }
        const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(*pc);
        ASSERTMSG_(
            sanityPassed, mrpt::format("sanity check did not pass for layer: '%s'", name.c_str()));
    }

    res.success = true;
    return res;
}
catch (const std::exception& e)
{
    MapLoadResult res;
    res.errorMessage = std::string("Error loading map: ") + e.what();
    std::cerr << res.errorMessage << std::endl;
    return res;
}

/** Starts loading `mapFile` on a background thread; see pollMapLoad() for the main-thread
 * commit. */
void startMapLoad(const std::string& mapFile)
{
    app.isLoadingMap    = true;
    app.loadingFileName = mapFile;
    app.mapLoadTask.start([mapFile]() { return loadMapFileWorker(mapFile); });
}

/** Installs a (successful or failed) load result into AppState. Must run on the main thread.
 * Does NOT call updateGuiAfterLoadingNewMap() -- callers do that explicitly, since the initial
 * CLI-arg load (before the GUI even exists) already gets it invoked once, unconditionally,
 * right after the window is set up. */
void commitMapLoadResult(MapLoadResult&& res)
{
    app.mapGeneration++;  // invalidate any in-flight vizBuildTask started for the old map

    if (!res.success)
    {
        // Leave bookkeeping consistent with "no map loaded", rather than describing whatever
        // (different) map was loaded before this attempt.
        app.theMap         = mp2p_icp::metric_map_t();
        app.theMapFileName = "unnamed.mm";
        app.layerNames.clear();
        app.knownPointFields.clear();
        app.layerVisible.clear();
        return;
    }

    app.theMap           = std::move(res.map);
    app.theMapFileName   = res.mapFileName;
    app.layerNames       = std::move(res.layerNames);
    app.knownPointFields = std::move(res.knownPointFields);
}

/** Call once per frame: if a background map load finished, commits it on the main thread. */
void pollMapLoad()
{
    if (auto res = app.mapLoadTask.poll())
    {
        app.isLoadingMap = false;
        commitMapLoadResult(std::move(*res));
        updateGuiAfterLoadingNewMap();
    }
}

/** Transform to show in the selected frame of reference and units: "map", "enu", or "lat/lon" */
std::string transformAndFormatSelectedPoint(const mrpt::math::TPoint3D& pt)
{
    const bool hasGeoref = app.theMap.georeferencing.has_value();
    const bool ptIsENU   = hasGeoref && app.applyGeoRef;
    const int  idx       = hasGeoref ? app.mouseUnitsIdx : 0;

    // `pt` is in the ENU frame when ptIsENU, otherwise it is in the "map" frame -- convert to
    // ENU once so cases 1/2 (which are only offered when hasGeoref) are correct regardless of
    // whether "Apply georeferenced pose" is currently checked.
    const mrpt::math::TPoint3D ptEnu =
        (!ptIsENU && hasGeoref) ? app.theMap.georeferencing->T_enu_to_map.mean.composePoint(pt)
                                : pt;

    switch (idx)
    {
        case 0:  // show in map
        {
            mrpt::math::TPoint3D ptViz;
            if (ptIsENU)
            {
                ptViz = app.theMap.georeferencing->T_enu_to_map.mean.inverseComposePoint(pt);
            }
            else
            {
                ptViz = pt;
            }
            return mrpt::format("X=%6.03f Y=%6.03f Z=%6.03f", ptViz.x, ptViz.y, ptViz.z);
        }
        case 1:  // show in enu
            return mrpt::format("X=%6.03f Y=%6.03f Z=%6.03f", ptEnu.x, ptEnu.y, ptEnu.z);
        case 2:  // show as lat/lon
        {
            try
            {
                mrpt::topography::TGeocentricCoords geocentricPt;
                mrpt::topography::ENUToGeocentric(
                    ptEnu, app.theMap.georeferencing->geo_coord, geocentricPt,
                    mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

                mrpt::topography::TGeodeticCoords outCoords;
                mrpt::topography::geocentricToGeodetic(geocentricPt, outCoords);

                return mrpt::format(
                    "%.06f, %.06f, h=%.03f", outCoords.lat.getDecimalValue(),
                    outCoords.lon.getDecimalValue(), outCoords.height);
            }
            catch (const std::exception& e)
            {
                std::cerr << "[transformAndFormatSelectedPoint] " << e.what() << "\n";
                return {};
            }
        }
        default:
            return {};
    }
}

void updateCameraLookCoordinates()
{
    const auto&                cam = app.sceneView.camera();
    const mrpt::math::TPoint3D pt(cam.getPointingAtX(), cam.getPointingAtY(), cam.getPointingAtZ());
    app.cameraLookText = "Camera looking at: " + transformAndFormatSelectedPoint(pt);
}

/** Wired as `sceneView.onOverlayGui`: runs right after the 3-D canvas is drawn, while it is
 * still the last ImGui item, so `ImGui::IsItemHovered()`/`GetItemRect*()` refer to it. */
void renderSceneOverlay()
{
    updateCameraLookCoordinates();

    const bool   hovered  = ImGui::IsItemHovered();
    const ImVec2 rectMin  = ImGui::GetItemRectMin();
    const ImVec2 rectSize = ImGui::GetItemRectSize();

    if (!hovered)
    {
        app.mouseCoordText = "Mouse pointing to: -";
        return;
    }

    const ImVec2 mouse  = ImGui::GetMousePos();
    const double localX = static_cast<double>(mouse.x - rectMin.x);
    const double localY = static_cast<double>(mouse.y - rectMin.y);
    if (localX < 0 || localY < 0 || localX >= rectSize.x || localY >= rectSize.y)
    {
        app.mouseCoordText = "Mouse pointing to: -";
        return;
    }

    mrpt::math::TLine3D mouseRay;
    app.scene->getViewport("main")->get3DRayForPixelCoord(localX, localY, mouseRay);

    using mrpt::math::TPoint3D;
    const mrpt::math::TPlane groundPlane(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
    mrpt::math::TObject3D    inters;
    mrpt::math::intersect(mouseRay, groundPlane, inters);
    mrpt::math::TPoint3D intersPt;
    if (inters.getPoint(intersPt))
    {
        app.mouseCoordText = "Mouse pointing to: " + transformAndFormatSelectedPoint(intersPt);
    }
}

/** Creates the "Map frame"/"ENU frame" axis-corner gizmo viewports once. See the comment next to
 * FIRST_MINI_VIEW_NAME/SECOND_MINI_VIEW_NAME above: not currently rendered under MRPT 2.x. */
void ensureMiniCornerViewports()
{
    if (!app.scene->getViewport(FIRST_MINI_VIEW_NAME))
    {
        auto gl_view = app.scene->createViewport(FIRST_MINI_VIEW_NAME);

        gl_view->setViewportPosition(0, 0, 0.1, 0.1 * 16.0 / 9.0);
        gl_view->setTransparent(true);
        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("X");
            obj->setLocation(1.1, 0, 0);
            gl_view->insert(obj);
        }
        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("Y");
            obj->setLocation(0, 1.1, 0);
            gl_view->insert(obj);
        }
        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("Z");
            obj->setLocation(0, 0, 1.1);
            gl_view->insert(obj);
        }
        gl_view->insert(mrpt::opengl::stock_objects::CornerXYZ());
    }

    if (!app.scene->getViewport(SECOND_MINI_VIEW_NAME))
    {
        auto gl_view = app.scene->createViewport(SECOND_MINI_VIEW_NAME);

        gl_view->setViewportPosition(0.1, 0, 0.1, 0.1 * 16.0 / 9.0);
        gl_view->setTransparent(true);

        auto glRoot = mrpt::opengl::CSetOfObjects::Create();
        gl_view->insert(glRoot);

        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("X");
            obj->setLocation(1.1, 0, 0);
            glRoot->insert(obj);
        }
        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("Y");
            obj->setLocation(0, 1.1, 0);
            glRoot->insert(obj);
        }
        {
            mrpt::opengl::CText::Ptr obj = mrpt::opengl::CText::Create("Z");
            obj->setLocation(0, 0, 1.1);
            glRoot->insert(obj);
        }
        glRoot->insert(mrpt::opengl::stock_objects::CornerXYZ());
    }
}

/** Keeps the mini-corner viewport cameras/labels in sync with the main camera each frame.
 * See the comment next to FIRST_MINI_VIEW_NAME/SECOND_MINI_VIEW_NAME above: not currently
 * rendered under MRPT 2.x. */
void updateMiniCornerView()
{
    auto gl_view1 = app.scene->getViewport(FIRST_MINI_VIEW_NAME);
    if (!gl_view1)
    {
        return;
    }

    mrpt::opengl::TFontParams fp;
    fp.draw_shadow = true;
    fp.vfont_scale = 9.0f;

    {
        mrpt::opengl::CCamera& view_cam = gl_view1->getCamera();
        view_cam.setAzimuthDegrees(app.sceneView.camera().getAzimuthDegrees());
        view_cam.setElevationDegrees(app.sceneView.camera().getElevationDegrees());
        view_cam.setZoomDistance(5);
    }

    const bool show_two_corners = app.applyGeoRef && app.theMap.georeferencing.has_value();

    gl_view1->clearTextMessages();
    gl_view1->addTextMessage(5, 5, show_two_corners ? "ENU frame" : "Map frame", 0, fp);

    auto gl_view2 = app.scene->getViewport(SECOND_MINI_VIEW_NAME);
    if (!gl_view2 || gl_view2->empty())
    {
        return;
    }

    auto glRoot = *gl_view2->begin();
    if (!glRoot)
    {
        return;
    }
    glRoot->setVisibility(show_two_corners);

    if (!show_two_corners)
    {
        gl_view2->clearTextMessages();
        return;
    }

    glRoot->setPose(mrpt::poses::CPose3D::FromRotationAndTranslation(
        app.theMap.georeferencing->T_enu_to_map.mean.getRotationMatrix(),
        mrpt::math::TVector3D(0, 0, 0)));

    {
        mrpt::opengl::CCamera& view_cam = gl_view2->getCamera();
        view_cam.setAzimuthDegrees(app.sceneView.camera().getAzimuthDegrees());
        view_cam.setElevationDegrees(app.sceneView.camera().getElevationDegrees());
        view_cam.setZoomDistance(5);
    }

    gl_view2->clearTextMessages();
    gl_view2->addTextMessage(5, 5, "Map frame", 0, fp);
}

void updateGuiAfterLoadingNewMap()
{
    app.forceRebuildViz = true;

    app.layerVisible.clear();
    for (const auto& name : app.layerNames)
    {
        app.layerVisible[name] = true;
    }

    for (auto& evl : app.extraVizLayers)
    {
        evl.visible = true;
    }

    app.recolorizeByFieldIdx = 0;

    // Auto-range clip sliders from map Z bounds (linear mode only):
    if (!app.depthLogScale)
    {
        float zMin = std::numeric_limits<float>::max();
        float zMax = -std::numeric_limits<float>::max();
        for (const auto& [name, map] : app.theMap.layers)
        {
            const auto* pc = mp2p_icp::MapToPointsMap(*map);
            if (!pc || pc->empty())
            {
                continue;
            }
            const auto bb = pc->boundingBox();
            zMin          = std::min(zMin, bb.min.z);
            zMax          = std::max(zMax, bb.max.z);
        }
        if (zMin < zMax)
        {
            app.clipNear = zMin;
            app.clipFar  = zMax;
        }
    }
}

void camTravellingStop() { app.camTravellingCurrentTime.reset(); }

/** Recomputed every frame so it stays in sync as the user zooms/pans (the "linear" mode needs
 * the current camera pose). */
void updateCameraClipDistances()
{
    auto& cam = app.sceneView.camera();

    float clipNear = 0;
    float clipFar  = 0;
    if (app.depthLogScale)
    {
        const auto depthFieldMid       = std::pow(10.0, app.clipNear);
        const auto depthFieldThickness = std::pow(10.0, app.clipFar);
        clipNear = static_cast<float>(std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness));
        clipFar  = static_cast<float>(depthFieldMid + 0.5 * depthFieldThickness);
    }
    else
    {
        // Linear mode: slider values are world Z coordinates.
        // Convert to frustum distances using the camera elevation and zoom.
        const float elDeg   = cam.getElevationDegrees();
        const float sinEl   = std::sin(mrpt::DEG2RAD(elDeg));
        const float cameraZ = cam.getPointingAtZ() + cam.getZoomDistance() * sinEl;

        const float orthoFactor = cam.isProjective() ? 1.0f : 2.0f;
        const float zFloor      = orthoFactor * app.clipNear;
        const float zCeiling    = orthoFactor * app.clipFar;

        if (std::abs(sinEl) > 0.05f)
        {
            clipFar  = std::max(0.01f, (cameraZ - zFloor) / sinEl);
            clipNear = std::max(0.01f, std::min(clipFar - 0.01f, (cameraZ - zCeiling) / sinEl));
        }
        else
        {
            clipNear = std::max(0.01f, std::abs(zFloor));
            clipFar  = std::max(clipNear + 0.01f, std::abs(zCeiling));
        }
    }

    cam.setProjectiveFOVdeg(app.cameraFOV);

    app.scene->getViewport("main")->setViewportClipDistances(clipNear, clipFar);
}

void processCameraTravelling()
{
    if (!app.camTravellingCurrentTime.has_value())
    {
        return;
    }
    double& t = app.camTravellingCurrentTime.value();

    const double t0  = mrpt::Clock::toDouble(app.camTravelling.begin()->first);
    const double t1  = mrpt::Clock::toDouble(app.camTravelling.rbegin()->first);
    app.animProgress = (t1 > t0) ? static_cast<float>((t - t0) / (t1 - t0)) : 0.0f;

    if (t >= t1)
    {
        camTravellingStop();
        return;
    }

    const auto interpMethod = app.travellingInterpIdx == 0
                                  ? mrpt::poses::TInterpolatorMethod::imLinear2Neig
                                  : mrpt::poses::TInterpolatorMethod::imSSLSLL;
    app.camTravelling.setInterpolationMethod(interpMethod);

    mrpt::math::TPose3D p;
    bool                valid = false;
    app.camTravelling.interpolate(mrpt::Clock::fromDouble(t), p, valid);
    if (valid)
    {
        auto& cam = app.sceneView.camera();
        cam.setPointingAt(
            static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
        cam.setAzimuthDegrees(static_cast<float>(mrpt::RAD2DEG(p.yaw)));
        cam.setElevationDegrees(static_cast<float>(mrpt::RAD2DEG(p.pitch)));
        cam.setZoomDistance(static_cast<float>(p.roll / TRAVELING_ZOOM2ROLL));
    }

    const double dt = app.animFPS > 0 ? 1.0 / app.animFPS : 1.0 / 30.0;
    t += dt;
}

void handleKeyboard()
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput)
    {
        return;
    }

    constexpr float SLIDE_VELOCITY     = 0.01f;
    constexpr float SENSIBILITY_ROTATE = 1.0f;
    auto&           cam                = app.sceneView.camera();

    auto doStrideSides = [&](bool toTheRight)
    {
        const float az   = cam.getAzimuthDegrees();
        const float dx   = std::cos(mrpt::DEG2RAD(az + 90.f));
        const float dy   = std::sin(mrpt::DEG2RAD(az + 90.f));
        const float d    = toTheRight ? 1.0f : -1.0f;
        const float dist = cam.getZoomDistance();
        cam.setPointingAt(
            cam.getPointingAtX() + d * dx * dist * SLIDE_VELOCITY,
            cam.getPointingAtY() + d * dy * dist * SLIDE_VELOCITY, cam.getPointingAtZ());
    };

    auto doRotateEyeYaw = [&](bool toTheRight)
    {
        const float dis = std::max(0.01f, cam.getZoomDistance());
        const float az0 = cam.getAzimuthDegrees();
        const float el0 = cam.getElevationDegrees();

        const float eyeX = cam.getPointingAtX() +
                           dis * std::cos(mrpt::DEG2RAD(az0)) * std::cos(mrpt::DEG2RAD(el0));
        const float eyeY = cam.getPointingAtY() +
                           dis * std::sin(mrpt::DEG2RAD(az0)) * std::cos(mrpt::DEG2RAD(el0));
        const float eyeZ = cam.getPointingAtZ() + dis * std::sin(mrpt::DEG2RAD(el0));

        const float newAz = az0 + (toTheRight ? -SENSIBILITY_ROTATE : SENSIBILITY_ROTATE);
        cam.setAzimuthDegrees(newAz);

        cam.setPointingAt(
            eyeX - dis * std::cos(mrpt::DEG2RAD(newAz)) * std::cos(mrpt::DEG2RAD(el0)),
            eyeY - dis * std::sin(mrpt::DEG2RAD(newAz)) * std::cos(mrpt::DEG2RAD(el0)),
            eyeZ - dis * std::sin(mrpt::DEG2RAD(el0)));
    };

    auto doRotateEyeUpDown = [&](bool toUp)
    {
        const float az   = cam.getAzimuthDegrees();
        const float dx   = std::cos(mrpt::DEG2RAD(az));
        const float dy   = std::sin(mrpt::DEG2RAD(az));
        const float d    = toUp ? -1.0f : 1.0f;
        const float dist = cam.getZoomDistance();
        cam.setPointingAt(
            cam.getPointingAtX() + d * dx * dist * SLIDE_VELOCITY,
            cam.getPointingAtY() + d * dy * dist * SLIDE_VELOCITY, cam.getPointingAtZ());
    };

    auto doMoveVertically = [&](bool toUp)
    {
        const float d    = toUp ? 1.0f : -1.0f;
        const float dist = cam.getZoomDistance();
        cam.setPointingAt(
            cam.getPointingAtX(), cam.getPointingAtY(),
            cam.getPointingAtZ() + d * dist * SLIDE_VELOCITY);
    };

    const bool up =
        ImGui::IsKeyPressed(ImGuiKey_UpArrow, true) || ImGui::IsKeyPressed(ImGuiKey_W, true);
    const bool down =
        ImGui::IsKeyPressed(ImGuiKey_DownArrow, true) || ImGui::IsKeyPressed(ImGuiKey_S, true);
    if (up || down)
    {
        if (io.KeyShift)
        {
            doMoveVertically(up);
        }
        else
        {
            doRotateEyeUpDown(up);
        }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_A, true))
    {
        doStrideSides(false);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_D, true))
    {
        doStrideSides(true);
    }

    const bool right = ImGui::IsKeyPressed(ImGuiKey_RightArrow, true);
    const bool left  = ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true);
    if (right || left)
    {
        if (io.KeyShift)
        {
            doStrideSides(right);
        }
        else
        {
            doRotateEyeYaw(right);
        }
    }

    // CTRL+C copies the coordinates
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C, false))
    {
        const std::string text = app.mouseCoordText + "\n" + app.cameraLookText;
        glfwSetClipboardString(app.shell.windowHandle(), text.c_str());
    }
}

void onSaveLayers(const std::string& outFile)
{
    const std::filesystem::path base(outFile);

    for (const auto& lyName : app.layerNames)
    {
        if (auto itV = app.layerVisible.find(lyName); itV == app.layerVisible.end() || !itV->second)
        {
            continue;  // not a marked (visible) layer
        }
        if (auto itL = app.theMap.layers.find(lyName); itL != app.theMap.layers.end())
        {
            // One file per layer: writing every layer to the same outFile would clobber all
            // but the last one.
            const auto perLayerFile = base.parent_path() / (base.stem().string() + "_" + lyName +
                                                            base.extension().string());
            itL->second->saveMetricMapRepresentationToFile(perLayerFile.string());
        }
    }
}

void rebuild_3d_view()
{
    std::optional<mrpt::math::TBoundingBoxf> mapBbox;

    mp2p_icp::render_params_t rpMap;
    rpMap.points.visible = false;

    for (const auto& lyName : app.layerNames)
    {
        if (auto itL = app.theMap.layers.find(lyName); itL != app.theMap.layers.end())
        {
            if (auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itL->second); pc)
            {
                const auto bb = pc->boundingBox();
                mapBbox       = mapBbox.has_value() ? mapBbox->unionWith(bb) : bb;
            }
        }

        const auto itV       = app.layerVisible.find(lyName);
        const bool isVisible = (itV == app.layerVisible.end()) ? true : itV->second;
        if (!isVisible)
        {
            continue;  // hidden
        }
        rpMap.points.visible = true;

        auto& rpL                       = rpMap.points.perLayer[lyName];
        rpL.pointSize                   = app.pointSize;
        rpL.render_voxelmaps_as_points  = app.viewVoxelsAsPoints;
        rpL.render_voxelmaps_free_space = app.viewVoxelsFreeSpace;

        if (app.colorizeMap)
        {
            auto& cm    = rpL.colorMode.emplace();
            cm.colorMap = mrpt::typemeta::str2enum<mrpt::img::TColormap>(
                kColorIntensityNames[app.colorIntensityIdx]);

            if (!app.knownPointFields.empty())
            {
                cm.recolorizeByField =
                    app.knownPointFields.at(static_cast<size_t>(app.recolorizeByFieldIdx));
            }

            if (app.autoBBoxOutliers)
            {
                cm.autoBoundingBoxOutliersPercentile = app.autoBBoxOutliersPercentile;
            }
        }
        if (app.keepNativeCloudColors)
        {
            auto& cm                     = rpL.colorMode.emplace();
            cm.keep_original_cloud_color = true;
            rpL.force_alpha_channel      = true;
        }
    }

    for (auto& [layer, rp] : rpMap.points.perLayer)
    {
        rp.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0x80);
    }

    // Regenerate points opengl representation only if some parameter changed (or a new map
    // was just loaded, regardless of whether rpMap happens to compare equal to the last one).
    // Built on a background thread (theMap.get_visualization() can take a long time on large
    // maps) so the main/GL thread keeps pumping events instead of appearing "not responding".
    static std::optional<mp2p_icp::render_params_t> prevRenderParams;

    const bool needsRebuild =
        !prevRenderParams.has_value() || prevRenderParams.value() != rpMap || app.forceRebuildViz;

    if (needsRebuild && !app.isBuildingViz)
    {
        app.forceRebuildViz = false;
        prevRenderParams    = rpMap;

        app.isBuildingViz          = true;
        app.vizBuildTaskGeneration = app.mapGeneration;

        // Shallow copy: shares the underlying (immutable, once loaded) layer CMetricMap::Ptr
        // objects, so this is cheap regardless of map size, and safe to read from the
        // background thread even if the main thread replaces app.theMap in the meantime.
        const mp2p_icp::metric_map_t mapCopy = app.theMap;
        app.vizBuildTask.start([mapCopy, rpMap]() { return mapCopy.get_visualization(rpMap); });
    }

    if (auto glPts = app.vizBuildTask.poll())
    {
        app.isBuildingViz = false;

        if (app.vizBuildTaskGeneration == app.mapGeneration)
        {
            app.glVizMap->clear();
            app.glVizMap->insert(*glPts);
            app.glVizMap->insert(app.glMapCorner);
            app.glVizMap->insert(app.glTrajectory);
            app.glVizMap->insert(app.glVizObjects);
        }
        else
        {
            // Stale: app.theMap was replaced while this build was in flight. Force a fresh
            // rebuild for the current map on the next frame.
            app.forceRebuildViz = true;
        }
    }

    if (app.applyGeoRef && app.theMap.georeferencing.has_value())
    {
        app.glVizMap->setPose(app.theMap.georeferencing->T_enu_to_map.mean);
        app.glENUCorner->setVisibility(true);
    }
    else
    {
        app.glVizMap->setPose(mrpt::poses::CPose3D::Identity());
        app.glENUCorner->setVisibility(false);
    }

    if (mapBbox)
    {
        app.glGrid->setPlaneLimits(mapBbox->min.x, mapBbox->max.x, mapBbox->min.y, mapBbox->max.y);

        constexpr float MAX_GRID_LINES = 20.0f;
        const auto      bboxDiagonal   = (mapBbox->max - mapBbox->min).cast<float>().norm();
        app.glGrid->setGridFrequency(
            std::max<float>(1.0f, std::round(bboxDiagonal / MAX_GRID_LINES)));
    }
    app.glGrid->setVisibility(app.showGroundGrid);

    if (mapBbox && app.doFitView)
    {
        const auto midPt  = (mapBbox->min + mapBbox->max) * 0.5;
        const auto mapLen = (mapBbox->max - mapBbox->min).norm();

        app.sceneView.camera().setPointingAt(midPt.x, midPt.y, midPt.z);
        app.sceneView.camera().setZoomDistance(mapLen);
    }
    app.doFitView = false;

    if (app.glTrajectory->empty() && app.trajectory.size() >= 2)
    {
        const float trajectoryCylRadius = std::exp(app.trajectoryThicknessLog);

        std::optional<mrpt::math::TPose3D> prevPose;
        for (const auto& [t, p] : app.trajectory)
        {
            if (prevPose)
            {
                const auto& p0 = prevPose.value();

                auto glSegment = mrpt::opengl::CArrow::Create();
                glSegment->setArrowEnds(p0.translation(), p.translation());
                glSegment->setHeadRatio(.0);
                glSegment->setLargeRadius(trajectoryCylRadius);
                glSegment->setSmallRadius(trajectoryCylRadius);
                glSegment->setColor_u8(0x30, 0x30, 0x30, 0xff);

                app.glTrajectory->insert(glSegment);
            }
            prevPose = p;
        }
    }

    app.glVizObjects->clear();
    for (auto& evl : app.extraVizLayers)
    {
        evl.glObjects->setVisibility(evl.visible);
        app.glVizObjects->insert(evl.glObjects);
    }

    app.sceneView.camera().setProjectiveModel(!app.viewOrtho && !app.view2D);

    if (app.view2D)
    {
        app.sceneView.camera().setAzimuthDegrees(-90.0f);
        app.sceneView.camera().setElevationDegrees(90.0f);
    }

    ensureMiniCornerViewports();

    // Clip planes/FOV are refreshed every frame in updateCameraClipDistances(),
    // since the "linear" clip mode needs the up-to-date camera pose.
}

/** "Map viewer" window: file header + Open/Export + mouse/camera coordinate footer.
 *  Kept separate from the View/Maps/Travelling panels below so it can dock as a thin strip. */
void renderMapViewerPanel()
{
    ImGui::Begin("Map viewer");

    const std::string headerLine = app.theMapFileName + "  |  " + app.theMap.contents_summary();
    ImGui::TextWrapped("%s", headerLine.c_str());

    ImGui::BeginDisabled(app.isLoadingMap);
    if (ImGui::Button("Open..."))
    {
        app.openDialog.open(
            mp2p_icp_viz::SimpleFileDialog::Mode::Open,
            {{"mm", "Metric maps (*.mm)"}, {"bin", "Serialized CGenericPointsMap (*.bin)"}},
            "Open metric map");
    }
    ImGui::SameLine();
    if (ImGui::Button("Export marked layers..."))
    {
        app.exportDialog.open(
            mp2p_icp_viz::SimpleFileDialog::Mode::Save, {{"txt", "(*.txt)"}}, "Export layers");
    }
    ImGui::EndDisabled();

    if (app.isLoadingMap)
    {
        ImGui::TextColored(
            ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Loading '%s'...", app.loadingFileName.c_str());
    }
    else if (app.isBuildingViz)
    {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Building 3D visualization...");
    }

    ImGui::Separator();
    ImGui::TextUnformatted(app.mouseCoordText.c_str());
    ImGui::TextUnformatted(app.cameraLookText.c_str());

    const bool  hasGeoref     = app.theMap.georeferencing.has_value();
    const char* itemsGeoref[] = {"map", "enu", "lat/lon"};
    const char* itemsPlain[]  = {"map"};
    if (!hasGeoref)
    {
        app.mouseUnitsIdx = 0;
    }
    ImGui::SetNextItemWidth(100);
    if (hasGeoref)
    {
        ImGui::Combo("##mouseUnits", &app.mouseUnitsIdx, itemsGeoref, 3);
    }
    else
    {
        ImGui::BeginDisabled();
        int dummy = 0;
        ImGui::Combo("##mouseUnits", &dummy, itemsPlain, 1);
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::Button("Copy coords"))
    {
        const std::string text = app.mouseCoordText + "\n" + app.cameraLookText;
        glfwSetClipboardString(app.shell.windowHandle(), text.c_str());
    }

    ImGui::End();
}

/** "View" window: camera/rendering options (mirrors the old nanogui "View" tab). */
void renderViewPanel()
{
    ImGui::Begin("View");

    ImGui::SliderFloat(
        "Trajectory thickness", &app.trajectoryThicknessLog, std::log(0.005f), std::log(2.0f));
    if (ImGui::IsItemEdited())
    {
        app.glTrajectory->clear();  // force rebuild
    }

    ImGui::Checkbox("Log scale", &app.depthLogScale);
    if (ImGui::IsItemEdited())
    {
        if (app.depthLogScale)
        {
            app.clipNear = 0.1f;
            app.clipFar  = 4.0f;
        }
        else
        {
            app.clipNear = -5.0f;
            app.clipFar  = 25.0f;
        }
    }

    if (app.depthLogScale)
    {
        ImGui::SliderFloat("Frustum center (log10 m)", &app.clipNear, -2.0f, 3.0f);
        ImGui::SliderFloat("Frustum thickness (log10 m)", &app.clipFar, -2.0f, 6.0f);
    }
    else
    {
        ImGui::SliderFloat("Floor clip (near, m)", &app.clipNear, -5.0f, 25.0f);
        ImGui::SliderFloat("Ceiling clip (far, m)", &app.clipFar, -5.0f, 25.0f);
    }

    ImGui::SliderFloat("Camera FOV", &app.cameraFOV, 20.0f, 170.0f);
    ImGui::Checkbox("Orthogonal view", &app.viewOrtho);
    ImGui::SameLine();
    ImGui::Checkbox("Force 2D view", &app.view2D);

    ImGui::Checkbox("Show ground grid", &app.showGroundGrid);
    ImGui::SameLine();
    if (ImGui::Button("Fit view to map"))
    {
        app.doFitView = true;
    }

    ImGui::TextUnformatted("Set view along axis:");
    auto viewButton = [&](const char* caption, float az, float el)
    {
        if (ImGui::Button(caption))
        {
            app.view2D = false;
            app.sceneView.camera().setAzimuthDegrees(az);
            app.sceneView.camera().setElevationDegrees(el);
        }
        ImGui::SameLine();
    };
    viewButton("+X", 0.0f, 0.0f);
    viewButton("-X", 180.0f, 0.0f);
    viewButton("+Y", 90.0f, 0.0f);
    viewButton("-Y", -90.0f, 0.0f);
    viewButton("+Z", -90.0f, 90.0f);
    viewButton("-Z", -90.0f, -90.0f);
    ImGui::NewLine();

    ImGui::BeginDisabled(!app.theMap.georeferencing.has_value());
    ImGui::Checkbox("Apply georeferenced pose (if available)", &app.applyGeoRef);
    ImGui::EndDisabled();

    ImGui::End();
}

/** "Maps" window: layer list, visibility, and colorization options. */
void renderMapsPanel()
{
    ImGui::Begin("Maps");

    ImGui::SliderFloat("Point size", &app.pointSize, 1.0f, 10.0f);
    ImGui::Checkbox("Render voxel maps as point clouds", &app.viewVoxelsAsPoints);
    ImGui::Checkbox("Render free space of voxel maps", &app.viewVoxelsFreeSpace);

    ImGui::Checkbox("Recolorize points", &app.colorizeMap);
    ImGui::SameLine();
    ImGui::TextUnformatted("by:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    if (!app.knownPointFields.empty())
    {
        const char* preview =
            app.knownPointFields.at(static_cast<size_t>(app.recolorizeByFieldIdx)).c_str();
        if (ImGui::BeginCombo("##recolorizeField", preview))
        {
            for (size_t i = 0; i < app.knownPointFields.size(); i++)
            {
                const bool sel = (static_cast<int>(i) == app.recolorizeByFieldIdx);
                if (ImGui::Selectable(app.knownPointFields[i].c_str(), sel))
                {
                    app.recolorizeByFieldIdx = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SetNextItemWidth(140);
    ImGui::Combo("##colorMap", &app.colorIntensityIdx, kColorIntensityLabels, kNumColorIntensity);

    ImGui::Checkbox("Keep native map colors", &app.keepNativeCloudColors);

    ImGui::Checkbox("Outlier percentile:", &app.autoBBoxOutliers);
    ImGui::SameLine();
    ImGui::BeginDisabled(!app.autoBBoxOutliers);
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("##outlierPercentile", &app.autoBBoxOutliersPercentile, 0.0f, 0.25f, "%.3f");
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::TextUnformatted("Visible layers:");
    ImGui::SameLine();
    if (ImGui::SmallButton("All"))
    {
        for (auto& [name, vis] : app.layerVisible)
        {
            vis = true;
        }
        for (auto& evl : app.extraVizLayers)
        {
            evl.visible = true;
        }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("None"))
    {
        for (auto& [name, vis] : app.layerVisible)
        {
            vis = false;
        }
        for (auto& evl : app.extraVizLayers)
        {
            evl.visible = false;
        }
    }

    if (ImGui::BeginChild("##layerList", ImVec2(0, 150), true))
    {
        for (const auto& lyName : app.layerNames)
        {
            std::string caption = lyName;
            if (auto itL = app.theMap.layers.find(lyName); itL != app.theMap.layers.end())
            {
                if (auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itL->second); pc)
                {
                    caption = lyName + " | " +
                              mrpt::system::unitsFormat(static_cast<double>(pc->size()), 2, false) +
                              " points | class=" + pc->GetRuntimeClass()->className;
                }
                else
                {
                    caption = lyName + " | class=" + itL->second->GetRuntimeClass()->className;
                }
            }
            ImGui::Checkbox(caption.c_str(), &app.layerVisible[lyName]);
        }
        for (auto& evl : app.extraVizLayers)
        {
            ImGui::Checkbox(("(viz) " + evl.fileName).c_str(), &evl.visible);
        }
    }
    ImGui::EndChild();

    ImGui::End();
}

/** "Travelling" window: camera keyframe animation controls. */
void renderTravellingPanel()
{
    ImGui::Begin("Travelling");

    ImGui::TextUnformatted("Define camera travelling paths");

    // Plain read-only list (not a combo): keyframes cannot currently be selected, jumped to,
    // or deleted individually, so an interactive-looking widget would be misleading.
    ImGui::TextUnformatted("Keyframes:");
    if (ImGui::BeginChild("##travellingKeys", ImVec2(0, 100), true))
    {
        for (const auto& label : app.camTravellingLabels)
        {
            ImGui::TextUnformatted(label.c_str());
        }
    }
    ImGui::EndChild();

    ImGui::InputFloat("New keyframe time [s]", &app.newKeyframeTime);
    ImGui::SameLine();
    if (ImGui::Button("Add"))
    {
        auto&      cam = app.sceneView.camera();
        const auto p   = mrpt::math::TPose3D(
              cam.getPointingAtX(), cam.getPointingAtY(), cam.getPointingAtZ(),
              mrpt::DEG2RAD(cam.getAzimuthDegrees()), mrpt::DEG2RAD(cam.getElevationDegrees()),
              cam.getZoomDistance() * TRAVELING_ZOOM2ROLL);
        app.camTravelling.insert(
            mrpt::Clock::fromDouble(static_cast<double>(app.newKeyframeTime)), p);
        rebuildCamTravellingLabels();
        app.newKeyframeTime += 1.0f;
    }

    const bool isPlaying = app.camTravellingCurrentTime.has_value();
    ImGui::BeginDisabled(isPlaying || app.camTravelling.empty());
    if (ImGui::Button("Play"))
    {
        app.camTravellingCurrentTime.emplace(
            mrpt::Clock::toDouble(app.camTravelling.begin()->first));
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!isPlaying);
    if (ImGui::Button("Stop"))
    {
        camTravellingStop();
    }
    ImGui::EndDisabled();

    if (ImGui::InputFloat("Animation FPS", &app.animFPS))
    {
        app.animFPS = std::clamp(app.animFPS, 1.0f, 240.0f);
    }

    const char* interpItems[] = {"Linear", "Spline"};
    ImGui::Combo("Interpolation", &app.travellingInterpIdx, interpItems, 2);

    ImGui::BeginDisabled(true);
    ImGui::SliderFloat("Progress", &app.animProgress, 0.0f, 1.0f);
    ImGui::EndDisabled();

    ImGui::End();
}

/** Renders both file dialogs at top level, outside any other window's Begin/End -- nesting them
 * inside e.g. "Map viewer" would silently cancel an open dialog whenever that window is
 * collapsed (BeginPopupModal() fails to start, which SimpleFileDialog::render() treats as a
 * dismiss). */
void renderFileDialogs()
{
    if (auto path = app.openDialog.render(); path.has_value())
    {
        startMapLoad(*path);
    }
    if (auto path = app.exportDialog.render(); path.has_value())
    {
        onSaveLayers(*path);
    }
}

/** "3D View" window: fills the remaining (background) dockspace area. */
void renderSceneWindow()
{
    ImGui::Begin("3D View");
    app.sceneView.render();
    ImGui::End();
}

void renderFrame()
{
    handleKeyboard();
    pollMapLoad();
    processCameraTravelling();
    rebuild_3d_view();
    updateCameraClipDistances();
    updateMiniCornerView();

    renderMapViewerPanel();
    renderViewPanel();
    renderMapsPanel();
    renderTravellingPanel();
    renderFileDialogs();
    renderSceneWindow();
}

int mainShowGui()
{
    using namespace std::string_literals;

    if (!argMapFile.empty())
    {
        // Synchronous here: no GUI window exists yet at this point (still before
        // app.shell.init() below), so there is nothing for the window manager to consider
        // "not responding" -- this only matters once a window is on screen and pumping events.
        auto res = loadMapFileWorker(argMapFile);
        if (!res.success)
        {
            return 1;
        }
        commitMapLoadResult(std::move(res));
    }

    if (!arg_georefPolygon.empty())
    {
        if (!app.theMap.georeferencing.has_value())
        {
            std::cerr << "Warning: --georef-polygon given, but the loaded map has no "
                         "georeferencing information. Ignoring it.\n";
        }
        else
        {
            try
            {
                const auto latLonPoints = readLatLonPolygonFile(arg_georefPolygon);

                ExtraVizLayer evl;
                evl.fileName = mrpt::system::extractFileName(arg_georefPolygon);
                evl.glObjects =
                    buildGeorefPolygonLayer(latLonPoints, app.theMap.georeferencing.value());
                evl.glObjects->setName(evl.fileName);

                app.extraVizLayers.push_back(evl);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Warning: could not load georef polygon: " << e.what() << "\n";
            }
        }
    }

    if (!app.shell.init(APP_NAME, 1280, 800))
    {
        return 1;
    }

    // Default docking layout (only applied once, when no imgui.ini layout exists yet):
    // "Map viewer" as a thin left strip, View/Maps/Travelling tabbed below it, and the 3D
    // scene filling the remaining (background) area.
    app.shell.setupDefaultLayout = [](unsigned int dockspaceId)
    {
        ImGuiID rightId = 0;
        ImGuiID leftId =
            ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.28f, nullptr, &rightId);

        ImGuiID leftBottomId = 0;
        ImGuiID leftTopId =
            ImGui::DockBuilderSplitNode(leftId, ImGuiDir_Up, 0.32f, nullptr, &leftBottomId);

        // "Maps" gets its own independent docked panel, separate from the View/Travelling tab
        // group.
        ImGuiID leftMapsId =
            ImGui::DockBuilderSplitNode(leftBottomId, ImGuiDir_Down, 0.45f, nullptr, &leftBottomId);

        ImGui::DockBuilderDockWindow("Map viewer", leftTopId);
        ImGui::DockBuilderDockWindow("View", leftBottomId);
        ImGui::DockBuilderDockWindow("Travelling", leftBottomId);
        ImGui::DockBuilderDockWindow("Maps", leftMapsId);
        ImGui::DockBuilderDockWindow("3D View", rightId);
    };

    // Background scene:
    app.glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
    app.scene->insert(app.glGrid);

    app.glMapCorner = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    app.glMapCorner->setName("map");
    app.glMapCorner->enableShowName();

    app.glENUCorner = mrpt::opengl::stock_objects::CornerXYZ(2.0f);
    app.glENUCorner->setName("ENU");
    app.glENUCorner->enableShowName();
    app.scene->insert(app.glENUCorner);

    app.scene->insert(app.glVizMap);

    app.sceneView.setScene(app.scene);
    app.sceneView.onOverlayGui = &renderSceneOverlay;

    app.sceneView.camera().setPointingAt(8.0f, 0.0f, 0.0f);
    app.sceneView.camera().setAzimuthDegrees(110.0f);
    app.sceneView.camera().setElevationDegrees(15.0f);
    app.sceneView.camera().setZoomDistance(50.0f);

    updateGuiAfterLoadingNewMap();
    rebuildCamTravellingLabels();

    // Load/save persistent UI+camera state across sessions (separate from imgui.ini, which
    // only remembers panel docking):
    char appCfgFile[1024];
    ::get_user_config_file(appCfgFile, sizeof(appCfgFile), APP_NAME);
    if (appCfgFile[0] == '\0')
    {
        std::cerr << "Warning: could not determine a user config file path; "
                     "UI/camera settings will not be persisted.\n";
    }
    mrpt::config::CConfigFile appCfg(appCfgFile);

    auto& cam               = app.sceneView.camera();
    app.applyGeoRef         = appCfg.read_bool("", "applyGeoRef", app.applyGeoRef);
    app.viewOrtho           = appCfg.read_bool("", "viewOrtho", app.viewOrtho);
    app.view2D              = appCfg.read_bool("", "view2D", app.view2D);
    app.viewVoxelsAsPoints  = appCfg.read_bool("", "viewVoxelsAsPoints", app.viewVoxelsAsPoints);
    app.viewVoxelsFreeSpace = appCfg.read_bool("", "viewVoxelsFreeSpace", app.viewVoxelsFreeSpace);
    app.colorizeMap         = appCfg.read_bool("", "colorizeMap", app.colorizeMap);
    app.keepNativeCloudColors =
        appCfg.read_bool("", "keepNativeCloudColors", app.keepNativeCloudColors);
    app.autoBBoxOutliers = appCfg.read_bool("", "autoBBoxOutliers", app.autoBBoxOutliers);
    app.showGroundGrid   = appCfg.read_bool("", "showGroundGrid", app.showGroundGrid);
    app.depthLogScale    = appCfg.read_bool("", "depthLogScale", app.depthLogScale);

    app.pointSize = appCfg.read_float("", "pointSize", app.pointSize);
    app.autoBBoxOutliersPercentile =
        appCfg.read_float("", "autoBBoxOutliersPercentile", app.autoBBoxOutliersPercentile);
    app.trajectoryThicknessLog =
        appCfg.read_float("", "trajectoryThicknessLog", app.trajectoryThicknessLog);
    app.clipNear  = appCfg.read_float("", "clipNear", app.clipNear);
    app.clipFar   = appCfg.read_float("", "clipFar", app.clipFar);
    app.cameraFOV = appCfg.read_float("", "cameraFOV", app.cameraFOV);

    cam.setPointingAt(
        appCfg.read_float("", "cam_x", cam.getPointingAtX()),
        appCfg.read_float("", "cam_y", cam.getPointingAtY()),
        appCfg.read_float("", "cam_z", cam.getPointingAtZ()));
    cam.setAzimuthDegrees(appCfg.read_float("", "cam_az", cam.getAzimuthDegrees()));
    cam.setElevationDegrees(appCfg.read_float("", "cam_el", cam.getElevationDegrees()));
    cam.setZoomDistance(appCfg.read_float("", "cam_d", cam.getZoomDistance()));

    app.shell.run(&renderFrame);

    appCfg.write("", "applyGeoRef", app.applyGeoRef);
    appCfg.write("", "viewOrtho", app.viewOrtho);
    appCfg.write("", "view2D", app.view2D);
    appCfg.write("", "viewVoxelsAsPoints", app.viewVoxelsAsPoints);
    appCfg.write("", "viewVoxelsFreeSpace", app.viewVoxelsFreeSpace);
    appCfg.write("", "colorizeMap", app.colorizeMap);
    appCfg.write("", "keepNativeCloudColors", app.keepNativeCloudColors);
    appCfg.write("", "autoBBoxOutliers", app.autoBBoxOutliers);
    appCfg.write("", "showGroundGrid", app.showGroundGrid);
    appCfg.write("", "depthLogScale", app.depthLogScale);

    appCfg.write("", "pointSize", app.pointSize);
    appCfg.write("", "autoBBoxOutliersPercentile", app.autoBBoxOutliersPercentile);
    appCfg.write("", "trajectoryThicknessLog", app.trajectoryThicknessLog);
    appCfg.write("", "clipNear", app.clipNear);
    appCfg.write("", "clipFar", app.clipFar);
    appCfg.write("", "cameraFOV", app.cameraFOV);

    appCfg.write("", "cam_x", cam.getPointingAtX());
    appCfg.write("", "cam_y", cam.getPointingAtY());
    appCfg.write("", "cam_z", cam.getPointingAtZ());
    appCfg.write("", "cam_az", cam.getAzimuthDegrees());
    appCfg.write("", "cam_el", cam.getElevationDegrees());
    appCfg.write("", "cam_d", cam.getZoomDistance());

    app.shell.shutdown();
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    cmd.add_option("input", argMapFile, "Load this metric map file (*.mm)");
    cmd.add_option(
        "-l,--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins");
    cmd.add_option(
        "-t,--trajectory", arg_tumTrajectory,
        "Also draw a trajectory, given by a TUM file trajectory.");
    cmd.add_option(
        "-s,--add-3d-scene", arg_add3dScenes,
        "Adds an extra 3D scene file (*.3dscene) for visualization.");
    cmd.add_option(
        "-g,--georef-polygon", arg_georefPolygon,
        "Overlay a polygon given by a text file with one \"lat,lon\" vertex (WGS84 degrees) per "
        "line. Ignored if the loaded map has no georeferencing information.");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        if (!arg_plugins.empty())
        {
            if (!load_plugins(arg_plugins))
            {
                return 1;
            }
        }

        if (!arg_tumTrajectory.empty())
        {
            ASSERT_FILE_EXISTS_(arg_tumTrajectory);
            const bool trajectoryReadOk = app.trajectory.loadFromTextFile_TUM(arg_tumTrajectory);
            ASSERT_(trajectoryReadOk);
            std::cout << "Read trajectory with " << app.trajectory.size() << " keyframes.\n";
        }

        for (const auto& path : arg_add3dScenes)
        {
            ASSERT_FILE_EXISTS_(path);
            auto scene  = mrpt::opengl::Scene::Create();
            bool readOk = scene->loadFromFile(path);
            ASSERT_(readOk);

            ExtraVizLayer evl;
            evl.fileName  = mrpt::system::extractFileName(path);
            evl.glObjects = mrpt::opengl::CSetOfObjects::Create();
            evl.glObjects->setName(evl.fileName);

            for (const auto& obj : *scene->getViewport())
            {
                evl.glObjects->insert(obj);
            }

            app.extraVizLayers.push_back(evl);
        }

        return mainShowGui();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
