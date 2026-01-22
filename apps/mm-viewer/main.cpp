/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
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
#include <mrpt/gui/CDisplayWindowGUI.h>

// other deps:
#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // loadPluginModules()
#include <mrpt/system/string_utils.h>  // unitsFormat()
#include <mrpt/topography/conversions.h>
#include <mrpt/version.h>

#include <iostream>

#include "../libcfgpath/cfgpath.h"

constexpr const char* APP_NAME        = "mm-viewer";
constexpr int         MID_FONT_SIZE   = 14;
constexpr int         SMALL_FONT_SIZE = 13;

constexpr const char* FIRST_MINI_VIEW_NAME  = "small-view-1";
constexpr const char* SECOND_MINI_VIEW_NAME = "small-view-2";

// =========== Declare supported cli switches ===========
static TCLAP::CmdLine cmd(APP_NAME);

static TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", false, "myMap.mm", "myMap.mm", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "l", "load-plugins", "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

static TCLAP::ValueArg<std::string> arg_tumTrajectory(
    "t", "trajectory", "Also draw a trajectory, given by a TUM file trajectory.", false,
    "trajectory.tum", "trajectory.tum", cmd);

static TCLAP::MultiArg<std::string> arg_add3dScenes(
    "s", "add-3d-scene", "Adds an extra 3D scene file (*.3dscene) for visualization.", false,
    "scene.3dscene", cmd);

// =========== Declare global variables ===========
#if MRPT_HAS_NANOGUI

auto                             glVizMap = mrpt::opengl::CSetOfObjects::Create();
auto                             glGrid   = mrpt::opengl::CGridPlaneXY::Create();
mrpt::opengl::CSetOfObjects::Ptr glENUCorner;
mrpt::opengl::CSetOfObjects::Ptr glMapCorner;
mrpt::opengl::CSetOfObjects::Ptr glTrajectory;
mrpt::opengl::CSetOfObjects::Ptr glVizObjects;

mrpt::gui::CDisplayWindowGUI::Ptr win;

std::array<nanogui::TextBox*, 2> lbMapStats              = {nullptr, nullptr};
nanogui::CheckBox*               cbApplyGeoRef           = nullptr;
nanogui::CheckBox*               cbViewOrtho             = nullptr;
nanogui::CheckBox*               cbView2D                = nullptr;
nanogui::CheckBox*               cbViewVoxelsAsPoints    = nullptr;
nanogui::CheckBox*               cbViewVoxelsFreeSpace   = nullptr;
nanogui::CheckBox*               cbColorizeMap           = nullptr;
nanogui::CheckBox*               cbKeepNativeCloudColors = nullptr;
nanogui::ComboBox*               cmbColorIntensity       = nullptr;
nanogui::ComboBox*               cmbRecolorizeByField    = nullptr;
nanogui::CheckBox*               cbShowGroundGrid        = nullptr;
nanogui::Slider*                 slPointSize             = nullptr;
nanogui::Slider*                 slTrajectoryThickness   = nullptr;
nanogui::Slider*                 slMidDepthField         = nullptr;
nanogui::Slider*                 slThicknessDepthField   = nullptr;
nanogui::Slider*                 slCameraFOV             = nullptr;
nanogui::Label*                  lbCameraFOV             = nullptr;
nanogui::Label*                  lbMousePos              = nullptr;
nanogui::ComboBox*               cbMouseUnits            = nullptr;
nanogui::Button*                 btnCopyCoords           = nullptr;
nanogui::Label*                  lbCameraPointing        = nullptr;
nanogui::Label *                 lbDepthFieldValues = nullptr, *lbDepthFieldMid = nullptr,
               *lbDepthFieldThickness = nullptr, *lbPointSize = nullptr;
nanogui::Label*    lbTrajectoryThick = nullptr;
nanogui::Widget*   panelLayers       = nullptr;
nanogui::ComboBox* cbTravellingKeys  = nullptr;
nanogui::TextBox*  edAnimFPS         = nullptr;
nanogui::Slider*   slAnimProgress    = nullptr;
nanogui::Button *  btnAnimate = nullptr, *btnAnimStop = nullptr;
nanogui::ComboBox* cbTravellingInterp = nullptr;

std::vector<std::string>                  layerNames;
std::vector<std::string>                  knownPointFields;
std::map<std::string, nanogui::CheckBox*> cbLayersByName;
bool                                      doFitView = false;

mp2p_icp::metric_map_t theMap;
std::string            theMapFileName = "unnamed.mm";

// Robot path to display (optional):
mrpt::poses::CPose3DInterpolator trajectory;

// Extra viz layers from 3Dscene files (optional)
struct ExtraVizLayer
{
    std::string                      fileName;
    mrpt::opengl::CSetOfObjects::Ptr glObjects;
    nanogui::CheckBox*               checkBox = nullptr;
};
static std::vector<ExtraVizLayer> extraVizLayers;

// Camera travelling trajectory:
mrpt::poses::CPose3DInterpolator camTravelling;
std::optional<double>            camTravellingCurrentTime;

constexpr float TRAVELING_ZOOM2ROLL = 1e-4;

static void rebuild_3d_view(bool force_rebuild_view = false);
static void onSaveLayers();

namespace
{
static bool load_plugins(const std::string& plugins)
{
    std::string errMsg;
    if (!mrpt::system::loadPluginModules(plugins, errMsg))
    {
        std::cerr << errMsg << std::endl;
        return false;
    }
    return true;
}

bool loadMapFile(const std::string& mapFile)
{
    // Load one single file:
    std::cout << "Loading map file: " << mapFile << std::endl;

    std::string loadErrorMsg;

    if (!theMap.load_from_file(mapFile, loadErrorMsg))
    {
        bool retry_was_successful = false;

        // If it fails and it's because of a missing plugin, try to load plugins to make life of
        // users easier:
        if (loadErrorMsg.find("which is not registered") != std::string::npos &&
            !arg_plugins.isSet())
        {
            std::cout << "The map file requires plugins for missing C++ classes.\n"
                         "Trying to load 'libmola_metric_maps.so' and retrying.\n"
                         "Note that you can directly use '-l libmola_metric_maps.so' or any other "
                         "custom plugin next time.\n";

            if (!load_plugins("libmola_metric_maps.so"))
            {
                return false;
            }
            // Retry:
            retry_was_successful = theMap.load_from_file(mapFile, loadErrorMsg);
        }

        if (!retry_was_successful)
        {
            std::cerr << "Error loading metric map from file!:\n" << loadErrorMsg << std::endl;
            return false;
        }
    }

    theMapFileName = mapFile;

    // Obtain layer info:
    std::cout << "Loaded map: " << theMap.contents_summary() << std::endl;

    for (const auto& [name, map] : theMap.layers)
    {
        layerNames.push_back(name);
    }

    // Find point cloud field names:
    {
        std::set<std::string> fields;
        fields.insert("x");
        fields.insert("y");
        fields.insert("z");

        for (const auto& [name, map] : theMap.layers)
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

        // Handle special color channels:
        // See reference: mrpt::obs::PointCloudRecoloringParameters
        // - `rgb` for `{color_r,color_g,color_b}` (uint8_t, in range 0-255)
        // - `rgbf` for `{color_rf,color_gf,color_bf}` (float, in range 0-1)
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

        // Now convert the set into a vector:
        knownPointFields.clear();
        for (const auto& f : fields)
        {
            knownPointFields.push_back(f);
        }
    }

    // sanity checks:
    for (const auto& [name, map] : theMap.layers)
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

    return true;
}

/** Transform to show in the selected frame of reference and units: "enu", "map", or "lat/lon" */
std::string transformAndFormatSelectedPoint(const mrpt::math::TPoint3D& pt)
{
    using namespace std::string_literals;
    const bool ptIsENU = cbApplyGeoRef->checked() && theMap.georeferencing.has_value();

    ASSERT_(cbMouseUnits);

    if (theMap.georeferencing.has_value())
    {
        if (cbMouseUnits->items().size() != 3)
        {
            cbMouseUnits->setItems(std::vector<std::string>({"map", "enu", "lat/lon "}));
            cbMouseUnits->setEnabled(true);
        }
    }
    else
    {
        if (cbMouseUnits->items().size() != 1)
        {
            cbMouseUnits->setItems(std::vector<std::string>({"map"}));
            cbMouseUnits->setSelectedIndex(0);
            cbMouseUnits->setEnabled(false);
        }
    }

    switch (cbMouseUnits->selectedIndex())
    {
        case 0:  // show in map
        {
            mrpt::math::TPoint3D ptViz;
            if (ptIsENU)
            {
                ptViz = theMap.georeferencing->T_enu_to_map.mean.inverseComposePoint(pt);
            }
            else  // pt is already map
            {
                ptViz = pt;
            }
            return mrpt::format("X=%6.03f Y=%6.03f Z=%6.03f", ptViz.x, ptViz.y, ptViz.z);
        }
        break;

        case 1:  // show in enu
        {
            // ptIsENU must be true here.
            return mrpt::format("X=%6.03f Y=%6.03f Z=%6.03f", pt.x, pt.y, pt.z);
        }
        break;

        case 2:  // show as lat/lon
        {
            // pt is ENU.
            try
            {
                mrpt::topography::TGeocentricCoords geocentricPt;
                mrpt::topography::ENUToGeocentric(
                    pt, theMap.georeferencing->geo_coord, geocentricPt,
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
        break;
        default:
            THROW_EXCEPTION(
                "Unexpected selection in combo-box for mouse coordinate transformation");
            break;
    };
}

void updateMouseCoordinates()
{
    const auto mouse_xy = win->mousePos();

    mrpt::math::TLine3D mouse_ray;
    win->background_scene->getViewport("main")->get3DRayForPixelCoord(
        mouse_xy.x(), mouse_xy.y(), mouse_ray);

    // Create a 3D plane, e.g. Z=0
    using mrpt::math::TPoint3D;

    const mrpt::math::TPlane ground_plane(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
    // Intersection of the line with the plane:
    mrpt::math::TObject3D inters;
    mrpt::math::intersect(mouse_ray, ground_plane, inters);
    // Interpret the intersection as a point, if there is an intersection:
    mrpt::math::TPoint3D inters_pt;
    if (inters.getPoint(inters_pt))
    {
        lbMousePos->setCaption(
            mrpt::format("Mouse at: %s", transformAndFormatSelectedPoint(inters_pt).c_str()));
    }
}

void updateCameraLookCoordinates()
{
    const auto pt = mrpt::math::TPoint3D(
        win->camera().getCameraPointingX(), win->camera().getCameraPointingY(),
        win->camera().getCameraPointingZ());

    lbCameraPointing->setCaption(
        mrpt::format("Looking at: %s", transformAndFormatSelectedPoint(pt).c_str()));
}

void observeViewOptions()
{
    if (cbView2D->checked())
    {
        win->camera().setAzimuthDegrees(-90.0f);
        win->camera().setElevationDegrees(90.0f);
    }
}

void updateMiniCornerView()
{
    auto gl_view1 = win->background_scene->getViewport(FIRST_MINI_VIEW_NAME);
    if (!gl_view1)
    {
        return;
    }

    mrpt::opengl::TFontParams fp;
    fp.draw_shadow = true;
    fp.vfont_scale = 9.0f;

    {
        mrpt::opengl::CCamera& view_cam = gl_view1->getCamera();

        view_cam.setAzimuthDegrees(win->camera().getAzimuthDegrees());
        view_cam.setElevationDegrees(win->camera().getElevationDegrees());
        view_cam.setZoomDistance(5);
    }

    const bool show_two_corners = cbApplyGeoRef->checked() && theMap.georeferencing.has_value();

    gl_view1->addTextMessage(
        static_cast<double>(win->width()) * 0.05 - 35, 5,
        show_two_corners ? "ENU frame" : "Map frame", 0, fp);

    auto gl_view2 = win->background_scene->getViewport(SECOND_MINI_VIEW_NAME);
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
        theMap.georeferencing.value().T_enu_to_map.mean.getRotationMatrix(),
        mrpt::math::TVector3D(0, 0, 0)));

    {
        mrpt::opengl::CCamera& view_cam = gl_view2->getCamera();

        view_cam.setAzimuthDegrees(win->camera().getAzimuthDegrees());
        view_cam.setElevationDegrees(win->camera().getElevationDegrees());
        view_cam.setZoomDistance(5);
    }

    gl_view2->addTextMessage(static_cast<double>(win->width()) * 0.05 - 35, 5, "Map frame", 0, fp);
}

void updateGuiAfterLoadingNewMap()
{
    // Build layer checkboxes:
    ASSERT_(panelLayers);
    while (panelLayers->childCount())
    {  //
        panelLayers->removeChild(0);
    }

    for (size_t i = 0; i < layerNames.size(); i++)
    {
        auto cb = panelLayers->add<nanogui::CheckBox>(layerNames.at(i));
        cb->setChecked(true);
        cb->setCallback([](bool) { rebuild_3d_view(); });
        cb->setFontSize(SMALL_FONT_SIZE);

        cbLayersByName[layerNames.at(i)] = cb;
    }

    for (auto& evl : extraVizLayers)
    {
        evl.checkBox = panelLayers->add<nanogui::CheckBox>(
            "(viz) " + evl.fileName,
            [&evl](bool checked)
            {
                evl.glObjects->setVisibility(checked);
                rebuild_3d_view(false);
            });
        evl.checkBox->setFontSize(SMALL_FONT_SIZE);
        evl.checkBox->setChecked(true);
    }

    // and point cloud fields:
    ASSERT_(cmbRecolorizeByField);
    cmbRecolorizeByField->setItems(knownPointFields);
}

void rebuildCamTravellingCombo()
{
    std::vector<std::string> lst, lstShort;
    for (size_t i = 0; i < camTravelling.size(); i++)
    {
        auto it = camTravelling.begin();
        std::advance(it, i);

        lstShort.push_back(std::to_string(i));
        lst.push_back(mrpt::format(
            "[%02u] t=%.02fs pose=%s", static_cast<unsigned int>(i),
            mrpt::Clock::toDouble(it->first), it->second.asString().c_str()));
    }
    cbTravellingKeys->setItems(lst, lstShort);

    if (!lst.empty())
    {
        cbTravellingKeys->setSelectedIndex(static_cast<int>(lst.size()) - 1);
    }
    win->performLayout();
}

void onKeyboardAction(int key, int modifiers)
{
    using mrpt::DEG2RAD;

    constexpr float SLIDE_VELOCITY     = 0.01;
    constexpr float SENSIBILITY_ROTATE = 1.0;

    auto cam = win->camera().cameraParams();

    auto doStrideSides = [&](bool toTheRight)
    {
        const float dx = std::cos(DEG2RAD(cam.cameraAzimuthDeg + 90.f));
        const float dy = std::sin(DEG2RAD(cam.cameraAzimuthDeg + 90.f));
        const float d  = toTheRight ? 1.0f : -1.0f;
        cam.cameraPointingX += d * dx * cam.cameraZoomDistance * SLIDE_VELOCITY;
        cam.cameraPointingY += d * dy * cam.cameraZoomDistance * SLIDE_VELOCITY;
        win->camera().setCameraParams(cam);
    };

    auto doRotateEyeYaw = [&](bool toTheRight)
    {
        int cmd_rot  = toTheRight ? 1 : -1;
        int cmd_elev = 0;

        const float dis   = std::max(0.01f, (cam.cameraZoomDistance));
        const auto  eye_x = cam.cameraPointingX + dis * cos(DEG2RAD(cam.cameraAzimuthDeg)) *
                                                     cos(DEG2RAD(cam.cameraElevationDeg));
        const auto eye_y = cam.cameraPointingY + dis * sin(DEG2RAD(cam.cameraAzimuthDeg)) *
                                                     cos(DEG2RAD(cam.cameraElevationDeg));
        const auto eye_z = cam.cameraPointingZ + dis * sin(DEG2RAD(cam.cameraElevationDeg));

        // Orbit camera:
        float A_AzimuthDeg = -SENSIBILITY_ROTATE * static_cast<float>(cmd_rot);
        cam.cameraAzimuthDeg += A_AzimuthDeg;

        float A_ElevationDeg = SENSIBILITY_ROTATE * static_cast<float>(cmd_elev);
        cam.setElevationDeg(cam.cameraElevationDeg + A_ElevationDeg);

        // Move cameraPointing pos:
        cam.cameraPointingX = static_cast<float>(
            eye_x -
            dis * cos(DEG2RAD(cam.cameraAzimuthDeg)) * cos(DEG2RAD(cam.cameraElevationDeg)));
        cam.cameraPointingY = static_cast<float>(
            eye_y -
            dis * sin(DEG2RAD(cam.cameraAzimuthDeg)) * cos(DEG2RAD(cam.cameraElevationDeg)));
        cam.cameraPointingZ =
            static_cast<float>(eye_z - dis * sin(DEG2RAD(cam.cameraElevationDeg)));

        win->camera().setCameraParams(cam);
    };

    auto doRotateEyeUpDown = [&](bool toUp)
    {
        const float dx = std::cos(DEG2RAD(cam.cameraAzimuthDeg));
        const float dy = std::sin(DEG2RAD(cam.cameraAzimuthDeg));
        const float d  = toUp ? -1.0f : 1.0f;
        cam.cameraPointingX += d * dx * cam.cameraZoomDistance * SLIDE_VELOCITY;
        cam.cameraPointingY += d * dy * cam.cameraZoomDistance * SLIDE_VELOCITY;
        win->camera().setCameraParams(cam);
    };

    auto doMoveVertically = [&](bool toUp)
    {
        const float d = toUp ? 1.0f : -1.0f;
        cam.cameraPointingZ += d * cam.cameraZoomDistance * SLIDE_VELOCITY;
        win->camera().setCameraParams(cam);
    };

    switch (key)
    {
        case GLFW_KEY_UP:
        case GLFW_KEY_DOWN:
        case GLFW_KEY_S:
        case GLFW_KEY_W:
        {
            if (modifiers & GLFW_MOD_SHIFT)
            {
                // Move vertically:
                doMoveVertically(key == GLFW_KEY_UP || key == GLFW_KEY_W);
            }
            else
            {
                // Rotate:
                doRotateEyeUpDown(key == GLFW_KEY_UP || key == GLFW_KEY_W);
            }
        }
        break;

        case GLFW_KEY_A:
        case GLFW_KEY_D:
        {
            doStrideSides(key == GLFW_KEY_D);
        }
        break;

        case GLFW_KEY_RIGHT:
        case GLFW_KEY_LEFT:
        {
            if (modifiers & GLFW_MOD_SHIFT)
            {
                // Strafe:
                doStrideSides(key == GLFW_KEY_RIGHT);
            }
            else
            {
                // Rotate:
                doRotateEyeYaw(key == GLFW_KEY_RIGHT);
            }
        }
        break;

        // CTRL+C copies the coordinates
        case GLFW_KEY_C:
        {
            if (btnCopyCoords && (modifiers & GLFW_MOD_CONTROL))
            {
                // this seems to need to be done from the nanogui thread.
                btnCopyCoords->callback()();
            }
        }
        break;

        default:  // Do nothing
            break;
    };
}

void camTravellingStop()
{
    camTravellingCurrentTime.reset();
    btnAnimate->setEnabled(true);
    btnAnimStop->setEnabled(false);
}

void processCameraTravelling()
{
    if (!camTravellingCurrentTime.has_value())
    {
        return;
    }
    double& t = camTravellingCurrentTime.value();

    // Time range:
    const double t0 = mrpt::Clock::toDouble(camTravelling.begin()->first);
    const double t1 = mrpt::Clock::toDouble(camTravelling.rbegin()->first);
    slAnimProgress->setRange(std::make_pair<float>(static_cast<float>(t0), static_cast<float>(t1)));
    slAnimProgress->setValue(static_cast<float>(t));

    if (t >= t1)
    {
        camTravellingStop();
        return;
    }

    // Interpolate camera params:
    const auto interpMethod = cbTravellingInterp->selectedIndex() == 0
                                  ? mrpt::poses::TInterpolatorMethod::imLinear2Neig
                                  : mrpt::poses::TInterpolatorMethod::imSSLSLL;
    camTravelling.setInterpolationMethod(interpMethod);

    mrpt::math::TPose3D p;
    bool                valid = false;
    camTravelling.interpolate(mrpt::Clock::fromDouble(t), p, valid);
    if (valid)
    {
        const auto pt = p.translation().cast<float>();
        win->camera().setCameraPointing(pt.x, pt.y, pt.z);
        win->camera().setAzimuthDegrees(static_cast<float>(mrpt::RAD2DEG(p.yaw)));
        win->camera().setElevationDegrees(static_cast<float>(mrpt::RAD2DEG(p.pitch)));
        win->camera().setZoomDistance(static_cast<float>(p.roll / TRAVELING_ZOOM2ROLL));
    }

    // Move to next time step:
    const double FPS = std::stod(edAnimFPS->value());
    ASSERT_(FPS > 0);
    const double dt = 1.0 / FPS;
    t += dt;
}

void main_show_gui()
{
    using namespace std::string_literals;

    if (argMapFile.isSet())
    {
        if (!loadMapFile(argMapFile.getValue()))
        {
            return;
        }
    }

    // Get user app config file
    char appCfgFile[1024];
    ::get_user_config_file(appCfgFile, sizeof(appCfgFile), APP_NAME);
    mrpt::config::CConfigFile appCfg(appCfgFile);

    /*
     * -------------------------------------------------------------------
     * GUI
     * --------------------------------------------------------------------
     */
    nanogui::init();

    mrpt::gui::CDisplayWindowGUI_Params cp;
    cp.maximized = true;
    win          = mrpt::gui::CDisplayWindowGUI::Create(APP_NAME, 1024, 800, cp);

    // Add a background scene:
    auto scene = mrpt::opengl::COpenGLScene::Create();
    {
        glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
        scene->insert(glGrid);
    }

    glMapCorner = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    glMapCorner->setName("map");
    glMapCorner->enableShowName();

    glTrajectory = mrpt::opengl::CSetOfObjects::Create();
    glVizObjects = mrpt::opengl::CSetOfObjects::Create();

    glENUCorner = mrpt::opengl::stock_objects::CornerXYZ(2.0f);
    glENUCorner->setName("ENU");
    glENUCorner->enableShowName();
    scene->insert(glENUCorner);

    scene->insert(glVizMap);

    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->background_scene = std::move(scene);
    }

    // Control GUI sub-window:
    auto w = win->createManagedSubWindow("Map viewer");
    w->setPosition({5, 25});
    w->requestFocus();
    w->setLayout(
        new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
    w->setFixedWidth(350);

    for (size_t i = 0; i < lbMapStats.size(); i++)
    {
        auto& lb = lbMapStats[i];

        if (i == 0)
        {
            auto pn = w->add<nanogui::Widget>();
            pn->setLayout(
                new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill));
            lb = pn->add<nanogui::TextBox>("  ");
            lb->setFixedWidth(300);
            auto btnLoad = pn->add<nanogui::Button>("", ENTYPO_ICON_ARCHIVE);
            btnLoad->setCallback(
                []()
                {
                    try
                    {
                        const auto fil =
                            nanogui::file_dialog({{"mm", "Metric maps (*.mm)"}}, false);
                        if (fil.empty())
                        {
                            return;
                        }

                        loadMapFile(fil);
                        updateGuiAfterLoadingNewMap();
                        win->performLayout();
                        rebuild_3d_view();
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << e.what() << std::endl;
                    }
                });
        }
        else
        {
            lb = w->add<nanogui::TextBox>("  ");
        }

        lb->setFontSize(MID_FONT_SIZE);
        lb->setAlignment(nanogui::TextBox::Alignment::Left);
        lb->setEditable(true);
    }

    //
    w->add<nanogui::Label>(" ");  // separator

    auto tabWidget = w->add<nanogui::TabWidget>();

    auto* tab1 = tabWidget->createTab("View");
    tab1->setLayout(new nanogui::GroupLayout());

    auto* tab2 = tabWidget->createTab("Maps");
    tab2->setLayout(
        new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill));

    auto* tab3 = tabWidget->createTab("Travelling");
    tab3->setLayout(
        new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill));

    tabWidget->setActiveTab(0);

    // --------------------------------------------------------------
    // Tab: Map layers
    // --------------------------------------------------------------
    tab2->add<nanogui::Label>("Render options:");

    {
        auto pn = tab2->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        lbPointSize = pn->add<nanogui::Label>("Point size");
        lbPointSize->setFontSize(MID_FONT_SIZE);

        slPointSize = pn->add<nanogui::Slider>();
        slPointSize->setRange({1.0f, 10.0f});
        slPointSize->setValue(2.0f);
        slPointSize->setCallback([&](float) { rebuild_3d_view(); });
    }

    cbViewVoxelsAsPoints = tab2->add<nanogui::CheckBox>("Render voxel maps as point clouds");
    cbViewVoxelsAsPoints->setFontSize(MID_FONT_SIZE);
    cbViewVoxelsAsPoints->setChecked(false);
    cbViewVoxelsAsPoints->setCallback([&](bool) { rebuild_3d_view(); });

    cbViewVoxelsFreeSpace = tab2->add<nanogui::CheckBox>("Render free space of voxel maps");
    cbViewVoxelsFreeSpace->setFontSize(MID_FONT_SIZE);
    cbViewVoxelsFreeSpace->setChecked(false);
    cbViewVoxelsFreeSpace->setCallback([&](bool) { rebuild_3d_view(); });

    {
        auto pn = tab2->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 4, nanogui::Alignment::Fill));

        cbColorizeMap = pn->add<nanogui::CheckBox>("Recolorize points");
        cbColorizeMap->setFontSize(MID_FONT_SIZE);
        cbColorizeMap->setChecked(true);
        cbColorizeMap->setCallback([&](bool) { rebuild_3d_view(); });

        auto lb = pn->add<nanogui::Label>("by:");
        lb->setFontSize(MID_FONT_SIZE);

        cmbRecolorizeByField = pn->add<nanogui::ComboBox>();
        cmbRecolorizeByField->setItems({"intensity"});
        cmbRecolorizeByField->setFontSize(MID_FONT_SIZE);
        cmbRecolorizeByField->setSelectedIndex(0);
        cmbRecolorizeByField->setCallback([&](int) { rebuild_3d_view(true); });

        cmbColorIntensity = pn->add<nanogui::ComboBox>();
        cmbColorIntensity->setItems(
            {"cmNONE", "cmHOT", "cmJET", "cmGRAYSCALE"}, {"None", "Hot", "Jet", "Grayscale"});
        cmbColorIntensity->setFontSize(MID_FONT_SIZE);
        cmbColorIntensity->setSelectedIndex(2);
        cmbColorIntensity->setCallback([&](int) { rebuild_3d_view(true); });
    }

    cbKeepNativeCloudColors = tab2->add<nanogui::CheckBox>("Keep native map colors");
    cbKeepNativeCloudColors->setFontSize(MID_FONT_SIZE);
    cbKeepNativeCloudColors->setChecked(false);
    cbKeepNativeCloudColors->setCallback([&](bool) { rebuild_3d_view(); });

    tab2->add<nanogui::Label>(" ");
    {
        auto pn = tab2->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 4, nanogui::Alignment::Fill));
        pn->add<nanogui::Label>("Visible layers:");

        auto* btnCheckAll = pn->add<nanogui::Button>("", ENTYPO_ICON_CHECK);
        btnCheckAll->setFontSize(SMALL_FONT_SIZE);
        btnCheckAll->setCallback(
            []()
            {
                for (auto& [name, cb] : cbLayersByName)
                {
                    cb->setChecked(true);
                }
                for (const auto& evl : extraVizLayers)
                {
                    evl.checkBox->setChecked(true);
                }
                rebuild_3d_view();
            });

        auto* btnCheckNone = pn->add<nanogui::Button>("", ENTYPO_ICON_CIRCLE_WITH_CROSS);
        btnCheckNone->setFontSize(SMALL_FONT_SIZE);
        btnCheckNone->setCallback(
            []()
            {
                for (auto& [name, cb] : cbLayersByName)
                {
                    cb->setChecked(false);
                }
                for (const auto& evl : extraVizLayers)
                {
                    evl.checkBox->setChecked(false);
                }
                rebuild_3d_view();
            });
    }

    panelLayers = tab2->add<nanogui::Widget>();
    panelLayers->setLayout(
        new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill));

    {
        tab2->add<nanogui::Label>(" ");  // separator
        auto btnSave = tab2->add<nanogui::Button>("Export marked layers...");
        btnSave->setFontSize(MID_FONT_SIZE);
        btnSave->setCallback([]() { onSaveLayers(); });
    }

    // --------------------------------------------------------------
    // Tab: View
    // --------------------------------------------------------------
    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        lbTrajectoryThick = pn->add<nanogui::Label>("Trajectory thickness:");
        lbTrajectoryThick->setFontSize(MID_FONT_SIZE);

        slTrajectoryThickness = pn->add<nanogui::Slider>();
        slTrajectoryThickness->setEnabled(trajectory.size() >= 2);
        slTrajectoryThickness->setRange({std::log(0.005f), std::log(2.0f)});
        slTrajectoryThickness->setValue(std::log(0.05f));
        slTrajectoryThickness->setCallback(
            [&](float)
            {
                glTrajectory->clear();  // force rebuild
                rebuild_3d_view();
            });
    }

    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        lbDepthFieldMid = pn->add<nanogui::Label>("Center depth clip plane:");
        lbDepthFieldMid->setFontSize(MID_FONT_SIZE);

        slMidDepthField = pn->add<nanogui::Slider>();
        slMidDepthField->setRange({-2.0, 3.0});
        slMidDepthField->setValue(1.0f);
        slMidDepthField->setCallback([&](float) { rebuild_3d_view(); });
    }

    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        lbDepthFieldThickness = pn->add<nanogui::Label>("Max-Min depth thickness:");
        lbDepthFieldThickness->setFontSize(MID_FONT_SIZE);

        slThicknessDepthField = pn->add<nanogui::Slider>();
        slThicknessDepthField->setRange({-2.0, 6.0});
        slThicknessDepthField->setValue(3.0);
        slThicknessDepthField->setCallback([&](float) { rebuild_3d_view(); });
    }
    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        lbCameraFOV = pn->add<nanogui::Label>("Camera FOV:");
        lbCameraFOV->setFontSize(MID_FONT_SIZE);
        slCameraFOV = pn->add<nanogui::Slider>();
        slCameraFOV->setRange({20.0f, 170.0f});
        slCameraFOV->setValue(90.0f);
        slCameraFOV->setCallback([&](float) { rebuild_3d_view(); });
    }
    lbDepthFieldValues = tab1->add<nanogui::Label>(" ");
    lbDepthFieldValues->setFontSize(MID_FONT_SIZE);

    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        cbViewOrtho = pn->add<nanogui::CheckBox>("Orthogonal view");
        cbViewOrtho->setFontSize(MID_FONT_SIZE);
        cbViewOrtho->setCallback([&](bool) { rebuild_3d_view(); });
        cbViewOrtho->setFontSize(MID_FONT_SIZE);

        cbView2D = pn->add<nanogui::CheckBox>("Force 2D view");
        cbView2D->setFontSize(MID_FONT_SIZE);
        cbView2D->setCallback([&](bool) { rebuild_3d_view(); });
    }

    {
        auto pn = tab1->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        cbShowGroundGrid = pn->add<nanogui::CheckBox>("Show ground grid");
        cbShowGroundGrid->setFontSize(MID_FONT_SIZE);
        cbShowGroundGrid->setChecked(true);
        cbShowGroundGrid->setCallback([&](bool) { rebuild_3d_view(); });

        auto btnFitView = pn->add<nanogui::Button>("Fit view to map");
        btnFitView->setFontSize(MID_FONT_SIZE);
        btnFitView->setCallback(
            [&]()
            {
                doFitView = true;
                rebuild_3d_view();
            });
    }

    cbApplyGeoRef = tab1->add<nanogui::CheckBox>("Apply georeferenced pose (if available)");
    cbApplyGeoRef->setFontSize(MID_FONT_SIZE);
    cbApplyGeoRef->setCallback([&](bool) { rebuild_3d_view(); });

    // --------------------------------------------------------------
    // Tab: Travelling
    // --------------------------------------------------------------
    tab3->add<nanogui::Label>("Define camera travelling paths");

    {
        auto pn = tab3->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        auto lb = pn->add<nanogui::Label>("Keyframes:");
        lb->setFontSize(MID_FONT_SIZE);

        cbTravellingKeys = tab3->add<nanogui::ComboBox>();
        cbTravellingKeys->setFontSize(MID_FONT_SIZE);
    }
    rebuildCamTravellingCombo();

    tab3->add<nanogui::Label>("");

    nanogui::TextBox* edTime;
    {
        auto pn = tab3->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill));

        pn->add<nanogui::Label>("New keyframe:")->setFontSize(MID_FONT_SIZE);

        edTime = pn->add<nanogui::TextBox>();
        edTime->setAlignment(nanogui::TextBox::Alignment::Left);
        edTime->setValue("0.0");
        edTime->setEditable(true);
        edTime->setPlaceholder("Time for this keyframe [s]");
        edTime->setFormat("[0-9\\.]*");
        edTime->setFontSize(MID_FONT_SIZE);

        auto btnAdd = pn->add<nanogui::Button>("Add", ENTYPO_ICON_ADD_TO_LIST);
        btnAdd->setFontSize(MID_FONT_SIZE);
        btnAdd->setCallback(
            [edTime]()
            {
                const auto p = mrpt::math::TPose3D(
                    win->camera().cameraParams().cameraPointingX,
                    win->camera().cameraParams().cameraPointingY,
                    win->camera().cameraParams().cameraPointingZ,
                    mrpt::DEG2RAD(win->camera().cameraParams().cameraAzimuthDeg),
                    mrpt::DEG2RAD(win->camera().cameraParams().cameraElevationDeg),
                    win->camera().cameraParams().cameraZoomDistance * TRAVELING_ZOOM2ROLL);
                camTravelling.insert(mrpt::Clock::fromDouble(std::stod(edTime->value())), p);
                rebuildCamTravellingCombo();

                edTime->setValue(std::to_string(std::stod(edTime->value()) + 1));
            });
    }

    tab3->add<nanogui::Label>("");

    {
        auto pn = tab3->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill));

        pn->add<nanogui::Label>("Playback:");
        btnAnimate = pn->add<nanogui::Button>("", ENTYPO_ICON_CONTROLLER_PLAY);
        btnAnimate->setFontSize(MID_FONT_SIZE);
        btnAnimate->setCallback(
            []()
            {
                if (camTravelling.empty())
                {
                    return;
                }
                camTravellingCurrentTime.emplace(
                    mrpt::Clock::toDouble(camTravelling.begin()->first));
                btnAnimate->setEnabled(false);
                btnAnimStop->setEnabled(true);
            });
        btnAnimStop = pn->add<nanogui::Button>("", ENTYPO_ICON_CIRCLE_WITH_CROSS);
        btnAnimStop->setFontSize(MID_FONT_SIZE);
        btnAnimStop->setEnabled(false);
        btnAnimStop->setCallback([]() { camTravellingStop(); });
    }

    {
        auto pn = tab3->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        auto lb = pn->add<nanogui::Label>("Animation FPS:");
        lb->setFontSize(MID_FONT_SIZE);

        edAnimFPS = pn->add<nanogui::TextBox>("30.0");
        edAnimFPS->setFontSize(MID_FONT_SIZE);
        edAnimFPS->setFormat("[0-9\\.]*");
        edAnimFPS->setEditable(true);
    }
    {
        auto pn = tab3->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

        auto lb = pn->add<nanogui::Label>("Interpolation:");
        lb->setFontSize(MID_FONT_SIZE);

        cbTravellingInterp = pn->add<nanogui::ComboBox>();
        cbTravellingInterp->setFontSize(MID_FONT_SIZE);
        cbTravellingInterp->setItems({"Linear", "Spline"});
        cbTravellingInterp->setSelectedIndex(0);
    }
    slAnimProgress = tab3->add<nanogui::Slider>();
    slAnimProgress->setEnabled(false);

    // ---

    {
        auto pn = w->add<nanogui::Widget>();
        pn->setLayout(
            new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill));

        auto pnLeft = pn->add<nanogui::Widget>();
        pnLeft->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Vertical, 2, nanogui::Alignment::Middle));

        auto pnRight = pn->add<nanogui::Widget>();
        pnRight->setLayout(
            new nanogui::GridLayout(nanogui::Orientation::Vertical, 2, nanogui::Alignment::Fill));

        lbMousePos = pnLeft->add<nanogui::Label>("Mouse pointing to:");
        lbMousePos->setFontSize(SMALL_FONT_SIZE);
        lbMousePos->setFixedWidth(280);
        lbCameraPointing = pnLeft->add<nanogui::Label>("Camera looking at:");
        lbCameraPointing->setFontSize(SMALL_FONT_SIZE);
        lbCameraPointing->setFixedWidth(280);

        cbMouseUnits =
            pnRight->add<nanogui::ComboBox>(std::vector<std::string>({"map", "ENU", "lat/lon "}));
        cbMouseUnits->setTooltip("Reference frame and format of coordinates");
        cbMouseUnits->setFontSize(SMALL_FONT_SIZE);
        cbMouseUnits->setFixedHeight(30);

        btnCopyCoords = pnRight->add<nanogui::Button>("", ENTYPO_ICON_COPY);
        btnCopyCoords->setTooltip("Copy coordinates to clipboard (CTRL+C)");
        btnCopyCoords->setCallback(
            [&]()
            {
                glfwSetClipboardString(
                    win->glfwWindow(),
                    (lbMousePos->caption() + std::string("\n") + lbCameraPointing->caption())
                        .c_str());
            });
    }

    w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
        ->setCallback([]() { win->setVisible(false); });

    win->setKeyboardCallback(
        [&](int key, [[maybe_unused]] int scancode, int action, int modifiers)
        {
            if (action != GLFW_PRESS && action != GLFW_REPEAT)
            {
                return false;
            }
            onKeyboardAction(key, modifiers);
            return false;
        });

    updateGuiAfterLoadingNewMap();

    // --------------------------------------------------------------
    // ^^^^^^^^ GUI definition done ^^^^^^^^
    // --------------------------------------------------------------
    win->performLayout();
    win->camera().setCameraPointing(8.0f, .0f, .0f);
    win->camera().setAzimuthDegrees(110.0f);
    win->camera().setElevationDegrees(15.0f);
    win->camera().setZoomDistance(50.0f);

    // save and load UI state:
#define LOAD_CB_STATE(CB_NAME__) do_cb(CB_NAME__, #CB_NAME__)
#define SAVE_CB_STATE(CB_NAME__) appCfg.write("", #CB_NAME__, (CB_NAME__)->checked())

#define LOAD_SL_STATE(SL_NAME__) do_sl(SL_NAME__, #SL_NAME__)
#define SAVE_SL_STATE(SL_NAME__) appCfg.write("", #SL_NAME__, (SL_NAME__)->value())

    auto load_UI_state_from_user_config = [&]()
    {
        auto do_cb = [&](nanogui::CheckBox* cb, const std::string& name)
        { cb->setChecked(appCfg.read_bool("", name, cb->checked())); };
        auto do_sl = [&](nanogui::Slider* sl, const std::string& name)
        { sl->setValue(appCfg.read_float("", name, sl->value())); };

        LOAD_CB_STATE(cbApplyGeoRef);
        LOAD_CB_STATE(cbViewOrtho);
        LOAD_CB_STATE(cbView2D);
        LOAD_CB_STATE(cbViewVoxelsAsPoints);
        LOAD_CB_STATE(cbViewVoxelsFreeSpace);
        LOAD_CB_STATE(cbColorizeMap);
        LOAD_CB_STATE(cbKeepNativeCloudColors);
        LOAD_CB_STATE(cbShowGroundGrid);

        LOAD_SL_STATE(slPointSize);
        LOAD_SL_STATE(slTrajectoryThickness);
        LOAD_SL_STATE(slMidDepthField);
        LOAD_SL_STATE(slThicknessDepthField);
        LOAD_SL_STATE(slCameraFOV);

        win->camera().setCameraPointing(
            appCfg.read_float("", "cam_x", win->camera().getCameraPointingX()),
            appCfg.read_float("", "cam_y", win->camera().getCameraPointingY()),
            appCfg.read_float("", "cam_z", win->camera().getCameraPointingZ()));
        win->camera().setAzimuthDegrees(
            appCfg.read_float("", "cam_az", win->camera().getAzimuthDegrees()));
        win->camera().setElevationDegrees(
            appCfg.read_float("", "cam_el", win->camera().getElevationDegrees()));
        win->camera().setZoomDistance(
            appCfg.read_float("", "cam_d", win->camera().getZoomDistance()));
    };
    auto save_UI_state_to_user_config = [&]()
    {
        SAVE_CB_STATE(cbApplyGeoRef);
        SAVE_CB_STATE(cbViewOrtho);
        SAVE_CB_STATE(cbView2D);
        SAVE_CB_STATE(cbViewVoxelsAsPoints);
        SAVE_CB_STATE(cbViewVoxelsFreeSpace);
        SAVE_CB_STATE(cbColorizeMap);
        SAVE_CB_STATE(cbKeepNativeCloudColors);
        SAVE_CB_STATE(cbShowGroundGrid);

        SAVE_SL_STATE(slPointSize);
        SAVE_SL_STATE(slTrajectoryThickness);
        SAVE_SL_STATE(slMidDepthField);
        SAVE_SL_STATE(slThicknessDepthField);
        SAVE_SL_STATE(slCameraFOV);

        appCfg.write("", "cam_x", win->camera().getCameraPointingX());
        appCfg.write("", "cam_y", win->camera().getCameraPointingY());
        appCfg.write("", "cam_z", win->camera().getCameraPointingZ());
        appCfg.write("", "cam_az", win->camera().getAzimuthDegrees());
        appCfg.write("", "cam_el", win->camera().getElevationDegrees());
        appCfg.write("", "cam_d", win->camera().getZoomDistance());
    };

    // load UI state from last session:
    load_UI_state_from_user_config();

    // Build 3D:
    rebuild_3d_view();

    // Main loop
    // ---------------------
    win->drawAll();
    win->setVisible(true);

    win->addLoopCallback(
        [&]()
        {
            updateMouseCoordinates();
            updateCameraLookCoordinates();
            observeViewOptions();
            updateMiniCornerView();
            processCameraTravelling();
        });

    nanogui::mainloop(1000 /*idleLoopPeriod ms*/, 25 /* minRepaintPeriod ms */);

    nanogui::shutdown();

    // save UI state:
    save_UI_state_to_user_config();
}

}  // namespace

// ==============================
// rebuild_3d_view
// ==============================
void rebuild_3d_view(bool force_rebuild_view)
{
    using namespace std::string_literals;

    lbMapStats[0]->setValue(theMapFileName);
    lbMapStats[1]->setValue("Map: "s + theMap.contents_summary());

    cbApplyGeoRef->setEnabled(theMap.georeferencing.has_value());

    // 3D objects -------------------
    std::optional<mrpt::math::TBoundingBoxf> mapBbox;

    // the map:
    mp2p_icp::render_params_t rpMap;

    rpMap.points.visible = false;
    for (const auto& [lyName, cb] : cbLayersByName)
    {
        // Update stats in the cb label:
        cb->setCaption(lyName);  // default
        if (auto itL = theMap.layers.find(lyName); itL != theMap.layers.end())
        {
            if (auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itL->second); pc)
            {
                cb->setCaption(
                    lyName + " | "s +
                    mrpt::system::unitsFormat(static_cast<double>(pc->size()), 2, false) +
                    " points"s + " | class="s + pc->GetRuntimeClass()->className);

                const auto bb = pc->boundingBox();
                if (!mapBbox.has_value())
                {
                    mapBbox = bb;
                }
                else
                {
                    mapBbox = mapBbox->unionWith(bb);
                }
            }
            else
            {
                cb->setCaption(lyName + " | class="s + itL->second->GetRuntimeClass()->className);
            }
        }

        // show/hide:
        if (!cb->checked())
        {
            continue;  // hidden
        }
        rpMap.points.visible = true;

        auto& rpL                       = rpMap.points.perLayer[lyName];
        rpL.pointSize                   = slPointSize->value();
        rpL.render_voxelmaps_as_points  = cbViewVoxelsAsPoints->checked();
        rpL.render_voxelmaps_free_space = cbViewVoxelsFreeSpace->checked();

        lbPointSize->setCaption("Point size: " + std::to_string(rpL.pointSize));

        if (cbColorizeMap->checked())
        {
            auto& cm    = rpL.colorMode.emplace();
            cm.colorMap = mrpt::typemeta::str2enum<mrpt::img::TColormap>(
                cmbColorIntensity->items().at(cmbColorIntensity->selectedIndex()));

            cm.recolorizeByField =
                cmbRecolorizeByField->items().at(cmbRecolorizeByField->selectedIndex());
        }
        if (cbKeepNativeCloudColors->checked())
        {
            auto& cm                     = rpL.colorMode.emplace();
            cm.keep_original_cloud_color = true;
            rpL.force_alpha_channel      = true;
        }
    }

    // Default color:
    for (auto& [layer, rp] : rpMap.points.perLayer)
    {
        rp.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0x80);
    }

    // Regenerate points opengl representation only if some parameter changed:
    static std::optional<mp2p_icp::render_params_t> prevRenderParams;

    if (!prevRenderParams.has_value() || prevRenderParams.value() != rpMap || force_rebuild_view)
    {
        prevRenderParams = rpMap;
        glVizMap->clear();

        auto glPts = theMap.get_visualization(rpMap);

        // Show all or selected layers:
        rpMap.points.allLayers.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        glVizMap->insert(glPts);
        glVizMap->insert(glMapCorner);
        glVizMap->insert(glTrajectory);
        glVizMap->insert(glVizObjects);
    }

    if (cbApplyGeoRef->checked() && theMap.georeferencing.has_value())
    {
        glVizMap->setPose(theMap.georeferencing->T_enu_to_map.mean);
        glGrid->setPose(theMap.georeferencing->T_enu_to_map.mean);
        glENUCorner->setVisibility(true);
    }
    else
    {
        glVizMap->setPose(mrpt::poses::CPose3D::Identity());
        glGrid->setPose(mrpt::poses::CPose3D::Identity());
        glENUCorner->setVisibility(false);
    }

    // ground grid:
    if (mapBbox)
    {
        glGrid->setPlaneLimits(mapBbox->min.x, mapBbox->max.x, mapBbox->min.y, mapBbox->max.y);

        constexpr float MAX_GRID_LINES = 20.0f;

        const auto bboxDiagonal = (mapBbox->max - mapBbox->min).cast<float>().norm();
        glGrid->setGridFrequency(std::max<float>(1.0f, std::round(bboxDiagonal / MAX_GRID_LINES)));
    }
    glGrid->setVisibility(cbShowGroundGrid->checked());

    // Fit view to map:
    if (mapBbox && doFitView)
    {
        const auto midPt  = (mapBbox->min + mapBbox->max) * 0.5;
        const auto mapLen = (mapBbox->max - mapBbox->min).norm();

        win->camera().setCameraPointing(midPt.x, midPt.y, midPt.z);
        win->camera().setZoomDistance(mapLen);
    }
    doFitView = false;

    // glTrajectory:
    if (glTrajectory->empty() && trajectory.size() >= 2)
    {
        const float trajectoryCylRadius = std::exp(slTrajectoryThickness->value());
        lbTrajectoryThick->setCaption(
            "Trajectory thickness: " + std::to_string(trajectoryCylRadius));

        std::optional<mrpt::math::TPose3D> prevPose;
        for (const auto& [t, p] : trajectory)
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

                glTrajectory->insert(glSegment);
            }
            prevPose = p;
        }
    }

    // glVizObjects
    glVizObjects->clear();
    for (auto& evl : extraVizLayers)
    {
        glVizObjects->insert(evl.glObjects);
    }

    // XYZ corner overlay viewports: one for "Map", another for "ENU"
    if (!win->background_scene->getViewport(FIRST_MINI_VIEW_NAME))
    {
        auto gl_view = win->background_scene->createViewport(FIRST_MINI_VIEW_NAME);

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
    if (!win->background_scene->getViewport(SECOND_MINI_VIEW_NAME))
    {
        auto gl_view = win->background_scene->createViewport(SECOND_MINI_VIEW_NAME);

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

    // Global view options:
    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->camera().setCameraProjective(!cbViewOrtho->checked() && !cbView2D->checked());

        // clip planes:
        const auto depthFieldMid       = std::pow(10.0, slMidDepthField->value());
        const auto depthFieldThickness = std::pow(10.0, slThicknessDepthField->value());

        const auto clipNear =
            static_cast<float>(std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness));
        const auto clipFar = static_cast<float>(depthFieldMid + 0.5 * depthFieldThickness);

        const float cameraFOV = slCameraFOV->value();
        win->camera().setCameraFOV(cameraFOV);
        win->camera().setMaximumZoom(std::max<float>(1000.0f, 3.0f * clipFar));

        lbDepthFieldValues->setCaption(
            mrpt::format("Depth field: near=%g far=%g", clipNear, clipFar));
        lbDepthFieldMid->setCaption(mrpt::format("Frustum center: %.03g", depthFieldMid));
        lbDepthFieldThickness->setCaption(
            mrpt::format("Frustum thickness: %.03g", depthFieldThickness));

        lbDepthFieldValues->setCaption(
            mrpt::format("Frustum: near=%.02g far=%.02g", clipNear, clipFar));

        lbCameraFOV->setCaption(mrpt::format("Camera FOV: %.02f deg", cameraFOV));

        win->background_scene->getViewport()->setViewportClipDistances(clipNear, clipFar);
    }
}

void onSaveLayers()
{
    const std::string outFile = nanogui::file_dialog({{"txt", "(*.txt)"}}, true /*save*/);
    if (outFile.empty())
    {
        return;
    }

    for (const auto& [lyName, cb] : cbLayersByName)
    {
        if (auto itL = theMap.layers.find(lyName); itL != theMap.layers.end())
        {
            itL->second->saveMetricMapRepresentationToFile(outFile);
        }
    }
}

#else  // MRPT_HAS_NANOGUI
static void main_show_gui()
{
    THROW_EXCEPTION(
        "This application requires a version of MRPT built with nanogui "
        "support.");
}

#endif  // MRPT_HAS_NANOGUI

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        // Load plugins:
        if (arg_plugins.isSet())
        {
            if (!load_plugins(arg_plugins.getValue()))
            {
                return 1;
            }
        }

        // load trajectory?
        if (arg_tumTrajectory.isSet())
        {
            ASSERT_FILE_EXISTS_(arg_tumTrajectory.getValue());
            bool trajectoryReadOk = trajectory.loadFromTextFile_TUM(arg_tumTrajectory.getValue());
            ASSERT_(trajectoryReadOk);
            std::cout << "Read trajectory with " << trajectory.size() << " keyframes.\n";
        }

        // load 3D files:
        for (const auto& path : arg_add3dScenes.getValue())
        {
            ASSERT_FILE_EXISTS_(path);
            auto scene  = mrpt::opengl::Scene::Create();
            bool readOk = scene->loadFromFile(path);
            ASSERT_(readOk);

            ExtraVizLayer evl;
            evl.fileName  = mrpt::system::extractFileName(path);
            evl.glObjects = mrpt::opengl::CSetOfObjects::Create();
            evl.glObjects->setName(evl.fileName);

            // Move all objects from the loaded scene into our set
            for (const auto& obj : *scene->getViewport())
            {
                evl.glObjects->insert(obj);
            }

            extraVizLayers.push_back(evl);
        }

        main_show_gui();

        // Clean up OpenGL objects before closing the window:
        glVizMap.reset();
        glGrid.reset();
        glENUCorner.reset();
        glMapCorner.reset();
        glTrajectory.reset();
        // then, the window:
        win.reset();

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n" << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
