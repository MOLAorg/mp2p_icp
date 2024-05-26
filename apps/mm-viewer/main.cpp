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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // loadPluginModules()
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <iostream>

#include "../libcfgpath/cfgpath.h"

constexpr const char* APP_NAME        = "mm-viewer";
constexpr int         MID_FONT_SIZE   = 14;
constexpr int         SMALL_FONT_SIZE = 13;

// =========== Declare supported cli switches ===========
static TCLAP::CmdLine cmd(APP_NAME);

static TCLAP::UnlabeledValueArg<std::string> argMapFile(
    "input", "Load this metric map file (*.mm)", false, "myMap.mm", "myMap.mm",
    cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "l", "load-plugins",
    "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

// =========== Declare global variables ===========
#if MRPT_HAS_NANOGUI

auto glVizMap = mrpt::opengl::CSetOfObjects::Create();
auto glGrid   = mrpt::opengl::CGridPlaneXY::Create();
mrpt::opengl::CSetOfObjects::Ptr glENUCorner, glMapCorner;

mrpt::gui::CDisplayWindowGUI::Ptr win;

std::array<nanogui::TextBox*, 2> lbMapStats                = {nullptr, nullptr};
nanogui::CheckBox*               cbApplyGeoRef             = nullptr;
nanogui::CheckBox*               cbViewOrtho               = nullptr;
nanogui::CheckBox*               cbView2D                  = nullptr;
nanogui::CheckBox*               cbViewVoxelsAsPoints      = nullptr;
nanogui::CheckBox*               cbViewVoxelsFreeSpace     = nullptr;
nanogui::CheckBox*               cbColorizeMap             = nullptr;
nanogui::CheckBox*               cbKeepOriginalCloudColors = nullptr;
nanogui::CheckBox*               cbShowGroundGrid          = nullptr;
nanogui::Slider*                 slPointSize               = nullptr;
nanogui::Slider*                 slMidDepthField           = nullptr;
nanogui::Slider*                 slThicknessDepthField     = nullptr;
nanogui::Slider*                 slCameraFOV               = nullptr;
nanogui::Label*                  lbCameraFOV               = nullptr;
nanogui::Label*                  lbMousePos                = nullptr;
nanogui::Label *lbDepthFieldValues = nullptr, *lbDepthFieldMid = nullptr,
               *lbDepthFieldThickness = nullptr, *lbPointSize = nullptr;

std::vector<std::string>                  layerNames;
std::map<std::string, nanogui::CheckBox*> cbLayersByName;

mp2p_icp::metric_map_t theMap;
std::string            theMapFileName = "unnamed.mm";

static void rebuild_3d_view();
static void onSaveLayers();

namespace
{
void loadMapFile(const std::string& mapFile)
{
    // Load one single file:
    std::cout << "Loading map file: " << mapFile << std::endl;

    if (!theMap.load_from_file(mapFile))
    {
        std::cerr << "Error loading metric map from file!" << std::endl;
        return;
    }
    theMapFileName = mapFile;

    // Obtain layer info:
    std::cout << "Loaded map: " << theMap.contents_summary() << std::endl;

    for (const auto& [name, map] : theMap.layers) layerNames.push_back(name);
}

void updateMouseCoordinates()
{
    const auto mousexY = win->mousePos();

    mrpt::math::TLine3D mouse_ray;
    win->background_scene->getViewport("main")->get3DRayForPixelCoord(
        mousexY.x(), mousexY.y(), mouse_ray);

    // Create a 3D plane, e.g. Z=0
    using mrpt::math::TPoint3D;

    const mrpt::math::TPlane ground_plane(
        TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
    // Intersection of the line with the plane:
    mrpt::math::TObject3D inters;
    mrpt::math::intersect(mouse_ray, ground_plane, inters);
    // Interpret the intersection as a point, if there is an intersection:
    mrpt::math::TPoint3D inters_pt;
    if (inters.getPoint(inters_pt))
    {
        lbMousePos->setCaption(mrpt::format(
            "Mouse pointing to: X=%6.03f Y=%6.03f", inters_pt.x, inters_pt.y));
    }
}

void updateMiniCornerView()
{
    auto gl_view = win->background_scene->getViewport("small-view");
    if (!gl_view) return;

    mrpt::opengl::CCamera& view_cam = gl_view->getCamera();

    view_cam.setAzimuthDegrees(win->camera().getAzimuthDegrees());
    view_cam.setElevationDegrees(win->camera().getElevationDegrees());
    view_cam.setZoomDistance(5);
}

void main_show_gui()
{
    using namespace std::string_literals;

    if (argMapFile.isSet()) { loadMapFile(argMapFile.getValue()); }

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
    win = mrpt::gui::CDisplayWindowGUI::Create(APP_NAME, 1024, 800, cp);

    // Add a background scene:
    auto scene = mrpt::opengl::COpenGLScene::Create();
    {
        glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
        scene->insert(glGrid);
    }

    glMapCorner = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    glMapCorner->setName("map");
    glMapCorner->enableShowName();

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
    {
        auto w = win->createManagedSubWindow("Map viewer");
        w->setPosition({5, 25});
        w->requestFocus();
        w->setLayout(new nanogui::BoxLayout(
            nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
        w->setFixedWidth(350);

        for (auto& lb : lbMapStats)
        {
            lb = w->add<nanogui::TextBox>("  ");
            lb->setFontSize(MID_FONT_SIZE);
            lb->setAlignment(nanogui::TextBox::Alignment::Left);
            lb->setEditable(true);
        }

        //
        w->add<nanogui::Label>(" ");  // separator

        auto tabWidget = w->add<nanogui::TabWidget>();

        auto* tab1 = tabWidget->createTab("View");
        tab1->setLayout(new nanogui::GroupLayout());

        auto* tab2 = tabWidget->createTab("Layers");
        tab2->setLayout(new nanogui::GroupLayout());

        tabWidget->setActiveTab(0);

        tab2->add<nanogui::Label>("Visible layers:");

        for (size_t i = 0; i < layerNames.size(); i++)
        {
            auto cb = tab2->add<nanogui::CheckBox>(layerNames.at(i));
            cb->setChecked(true);
            cb->setCallback([](bool) { rebuild_3d_view(); });
            cb->setFontSize(SMALL_FONT_SIZE);

            cbLayersByName[layerNames.at(i)] = cb;
        }

        {
            tab2->add<nanogui::Label>(" ");  // separator
            auto btnSave =
                tab2->add<nanogui::Button>("Export marked layers...");
            btnSave->setFontSize(MID_FONT_SIZE);
            btnSave->setCallback([]() { onSaveLayers(); });
        }

        // tab
        {
            auto pn = tab1->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            lbPointSize = pn->add<nanogui::Label>("Point size");
            lbPointSize->setFontSize(MID_FONT_SIZE);

            slPointSize = pn->add<nanogui::Slider>();
            slPointSize->setRange({1.0f, 10.0f});
            slPointSize->setValue(2.0f);
            slPointSize->setCallback([&](float) { rebuild_3d_view(); });
        }

        {
            auto pn = tab1->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            lbDepthFieldMid =
                pn->add<nanogui::Label>("Center depth clip plane:");
            lbDepthFieldMid->setFontSize(MID_FONT_SIZE);

            slMidDepthField = pn->add<nanogui::Slider>();
            slMidDepthField->setRange({-2.0, 3.0});
            slMidDepthField->setValue(1.0f);
            slMidDepthField->setCallback([&](float) { rebuild_3d_view(); });
        }

        {
            auto pn = tab1->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            lbDepthFieldThickness =
                pn->add<nanogui::Label>("Max-Min depth thickness:");
            lbDepthFieldThickness->setFontSize(MID_FONT_SIZE);

            slThicknessDepthField = pn->add<nanogui::Slider>();
            slThicknessDepthField->setRange({-2.0, 6.0});
            slThicknessDepthField->setValue(3.0);
            slThicknessDepthField->setCallback([&](float)
                                               { rebuild_3d_view(); });
        }
        lbDepthFieldValues = tab1->add<nanogui::Label>(" ");
        lbDepthFieldValues->setFontSize(MID_FONT_SIZE);

        {
            auto pn = tab1->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            lbCameraFOV = pn->add<nanogui::Label>("Camera FOV:");
            lbCameraFOV->setFontSize(MID_FONT_SIZE);
            slCameraFOV = pn->add<nanogui::Slider>();
            slCameraFOV->setRange({20.0f, 170.0f});
            slCameraFOV->setValue(90.0f);
            slCameraFOV->setCallback([&](float) { rebuild_3d_view(); });
        }

        lbMousePos = tab1->add<nanogui::Label>("Mouse pointing to:");
        lbMousePos->setFontSize(MID_FONT_SIZE);

        cbViewOrtho = tab1->add<nanogui::CheckBox>("Orthogonal view");
        cbViewOrtho->setFontSize(MID_FONT_SIZE);
        cbViewOrtho->setCallback([&](bool) { rebuild_3d_view(); });
        cbViewOrtho->setFontSize(MID_FONT_SIZE);

        cbView2D = tab1->add<nanogui::CheckBox>("Force 2D view");
        cbView2D->setFontSize(MID_FONT_SIZE);
        cbView2D->setCallback([&](bool) { rebuild_3d_view(); });

        cbViewVoxelsAsPoints =
            tab1->add<nanogui::CheckBox>("Render voxel maps as point clouds");
        cbViewVoxelsAsPoints->setFontSize(MID_FONT_SIZE);
        cbViewVoxelsAsPoints->setChecked(false);
        cbViewVoxelsAsPoints->setCallback([&](bool) { rebuild_3d_view(); });

        cbViewVoxelsFreeSpace =
            tab1->add<nanogui::CheckBox>("Render free space of voxel maps");
        cbViewVoxelsFreeSpace->setFontSize(MID_FONT_SIZE);
        cbViewVoxelsFreeSpace->setChecked(false);
        cbViewVoxelsFreeSpace->setCallback([&](bool) { rebuild_3d_view(); });

        cbColorizeMap = tab1->add<nanogui::CheckBox>("Recolorize map points");
        cbColorizeMap->setFontSize(MID_FONT_SIZE);
        cbColorizeMap->setChecked(true);
        cbColorizeMap->setCallback([&](bool) { rebuild_3d_view(); });

        cbKeepOriginalCloudColors =
            tab1->add<nanogui::CheckBox>("Keep original cloud colors");
        cbKeepOriginalCloudColors->setFontSize(MID_FONT_SIZE);
        cbKeepOriginalCloudColors->setChecked(false);
        cbKeepOriginalCloudColors->setCallback([&](bool)
                                               { rebuild_3d_view(); });

        cbShowGroundGrid = tab1->add<nanogui::CheckBox>("Show ground grid");
        cbShowGroundGrid->setFontSize(MID_FONT_SIZE);
        cbShowGroundGrid->setChecked(true);
        cbShowGroundGrid->setCallback([&](bool) { rebuild_3d_view(); });

        cbApplyGeoRef = tab1->add<nanogui::CheckBox>(
            "Apply georeferenced pose (if available)");
        cbApplyGeoRef->setFontSize(MID_FONT_SIZE);
        cbApplyGeoRef->setCallback([&](bool) { rebuild_3d_view(); });

        // ----
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
            ->setCallback([]() { win->setVisible(false); });

        win->setKeyboardCallback(
            [&](int key, [[maybe_unused]] int scancode, int action,
                [[maybe_unused]] int modifiers)
            {
                if (action != GLFW_PRESS && action != GLFW_REPEAT) return false;

                switch (key)
                {
                    // case GLFW_KEY_LEFT: xxx; break;
                };

                return false;
            });
    }

    win->performLayout();
    win->camera().setCameraPointing(8.0f, .0f, .0f);
    win->camera().setAzimuthDegrees(110.0f);
    win->camera().setElevationDegrees(15.0f);
    win->camera().setZoomDistance(50.0f);

    // save and load UI state:
#define LOAD_CB_STATE(CB_NAME__) do_cb(CB_NAME__, #CB_NAME__)
#define SAVE_CB_STATE(CB_NAME__) \
    appCfg.write("", #CB_NAME__, CB_NAME__->checked())

#define LOAD_SL_STATE(SL_NAME__) do_sl(SL_NAME__, #SL_NAME__)
#define SAVE_SL_STATE(SL_NAME__) \
    appCfg.write("", #SL_NAME__, SL_NAME__->value())

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
        LOAD_CB_STATE(cbKeepOriginalCloudColors);
        LOAD_CB_STATE(cbShowGroundGrid);

        LOAD_SL_STATE(slPointSize);
        LOAD_SL_STATE(slMidDepthField);
        LOAD_SL_STATE(slThicknessDepthField);
        LOAD_SL_STATE(slCameraFOV);

        win->camera().setCameraPointing(
            appCfg.read_float("", "cam_x", win->camera().getCameraPointingX()),
            appCfg.read_float("", "cam_y", win->camera().getCameraPointingY()),
            appCfg.read_float("", "cam_z", win->camera().getCameraPointingZ()));
        win->camera().setAzimuthDegrees(
            appCfg.read_float("", "cam_az", win->camera().getAzimuthDegrees()));
        win->camera().setElevationDegrees(appCfg.read_float(
            "", "cam_el", win->camera().getElevationDegrees()));
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
        SAVE_CB_STATE(cbKeepOriginalCloudColors);
        SAVE_CB_STATE(cbShowGroundGrid);

        SAVE_SL_STATE(slPointSize);
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
            updateMiniCornerView();
        });

    nanogui::mainloop(1 /*refresh Hz*/);

    nanogui::shutdown();

    // save UI state:
    save_UI_state_to_user_config();
}

}  // namespace
// ==============================
// rebuild_3d_view
// ==============================
void rebuild_3d_view()
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
            if (auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                    itL->second);
                pc)
            {
                cb->setCaption(
                    lyName + " | "s +
                    mrpt::system::unitsFormat(
                        static_cast<double>(pc->size()), 2, false) +
                    " points"s + " | class="s +
                    pc->GetRuntimeClass()->className);

                const auto bb = pc->boundingBox();
                if (!mapBbox.has_value())
                    mapBbox = bb;
                else { mapBbox = mapBbox->unionWith(bb); }
            }
            else
            {
                cb->setCaption(
                    lyName + " | class="s +
                    itL->second->GetRuntimeClass()->className);
            }
        }

        // show/hide:
        if (!cb->checked()) continue;  // hidden
        rpMap.points.visible = true;

        auto& rpL                       = rpMap.points.perLayer[lyName];
        rpL.pointSize                   = slPointSize->value();
        rpL.render_voxelmaps_as_points  = cbViewVoxelsAsPoints->checked();
        rpL.render_voxelmaps_free_space = cbViewVoxelsFreeSpace->checked();

        lbPointSize->setCaption("Point size: " + std::to_string(rpL.pointSize));

        if (cbColorizeMap->checked())
        {
            auto& cm                  = rpL.colorMode.emplace();
            cm.colorMap               = mrpt::img::TColormap::cmJET;
            cm.recolorizeByCoordinate = mp2p_icp::Coordinate::Z;
        }
        if (cbKeepOriginalCloudColors->checked())
        {
            auto& cm                     = rpL.colorMode.emplace();
            cm.keep_original_cloud_color = true;
        }
    }

    // Default color:
    for (auto& [layer, rp] : rpMap.points.perLayer)
        rp.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

    // Regenerate points opengl representation only if some parameter changed:
    static std::optional<mp2p_icp::render_params_t> prevRenderParams;

    if (!prevRenderParams.has_value() || prevRenderParams.value() != rpMap)
    {
        prevRenderParams = rpMap;
        glVizMap->clear();

        auto glPts = theMap.get_visualization(rpMap);

        // Show all or selected layers:
        rpMap.points.allLayers.color =
            mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        glVizMap->insert(glPts);

        glVizMap->insert(glMapCorner);
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
        glGrid->setPlaneLimits(
            mapBbox->min.x, mapBbox->max.x, mapBbox->min.y, mapBbox->max.y);
    }
    glGrid->setVisibility(cbShowGroundGrid->checked());

    // XYZ corner overlay viewport:
    {
        auto gl_view = win->background_scene->createViewport("small-view");

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

    // Global view options:
    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->camera().setCameraProjective(
            !cbViewOrtho->checked() && !cbView2D->checked());

        if (cbView2D->checked())
        {
            win->camera().setAzimuthDegrees(-90.0f);
            win->camera().setElevationDegrees(90.0f);
        }

        // clip planes:
        const auto depthFieldMid = std::pow(10.0, slMidDepthField->value());
        const auto depthFieldThickness =
            std::pow(10.0, slThicknessDepthField->value());

        const auto clipNear =
            std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness);
        const auto clipFar = depthFieldMid + 0.5 * depthFieldThickness;

        const float cameraFOV = slCameraFOV->value();
        win->camera().setCameraFOV(cameraFOV);
        win->camera().setMaximumZoom(std::max<double>(1000, 3.0 * clipFar));

        lbDepthFieldValues->setCaption(
            mrpt::format("Depth field: near=%f far=%f", clipNear, clipFar));
        lbDepthFieldMid->setCaption(
            mrpt::format("Frustrum center: %.03f", depthFieldMid));
        lbDepthFieldThickness->setCaption(
            mrpt::format("Frustum thickness: %.03f", depthFieldThickness));

        lbDepthFieldValues->setCaption(
            mrpt::format("Frustum: near=%.02f far=%.02f", clipNear, clipFar));

        lbCameraFOV->setCaption(
            mrpt::format("Camera FOV: %.02f deg", cameraFOV));

        win->background_scene->getViewport()->setViewportClipDistances(
            clipNear, clipFar);
    }
}

void onSaveLayers()
{
    const std::string outFile =
        nanogui::file_dialog({{"txt", "(*.txt)"}}, true /*save*/);
    if (outFile.empty()) return;

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
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        // Load plugins:
        if (arg_plugins.isSet())
        {
            std::string errMsg;
            const auto  plugins = arg_plugins.getValue();
            std::cout << "Loading plugin(s): " << plugins << std::endl;
            if (!mrpt::system::loadPluginModules(plugins, errMsg))
            {
                std::cerr << errMsg << std::endl;
                return 1;
            }
        }

        main_show_gui();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
