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
#include <mrpt/core/round.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // loadPluginModules()
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <iostream>

constexpr const char* APP_NAME      = "mm-viewer";
constexpr int         MID_FONT_SIZE = 14;

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
mrpt::gui::CDisplayWindowGUI::Ptr win;

std::array<nanogui::TextBox*, 2> lbMapStats                = {nullptr, nullptr};
nanogui::CheckBox*               cbViewOrtho               = nullptr;
nanogui::CheckBox*               cbViewVoxelsAsPoints      = nullptr;
nanogui::CheckBox*               cbColorizeMap             = nullptr;
nanogui::CheckBox*               cbKeepOriginalCloudColors = nullptr;
nanogui::CheckBox*               cbShowGroundGrid          = nullptr;
nanogui::Slider*                 slPointSize               = nullptr;
nanogui::Slider*                 slMidDepthField           = nullptr;
nanogui::Slider*                 slThicknessDepthField     = nullptr;
nanogui::Slider*                 slCameraFOV               = nullptr;
nanogui::Label*                  lbCameraFOV               = nullptr;
nanogui::Label *lbDepthFieldValues = nullptr, *lbDepthFieldMid = nullptr,
               *lbDepthFieldThickness = nullptr;

std::vector<std::string>                  layerNames;
std::map<std::string, nanogui::CheckBox*> cbLayersByName;

mp2p_icp::metric_map_t theMap;
std::string            theMapFileName = "unnamed.mm";

static void rebuild_3d_view();

static void loadMapFile(const std::string& mapFile)
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

static void main_show_gui()
{
    using namespace std::string_literals;

    if (argMapFile.isSet()) { loadMapFile(argMapFile.getValue()); }

    /*
     * -------------------------------------------------------------------
     * Plot 3D:
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

    auto gl_base = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    gl_base->setName("map");
    gl_base->enableShowName();
    scene->insert(gl_base);

    scene->insert(glVizMap);

    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->background_scene = std::move(scene);
    }

    // Control GUI sub-window:
    {
        auto w = win->createManagedSubWindow("Control");
        w->setPosition({5, 25});
        w->requestFocus();
        w->setLayout(new nanogui::BoxLayout(
            nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
        w->setFixedWidth(400);

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
            cb->setFontSize(13);

            cbLayersByName[layerNames.at(i)] = cb;
        }

        // tab
        {
            auto pn = tab1->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            pn->add<nanogui::Label>("Point size");

            slPointSize = pn->add<nanogui::Slider>();
            slPointSize->setRange({1.0f, 10.0f});
            slPointSize->setValue(2.0f);
            slPointSize->setCallback([&](float) { rebuild_3d_view(); });
        }

        lbDepthFieldMid = tab1->add<nanogui::Label>("Center depth clip plane:");
        slMidDepthField = tab1->add<nanogui::Slider>();
        slMidDepthField->setRange({-2.0, 3.0});
        slMidDepthField->setValue(1.0f);
        slMidDepthField->setCallback([&](float) { rebuild_3d_view(); });

        lbDepthFieldThickness =
            tab1->add<nanogui::Label>("Max-Min depth thickness:");
        slThicknessDepthField = tab1->add<nanogui::Slider>();
        slThicknessDepthField->setRange({-2.0, 4.0});
        slThicknessDepthField->setValue(3.0);
        slThicknessDepthField->setCallback([&](float) { rebuild_3d_view(); });
        lbDepthFieldValues = tab1->add<nanogui::Label>(" ");

        lbCameraFOV = tab1->add<nanogui::Label>("Camera FOV:");
        slCameraFOV = tab1->add<nanogui::Slider>();
        slCameraFOV->setRange({20.0f, 170.0f});
        slCameraFOV->setValue(90.0f);
        slCameraFOV->setCallback([&](float) { rebuild_3d_view(); });

        cbViewOrtho = tab1->add<nanogui::CheckBox>("Orthogonal view");
        cbViewOrtho->setCallback([&](bool) { rebuild_3d_view(); });

        cbViewVoxelsAsPoints =
            tab1->add<nanogui::CheckBox>("Render voxel maps as point clouds");
        cbViewVoxelsAsPoints->setChecked(true);
        cbViewVoxelsAsPoints->setCallback([&](bool) { rebuild_3d_view(); });

        cbColorizeMap = tab1->add<nanogui::CheckBox>("Recolorize map points");
        cbColorizeMap->setChecked(true);
        cbColorizeMap->setCallback([&](bool) { rebuild_3d_view(); });

        cbKeepOriginalCloudColors =
            tab1->add<nanogui::CheckBox>("Keep original cloud colors");
        cbKeepOriginalCloudColors->setChecked(false);
        cbKeepOriginalCloudColors->setCallback(
            [&](bool) { rebuild_3d_view(); });

        cbShowGroundGrid = tab1->add<nanogui::CheckBox>("Show ground grid");
        cbShowGroundGrid->setChecked(true);
        cbShowGroundGrid->setCallback([&](bool) { rebuild_3d_view(); });

        // ----
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
            ->setCallback([]() { win->setVisible(false); });

        win->setKeyboardCallback([&](int key, [[maybe_unused]] int scancode,
                                     int                  action,
                                     [[maybe_unused]] int modifiers) {
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

    rebuild_3d_view();

    // Main loop
    // ---------------------
    win->drawAll();
    win->setVisible(true);

    win->addLoopCallback([&]() {
        // None
    });

    nanogui::mainloop(1 /*refresh Hz*/);

    nanogui::shutdown();
}

// ==============================
// rebuild_3d_view
// ==============================
void rebuild_3d_view()
{
    using namespace std::string_literals;

    lbMapStats[0]->setValue(theMapFileName);
    lbMapStats[1]->setValue("Map: "s + theMap.contents_summary());

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
                else
                {
                    mapBbox->unionWith(bb);
                }
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

        auto& rpL                      = rpMap.points.perLayer[lyName];
        rpL.pointSize                  = slPointSize->value();
        rpL.render_voxelmaps_as_points = cbViewVoxelsAsPoints->checked();

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
    }

    // ground grid:
    if (mapBbox)
    {
        glGrid->setPlaneLimits(
            mapBbox->min.x, mapBbox->max.x, mapBbox->min.y, mapBbox->max.y);
    }
    glGrid->setVisibility(cbShowGroundGrid->checked());

    // Global view options:
    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->camera().setCameraProjective(!cbViewOrtho->checked());

        // clip planes:
        const auto depthFieldMid = std::pow(10.0, slMidDepthField->value());
        const auto depthFieldThickness =
            std::pow(10.0, slThicknessDepthField->value());

        const auto clipNear =
            std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness);
        const auto clipFar = depthFieldMid + 0.5 * depthFieldThickness;

        const float cameraFOV = slCameraFOV->value();
        win->camera().setCameraFOV(cameraFOV);

        lbDepthFieldMid->setCaption(
            mrpt::format("Center depth clip plane: %f", depthFieldMid));
        lbDepthFieldThickness->setCaption(
            mrpt::format("Max-Min depth thickness: %f", depthFieldThickness));
        lbDepthFieldValues->setCaption(
            mrpt::format("Depth field: near=%f far=%f", clipNear, clipFar));
        lbCameraFOV->setCaption(
            mrpt::format("Camera FOV: %.02f deg", cameraFOV));

        win->background_scene->getViewport()->setViewportClipDistances(
            clipNear, clipFar);
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
