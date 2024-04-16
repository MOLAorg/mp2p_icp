/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   icp-log-viewer/main.cpp
 * @brief  GUI tool to analize ICP log records (*.icplog files)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 15, 2021
 */

// The goal is to visualize these guys:
#include <mp2p_icp/LogRecord.h>
// using this:
#include <mrpt/gui/CDisplayWindowGUI.h>

// other deps:
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config.h>
#include <mrpt/core/round.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <iostream>

constexpr const char* APP_NAME      = "icp-log-viewer";
constexpr int         MID_FONT_SIZE = 14;

// =========== Declare supported cli switches ===========
static TCLAP::CmdLine cmd(APP_NAME);

static TCLAP::ValueArg<std::string> argExtension(
    "e", "file-extension",
    "Filename extension to look for. Default is `icplog`", false, "icplog",
    "icplog", cmd);

static TCLAP::ValueArg<std::string> argSearchDir(
    "d", "directory", "Directory in which to search for *.icplog files.", false,
    ".", ".", cmd);

static TCLAP::ValueArg<std::string> argSingleFile(
    "f", "file", "Load just this one single log *.icplog file.", false,
    "log.icplog", "log.icplog", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "l", "load-plugins",
    "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

static TCLAP::ValueArg<double> argAutoPlayPeriod(
    "", "autoplay-period",
    "The period (in seconds) between timestamps to load and show in autoplay "
    "mode.",
    false, 0.1, "period [seconds]", cmd);

// =========== Declare global variables ===========
#if MRPT_HAS_NANOGUI

auto glVizICP = mrpt::opengl::CSetOfObjects::Create();
mrpt::gui::CDisplayWindowGUI::Ptr win;

nanogui::Slider* slSelectorICP   = nullptr;
nanogui::Button *btnSelectorBack = nullptr, *btnSelectorForw = nullptr;
nanogui::Button* btnSelectorAutoplay = nullptr;
bool             isAutoPlayActive    = false;
double           lastAutoPlayTime    = .0;

std::array<nanogui::TextBox*, 5> lbICPStats = {
    nullptr, nullptr, nullptr, nullptr, nullptr};
nanogui::CheckBox* cbShowInitialPose  = nullptr;
nanogui::Label*    lbICPIteration     = nullptr;
nanogui::Slider*   slIterationDetails = nullptr;

nanogui::CheckBox* cbViewOrtho          = nullptr;
nanogui::CheckBox* cbCameraFollowsLocal = nullptr;
nanogui::CheckBox* cbViewVoxelsAsPoints = nullptr;
nanogui::CheckBox* cbColorizeLocalMap   = nullptr;
nanogui::CheckBox* cbColorizeGlobalMap  = nullptr;

nanogui::CheckBox* cbViewPairings = nullptr;

nanogui::CheckBox* cbViewPairings_pt2pt = nullptr;
nanogui::CheckBox* cbViewPairings_pt2pl = nullptr;
nanogui::CheckBox* cbViewPairings_pt2ln = nullptr;

nanogui::TextBox *tbLogPose = nullptr, *tbInitialGuess = nullptr,
                 *tbInit2Final = nullptr, *tbCovariance = nullptr,
                 *tbConditionNumber = nullptr, *tbPairings = nullptr;

nanogui::Slider* slPairingsPl2PlSize   = nullptr;
nanogui::Slider* slPairingsPl2LnSize   = nullptr;
nanogui::Slider* slPointSize           = nullptr;
nanogui::Slider* slMidDepthField       = nullptr;
nanogui::Slider* slThicknessDepthField = nullptr;
nanogui::Label * lbDepthFieldValues = nullptr, *lbDepthFieldMid = nullptr,
               *lbDepthFieldThickness = nullptr;

nanogui::Slider* slGTPose[6] = {nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr};

std::vector<std::string> layerNames_global, layerNames_local;

std::map<std::string, nanogui::CheckBox*> cbLayersByName_global;
std::map<std::string, nanogui::CheckBox*> cbLayersByName_local;

class DelayedLoadLog
{
   public:
    DelayedLoadLog() = default;
    DelayedLoadLog(
        const std::string& fileName, const std::string& shortFileName)
        : filename_(fileName), shortFileName_(shortFileName)
    {
    }

    mp2p_icp::LogRecord& get()
    {
        if (!log_)
        {
            // Load now:
            log_ = mp2p_icp::LogRecord::LoadFromFile(filename_);
        }

        return log_.value();
    }

    void dispose() { log_.reset(); }

    const std::string& filename() const { return filename_; }
    const std::string& shortFileName() const { return shortFileName_; }

   private:
    std::optional<mp2p_icp::LogRecord> log_;
    std::string                        filename_, shortFileName_;
};

std::vector<DelayedLoadLog> logRecords;

static void rebuild_3d_view(bool regenerateMaps = true);

static void rebuild_3d_view_fast() { rebuild_3d_view(false); }

static void main_show_gui()
{
    using namespace std::string_literals;

    mrpt::system::CDirectoryExplorer::TFileInfoList files;

    if (!argSingleFile.isSet())
    {
        const std::string searchDir = argSearchDir.getValue();
        ASSERT_DIRECTORY_EXISTS_(searchDir);

        std::cout << "Searching in: '" << searchDir
                  << "' for files with extension '" << argExtension.getValue()
                  << "'" << std::endl;

        mrpt::system::CDirectoryExplorer::explore(
            searchDir, FILE_ATTRIB_ARCHIVE, files);
        mrpt::system::CDirectoryExplorer::filterByExtension(
            files, argExtension.getValue());
        mrpt::system::CDirectoryExplorer::sortByName(files);

        std::cout << "Found " << files.size() << " ICP records." << std::endl;
    }
    else
    {
        // Load one single file:
        std::cout << "Loading one single log file: " << argSingleFile.getValue()
                  << std::endl;

        files.resize(1);
        files[0].wholePath = argSingleFile.getValue();
    }

    // load files:
    for (const auto& file : files)
        logRecords.emplace_back(file.wholePath, file.name);

    ASSERT_(!logRecords.empty());

    // Obtain layer info from first entry:
    {
        const auto& lr = logRecords.front().get();
        if (layerNames_global.empty() && lr.pcGlobal)
        {
            for (const auto& layer : lr.pcGlobal->layers)
            {
                layerNames_global.push_back(layer.first);
                std::cout << "Global point cloud: Found point layer='"
                          << layer.first << "'\n";
            }
        }
        if (layerNames_local.empty() && lr.pcLocal)
        {
            for (const auto& layer : lr.pcLocal->layers)
            {
                layerNames_local.push_back(layer.first);
                std::cout << "Local point cloud: Found point layer='"
                          << layer.first << "'\n";
            }
        }
    }

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
        auto glGrid = mrpt::opengl::CGridPlaneXY::Create();
        glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
        scene->insert(glGrid);
    }

    auto gl_base = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    gl_base->setName("Global");
    gl_base->enableShowName();
    scene->insert(gl_base);

    scene->insert(glVizICP);

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
        w->setFixedWidth(450);

        cbShowInitialPose =
            w->add<nanogui::CheckBox>("Show at INITIAL GUESS pose");
        cbShowInitialPose->setCallback(
            [=](bool v)
            {
                if (slIterationDetails->enabled())
                {
                    const auto& r = slIterationDetails->range();
                    slIterationDetails->setValue(v ? r.first : r.second);
                }
                rebuild_3d_view_fast();
            });
        lbICPIteration     = w->add<nanogui::Label>("Show ICP iteration:");
        slIterationDetails = w->add<nanogui::Slider>();
        slIterationDetails->setRange({.0f, 1.0f});
        slIterationDetails->setCallback([](float) { rebuild_3d_view_fast(); });

        //
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Label>(mrpt::format(
            "Select ICP record file (N=%u)",
            static_cast<unsigned int>(logRecords.size())));
        slSelectorICP = w->add<nanogui::Slider>();
        slSelectorICP->setRange({.0f, logRecords.size() - 1});
        slSelectorICP->setValue(0);
        slSelectorICP->setCallback([&](float /*v*/) { rebuild_3d_view(); });

        for (auto& lb : lbICPStats)
        {
            lb = w->add<nanogui::TextBox>("  ");
            lb->setFontSize(MID_FONT_SIZE);
            lb->setAlignment(nanogui::TextBox::Alignment::Left);
            lb->setEditable(true);
        }

        // navigation panel:
        {
            auto pn = w->add<nanogui::Widget>();
            pn->setLayout(
                new nanogui::BoxLayout(nanogui::Orientation::Horizontal));

            // shortcut:
            auto s = slSelectorICP;

            btnSelectorBack = pn->add<nanogui::Button>(
                "", ENTYPO_ICON_CONTROLLER_FAST_BACKWARD);
            btnSelectorBack->setCallback(
                [=]()
                {
                    if (s->value() > 0)
                    {
                        s->setValue(s->value() - 1);
                        s->callback()(s->value());
                    }
                });

            btnSelectorForw = pn->add<nanogui::Button>(
                "", ENTYPO_ICON_CONTROLLER_FAST_FORWARD);
            btnSelectorForw->setCallback(
                [=]()
                {
                    if (s->value() < s->range().second - 0.01f)
                    {
                        s->setValue(s->value() + 1);
                        rebuild_3d_view();
                    }
                });

            pn->add<nanogui::Label>(" ");  // separator

            btnSelectorAutoplay =
                pn->add<nanogui::Button>("", ENTYPO_ICON_CONTROLLER_PLAY);
            btnSelectorAutoplay->setFlags(nanogui::Button::ToggleButton);

            btnSelectorAutoplay->setChangeCallback(
                [&](bool active) { isAutoPlayActive = active; });
        }

        //
        w->add<nanogui::Label>(" ");  // separator

        auto tabWidget = w->add<nanogui::TabWidget>();

        auto* tab1 = tabWidget->createTab("Summary");
        tab1->setLayout(new nanogui::GroupLayout());

        auto* tab2 = tabWidget->createTab("Uncertainty");
        tab2->setLayout(new nanogui::GroupLayout());

        auto* tab3 = tabWidget->createTab("Maps");
        tab3->setLayout(new nanogui::GroupLayout());

        auto* tab4 = tabWidget->createTab("Pairings");
        tab4->setLayout(new nanogui::GroupLayout());

        auto* tab5 = tabWidget->createTab("View");
        tab5->setLayout(new nanogui::GroupLayout());

        tabWidget->setActiveTab(0);

        tab1->add<nanogui::Label>(
            "ICP result pose [x y z yaw(deg) pitch(deg) roll(deg)]:");
        tbLogPose = tab1->add<nanogui::TextBox>();
        tbLogPose->setFontSize(MID_FONT_SIZE);
        tbLogPose->setEditable(true);
        tbLogPose->setAlignment(nanogui::TextBox::Alignment::Left);

        tab1->add<nanogui::Label>("Initial -> final pose change:");
        tbInit2Final = tab1->add<nanogui::TextBox>();
        tbInit2Final->setFontSize(MID_FONT_SIZE);
        tbInit2Final->setEditable(false);

        tab2->add<nanogui::Label>(
            "Uncertainty: diagonal sigmas (x y z yaw pitch roll)");
        tbCovariance = tab2->add<nanogui::TextBox>();
        tbCovariance->setFontSize(MID_FONT_SIZE);
        tbCovariance->setEditable(false);

        tab2->add<nanogui::Label>("Uncertainty: Covariance condition numbers");
        tbConditionNumber = tab2->add<nanogui::TextBox>();
        tbConditionNumber->setFontSize(MID_FONT_SIZE);
        tbConditionNumber->setEditable(false);

        const float handTunedRange[6] = {4.0,        4.0,         10.0,
                                         0.5 * M_PI, 0.25 * M_PI, 0.5};

        for (int i = 0; i < 6; i++)
        {
            slGTPose[i] = tab2->add<nanogui::Slider>();
            slGTPose[i]->setRange({-handTunedRange[i], handTunedRange[i]});
            slGTPose[i]->setValue(0.0f);

            slGTPose[i]->setCallback(
                [=](float v)
                {
                    const size_t idx = mrpt::round(slSelectorICP->value());
                    auto&        lr  = logRecords.at(idx).get();

                    auto p = lr.icpResult.optimal_tf.mean.asTPose();
                    p[i]   = v;
                    lr.icpResult.optimal_tf.mean = mrpt::poses::CPose3D(p);

                    rebuild_3d_view_fast();
                });
        }

        tab1->add<nanogui::Label>("Initial guess pose:");
        tbInitialGuess = tab1->add<nanogui::TextBox>();
        tbInitialGuess->setFontSize(14);
        tbInitialGuess->setEditable(true);

        // Save map buttons:
        auto lambdaSave = [&](const mp2p_icp::metric_map_t& m)
        {
            const std::string outFile = nanogui::file_dialog(
                {{"mm",
                  "mp2p_icp::metric_map_t binary serialized object (*.mm)"}},
                true /*save*/);
            if (outFile.empty()) return;
            m.save_to_file(outFile);
        };

        // tab 3:
        {
            auto pn = tab3->add<nanogui::Widget>();
            pn->setLayout(new nanogui::BoxLayout(
                nanogui::Orientation::Horizontal, nanogui::Alignment::Fill));
            pn->add<nanogui::Button>("Export 'local' map...")
                ->setCallback(
                    [&]()
                    {
                        const size_t idx = mrpt::round(slSelectorICP->value());
                        auto&        lr  = logRecords.at(idx).get();
                        ASSERT_(lr.pcLocal);
                        lambdaSave(*lr.pcLocal);
                    });
            pn->add<nanogui::Button>("Export 'global' map...")
                ->setCallback(
                    [&]()
                    {
                        const size_t idx = mrpt::round(slSelectorICP->value());
                        auto&        lr  = logRecords.at(idx).get();
                        ASSERT_(lr.pcGlobal);
                        lambdaSave(*lr.pcGlobal);
                    });
        }

        tab3->add<nanogui::Label>("[GLOBAL map] Visible layers:");

        for (size_t i = 0; i < layerNames_global.size(); i++)
        {
            auto cb = tab3->add<nanogui::CheckBox>(layerNames_global.at(i));
            cb->setChecked(true);
            cb->setCallback([](bool) { rebuild_3d_view(); });
            cb->setFontSize(13);

            cbLayersByName_global[layerNames_global.at(i)] = cb;
        }

        tab3->add<nanogui::Label>("[LOCAL map] Visible layers:");
        for (size_t i = 0; i < layerNames_local.size(); i++)
        {
            auto cb = tab3->add<nanogui::CheckBox>(layerNames_local.at(i));
            cb->setChecked(true);
            cb->setCallback([](bool) { rebuild_3d_view(); });
            cb->setFontSize(13);

            cbLayersByName_local[layerNames_local.at(i)] = cb;
        }

        // tab4: pairings:
        tbPairings = tab4->add<nanogui::TextBox>();
        tbPairings->setFontSize(16);
        tbPairings->setEditable(false);

        cbViewPairings = tab4->add<nanogui::CheckBox>("View pairings");
        cbViewPairings->setCallback([](bool) { rebuild_3d_view_fast(); });

        cbViewPairings_pt2pt =
            tab4->add<nanogui::CheckBox>("View: point-to-point");
        cbViewPairings_pt2pt->setChecked(true);
        cbViewPairings_pt2pt->setCallback([](bool) { rebuild_3d_view_fast(); });

        {
            auto pn = tab4->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill));

            cbViewPairings_pt2pl =
                pn->add<nanogui::CheckBox>("View: point-to-plane");
            cbViewPairings_pt2pl->setChecked(true);
            cbViewPairings_pt2pl->setCallback([](bool)
                                              { rebuild_3d_view_fast(); });

            pn->add<nanogui::Label>("Plane size:");
            slPairingsPl2PlSize = pn->add<nanogui::Slider>();
            slPairingsPl2PlSize->setRange({-2.0f, 2.0f});
            slPairingsPl2PlSize->setValue(-1.0f);
            slPairingsPl2PlSize->setCallback([&](float)
                                             { rebuild_3d_view_fast(); });
        }

        {
            auto pn = tab4->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill));

            cbViewPairings_pt2ln =
                pn->add<nanogui::CheckBox>("View: point-to-line");
            cbViewPairings_pt2ln->setChecked(true);
            cbViewPairings_pt2ln->setCallback([](bool)
                                              { rebuild_3d_view_fast(); });

            pn->add<nanogui::Label>("Line length:");
            slPairingsPl2LnSize = pn->add<nanogui::Slider>();
            slPairingsPl2LnSize->setRange({-2.0f, 2.0f});
            slPairingsPl2LnSize->setValue(-1.0f);
            slPairingsPl2LnSize->setCallback([&](float)
                                             { rebuild_3d_view_fast(); });
        }

        // tab5: view
        {
            auto pn = tab5->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            pn->add<nanogui::Label>("Point size");

            slPointSize = pn->add<nanogui::Slider>();
            slPointSize->setRange({1.0f, 10.0f});
            slPointSize->setValue(2.0f);
            slPointSize->setCallback([&](float) { rebuild_3d_view(); });
        }

        lbDepthFieldMid = tab5->add<nanogui::Label>("Center depth clip plane:");
        slMidDepthField = tab5->add<nanogui::Slider>();
        slMidDepthField->setRange({-2.0, 3.0});
        slMidDepthField->setValue(1.0f);
        slMidDepthField->setCallback([&](float) { rebuild_3d_view_fast(); });

        lbDepthFieldThickness =
            tab5->add<nanogui::Label>("Max-Min depth thickness:");
        slThicknessDepthField = tab5->add<nanogui::Slider>();
        slThicknessDepthField->setRange({-2.0, 4.0});
        slThicknessDepthField->setValue(3.0);
        slThicknessDepthField->setCallback([&](float)
                                           { rebuild_3d_view_fast(); });
        lbDepthFieldValues = tab5->add<nanogui::Label>(" ");

        cbViewOrtho = tab5->add<nanogui::CheckBox>("Orthogonal view");
        cbViewOrtho->setCallback([&](bool) { rebuild_3d_view_fast(); });

        cbCameraFollowsLocal =
            tab5->add<nanogui::CheckBox>("Camera follows 'local'");
        cbCameraFollowsLocal->setCallback([&](bool)
                                          { rebuild_3d_view_fast(); });

        cbViewVoxelsAsPoints =
            tab5->add<nanogui::CheckBox>("Render voxel maps as point clouds");
        cbViewVoxelsAsPoints->setChecked(true);
        cbViewVoxelsAsPoints->setCallback([&](bool) { rebuild_3d_view(); });

        cbColorizeLocalMap =
            tab5->add<nanogui::CheckBox>("Recolorize local map");
        cbColorizeLocalMap->setCallback([&](bool) { rebuild_3d_view(); });

        cbColorizeGlobalMap =
            tab5->add<nanogui::CheckBox>("Recolorize global map");
        cbColorizeGlobalMap->setCallback([&](bool) { rebuild_3d_view(); });

        // ----
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
            ->setCallback([]() { win->setVisible(false); });

        win->setKeyboardCallback(
            [&](int key, [[maybe_unused]] int scancode, int action,
                [[maybe_unused]] int modifiers)
            {
                if (action != GLFW_PRESS && action != GLFW_REPEAT) return false;

                int increment = 0;
                switch (key)
                {
                    case GLFW_KEY_LEFT:
                        increment = -1;
                        break;
                    case GLFW_KEY_RIGHT:
                        increment = +1;
                        break;
                    case GLFW_KEY_PAGE_DOWN:
                        increment = +100;
                        break;
                    case GLFW_KEY_PAGE_UP:
                        increment = -100;
                        break;
                    case GLFW_KEY_SPACE:
                        isAutoPlayActive = !isAutoPlayActive;
                        btnSelectorAutoplay->setPushed(isAutoPlayActive);
                        break;
                };

                if (increment != 0)
                {
                    nanogui::Slider* sl = slSelectorICP;  // shortcut
                    sl->setValue(sl->value() + increment);
                    if (sl->value() < 0) sl->setValue(0);
                    if (sl->value() > sl->range().second)
                        sl->setValue(sl->range().second);
                    rebuild_3d_view();
                }

                return false;
            });
    }

    win->performLayout();
    win->camera().setCameraPointing(8.0f, .0f, .0f);
    win->camera().setAzimuthDegrees(110.0f);
    win->camera().setElevationDegrees(15.0f);
    win->camera().setZoomDistance(30.0f);

    rebuild_3d_view();

    // Main loop
    // ---------------------
    win->drawAll();
    win->setVisible(true);

    win->addLoopCallback(
        [&]()
        {
            if (!isAutoPlayActive) return;

            const double tNow = mrpt::Clock::nowDouble();
            if (tNow - lastAutoPlayTime < argAutoPlayPeriod.getValue()) return;

            lastAutoPlayTime = tNow;

            if (slSelectorICP->value() < slSelectorICP->range().second - 0.01f)
            {
                slSelectorICP->setValue(slSelectorICP->value() + 1);
                rebuild_3d_view();
            }
        });

    nanogui::mainloop(10 /*refresh Hz*/);

    nanogui::shutdown();
}

template <class MATRIX>  //
double conditionNumber(const MATRIX& m)
{
    MATRIX              eVecs;
    std::vector<double> eVals;
    m.eig_symmetric(eVecs, eVals);
    return eVals.back() / eVals.front();
}

// ==============================
// rebuild_3d_view
// ==============================
void rebuild_3d_view(bool regenerateMaps)
{
    using namespace std::string_literals;

    const size_t idx = mrpt::round(slSelectorICP->value());

    btnSelectorBack->setEnabled(!logRecords.empty() && idx > 0);
    btnSelectorForw->setEnabled(
        !logRecords.empty() && idx < logRecords.size() - 1);

    glVizICP->clear();

    // Free memory
    static std::optional<size_t> lastIdx;
    bool                         mustResetIterationSlider = false;

    if (!lastIdx || (lastIdx && idx != *lastIdx))
    {
        // free memory:
        if (lastIdx) logRecords.at(*lastIdx).dispose();

        // and note that we should show the first/last ICP iteration:
        mustResetIterationSlider = true;
    }
    lastIdx = idx;

    // lazy load from disk happens in the "get()":
    const auto& lr = logRecords.at(idx).get();

    lbICPStats[0]->setValue(logRecords.at(idx).shortFileName());

    lbICPStats[1]->setValue(mrpt::format(
        "ICP log #%zu | Local: ID:%u%s | Global: ID:%u%s", idx,
        static_cast<unsigned int>(lr.pcLocal->id ? lr.pcLocal->id.value() : 0),
        lr.pcLocal->label ? lr.pcLocal->label.value().c_str() : "",
        static_cast<unsigned int>(
            lr.pcGlobal->id ? lr.pcGlobal->id.value() : 0),
        lr.pcGlobal->label ? lr.pcGlobal->label.value().c_str() : ""));

    lbICPStats[2]->setValue(mrpt::format(
        "Quality: %.02f%% | Iters: %u | Term.Reason: %s",
        100.0 * lr.icpResult.quality,
        static_cast<unsigned int>(lr.icpResult.nIterations),
        mrpt::typemeta::enum2str(lr.icpResult.terminationReason).c_str()));

    lbICPStats[3]->setValue("Global: "s + lr.pcGlobal->contents_summary());
    lbICPStats[4]->setValue("Local: "s + lr.pcLocal->contents_summary());

    tbInitialGuess->setValue(lr.initialGuessLocalWrtGlobal.asString());

    tbLogPose->setValue(lr.icpResult.optimal_tf.mean.asString());

    {
        const auto poseChange =
            lr.icpResult.optimal_tf.mean -
            mrpt::poses::CPose3D(lr.initialGuessLocalWrtGlobal);

        tbInit2Final->setValue(mrpt::format(
            "|T|=%.03f [m]  |R|=%.03f [deg]", poseChange.norm(),
            mrpt::RAD2DEG(
                mrpt::poses::Lie::SO<3>::log(poseChange.getRotationMatrix())
                    .norm())));
    }

    const auto poseFromCorner = mrpt::poses::CPose3D::Identity();
    mrpt::poses::CPose3DPDFGaussian relativePose;
    const mp2p_icp::Pairings*       pairsToViz = nullptr;

    if (!lr.iterationsDetails.has_value() || lr.iterationsDetails->empty())
    {
        slIterationDetails->setEnabled(false);

        if (cbShowInitialPose->checked())
        {
            relativePose.mean =
                mrpt::poses::CPose3D(lr.initialGuessLocalWrtGlobal);
            lbICPIteration->setCaption("Show ICP iteration: INITIAL");
        }
        else
        {
            relativePose = lr.icpResult.optimal_tf;
            lbICPIteration->setCaption("Show ICP iteration: FINAL");
        }

        if (cbViewPairings->checked()) pairsToViz = &lr.icpResult.finalPairings;
    }
    else
    {
        slIterationDetails->setEnabled(true);
        slIterationDetails->setRange(
            {.0f, static_cast<float>(lr.iterationsDetails->size() - 1)});

        if (mustResetIterationSlider)
            slIterationDetails->setValue(
                cbShowInitialPose->checked()
                    ? slIterationDetails->range().first
                    : slIterationDetails->range().second);

        // final or partial solution?
        auto it = lr.iterationsDetails->begin();

        size_t nIter = mrpt::round(slIterationDetails->value());
        mrpt::keep_min(nIter, lr.iterationsDetails->size() - 1);

        std::advance(it, nIter);

        relativePose.mean = it->second.optimalPose;
        if (cbViewPairings->checked()) pairsToViz = &it->second.pairings;

        lbICPIteration->setCaption(
            "Show ICP iteration: "s + std::to_string(it->first) + "/"s +
            std::to_string(lr.iterationsDetails->rbegin()->first));
    }

    {
        std::string s;
        for (int i = 0; i < 6; i++)
            s += mrpt::format("%.02f ", std::sqrt(relativePose.cov(i, i)));

        s += mrpt::format(
            " det(XYZ)=%.02e det(rot)=%.02e",
            relativePose.cov.blockCopy<3, 3>(0, 0).det(),
            relativePose.cov.blockCopy<3, 3>(3, 3).det());

        tbCovariance->setValue(s);
    }

    // Extract SE(2) covariance:
    const mrpt::poses::CPosePDFGaussian pose2D(relativePose);

    // Condition numbers:
    tbConditionNumber->setValue(mrpt::format(
        " cn{XYZ}=%.02f cn{SO(3)}=%.02f cn{SE(2)}=%.02f "
        "cn{SE(3)}=%.02f",
        conditionNumber(relativePose.cov.blockCopy<3, 3>(0, 0)),
        conditionNumber(relativePose.cov.blockCopy<3, 3>(3, 3)),
        conditionNumber(pose2D.cov), conditionNumber(relativePose.cov)));

    // 3D objects -------------------
    auto glCornerFrom =
        mrpt::opengl::stock_objects::CornerXYZSimple(0.75f, 3.0f);
    glCornerFrom->setPose(poseFromCorner);
    glVizICP->insert(glCornerFrom);

    auto glCornerLocal =
        mrpt::opengl::stock_objects::CornerXYZSimple(0.85f, 5.0f);
    glCornerLocal->setPose(relativePose.mean);
    glCornerLocal->setName("Local");
    glCornerLocal->enableShowName(true);
    glVizICP->insert(glCornerLocal);

    auto glCornerToCov = mrpt::opengl::CEllipsoid3D::Create();
    glCornerToCov->set3DsegmentsCount(16);
    glCornerToCov->enableDrawSolid3D(true);
    glCornerToCov->setColor_u8(0xff, 0x00, 0x00, 0x40);
    //  std::cout << "cov:\n" << relativePose.cov << "\n";
    glCornerToCov->setCovMatrixAndMean(
        relativePose.cov.blockCopy<3, 3>(0, 0),
        relativePose.mean.asVectorVal().head<3>());
    glVizICP->insert(glCornerToCov);

    // GLOBAL PC:
    mp2p_icp::render_params_t rpGlobal;

    rpGlobal.points.visible = false;
    for (const auto& [lyName, cb] : cbLayersByName_global)
    {
        // Update stats in the cb label:
        cb->setCaption(lyName);  // default
        if (auto itL = lr.pcGlobal->layers.find(lyName);
            itL != lr.pcGlobal->layers.end())
        {
            if (auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                    itL->second);
                pc)
            {
                cb->setCaption(
                    lyName + " | "s +
                    mrpt::system::unitsFormat(static_cast<double>(pc->size())) +
                    " points"s + " | class="s +
                    pc->GetRuntimeClass()->className);
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
        rpGlobal.points.visible = true;

        auto& rpL                      = rpGlobal.points.perLayer[lyName];
        rpL.pointSize                  = slPointSize->value();
        rpL.render_voxelmaps_as_points = cbViewVoxelsAsPoints->checked();

        if (cbColorizeGlobalMap->checked())
        {
            auto& cm                  = rpL.colorMode.emplace();
            cm.colorMap               = mrpt::img::TColormap::cmHOT;
            cm.recolorizeByCoordinate = mp2p_icp::Coordinate::Z;
        }
    }

    static mrpt::opengl::CSetOfObjects::Ptr lastGlobalPts;

    if (!lastGlobalPts || regenerateMaps)
    {
        // Show all or selected layers:
        for (auto& rpL : rpGlobal.points.perLayer)
            rpL.second.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        auto glPts    = lr.pcGlobal->get_visualization(rpGlobal);
        lastGlobalPts = glPts;

        // Show all or selected layers:
        rpGlobal.points.allLayers.color =
            mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        glVizICP->insert(glPts);
    }
    else
    {
        // avoid the costly method of re-rendering maps, if possible:
        glVizICP->insert(lastGlobalPts);
    }

    // LOCAL PC:
    mp2p_icp::render_params_t rpLocal;

    rpLocal.points.visible = false;
    for (const auto& [lyName, cb] : cbLayersByName_local)
    {
        // Update stats in the cb label:
        cb->setCaption(lyName);  // default
        if (auto itL = lr.pcLocal->layers.find(lyName);
            itL != lr.pcLocal->layers.end())
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
        rpLocal.points.visible = true;

        auto& rpL                      = rpLocal.points.perLayer[lyName];
        rpL.pointSize                  = slPointSize->value();
        rpL.render_voxelmaps_as_points = cbViewVoxelsAsPoints->checked();
        if (cbColorizeLocalMap->checked())
        {
            auto& cm                  = rpL.colorMode.emplace();
            cm.colorMap               = mrpt::img::TColormap::cmHOT;
            cm.recolorizeByCoordinate = mp2p_icp::Coordinate::Z;
        }
    }

    static mrpt::opengl::CSetOfObjects::Ptr lastLocalPts;

    if (!lastLocalPts || regenerateMaps)
    {
        // Show all or selected layers:
        for (auto& rpL : rpLocal.points.perLayer)
            rpL.second.color = mrpt::img::TColor(0x00, 0x00, 0xff, 0xff);

        auto glPts   = lr.pcLocal->get_visualization(rpLocal);
        lastLocalPts = glPts;

        glVizICP->insert(glPts);
    }
    else
    {
        // avoid the costly method of re-rendering maps, if possible:
        glVizICP->insert(lastLocalPts);
    }
    lastLocalPts->setPose(relativePose.mean);

    // Global view options:
    {
        std::lock_guard<std::mutex> lck(win->background_scene_mtx);
        win->camera().setCameraProjective(!cbViewOrtho->checked());

        if (cbCameraFollowsLocal->checked())
        {
            win->camera().setCameraPointing(
                relativePose.mean.x(), relativePose.mean.y(),
                relativePose.mean.z());
        }

        // clip planes:
        const auto depthFieldMid = std::pow(10.0, slMidDepthField->value());
        const auto depthFieldThickness =
            std::pow(10.0, slThicknessDepthField->value());

        const auto clipNear =
            std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness);
        const auto clipFar = depthFieldMid + 0.5 * depthFieldThickness;

        lbDepthFieldMid->setCaption(
            mrpt::format("Center depth clip plane: %f", depthFieldMid));
        lbDepthFieldThickness->setCaption(
            mrpt::format("Max-Min depth thickness: %f", depthFieldThickness));
        lbDepthFieldValues->setCaption(mrpt::format(
            "Depth field: near plane=%f far plane=%f", clipNear, clipFar));

        win->background_scene->getViewport()->setViewportClipDistances(
            clipNear, clipFar);
    }

    // Pairings ------------------

    // viz pairings:
    if (pairsToViz)
    {
        mp2p_icp::pairings_render_params_t rp;

        rp.pt2pt.visible        = cbViewPairings_pt2pt->checked();
        rp.pt2pl.visible        = cbViewPairings_pt2pl->checked();
        rp.pt2pl.planePatchSize = std::pow(10.0, slPairingsPl2PlSize->value());

        rp.pt2ln.visible    = cbViewPairings_pt2ln->checked();
        rp.pt2ln.lineLength = std::pow(10.0, slPairingsPl2LnSize->value());

        glVizICP->insert(pairsToViz->get_visualization(relativePose.mean, rp));

        tbPairings->setValue(pairsToViz->contents_summary());
    }
    else
    {
        tbPairings->setValue(
            "None selected (mark one of the checkboxes below)");
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
