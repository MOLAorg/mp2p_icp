/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mp2p-icp-log-viewer/main.cpp
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
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

constexpr const char* APP_NAME = "mp2p-icp-log-viewer";

// =========== Declare supported cli switches ===========
static TCLAP::CmdLine cmd(APP_NAME);

static TCLAP::ValueArg<std::string> argExtension(
    "e", "file-extension",
    "Filename extension to look for. Default is `icplog`", false, "icplog",
    "icplog", cmd);

static TCLAP::ValueArg<std::string> argSearchDir(
    "d", "directory", "Directory in which to search for *.icplog files.", false,
    ".", ".", cmd);

static TCLAP::ValueArg<std::string> argVerbosity(
    "v", "verbose", "Verbosity level", false, "DEBUG", "DEBUG", cmd);

static TCLAP::ValueArg<double> argMinQuality(
    "", "min-quality",
    "Minimum quality (range [0,1]) for a log files to be loaded and shown in "
    "the list. Default=0 so all log files are visible.",
    false, 0.0, "Quality[0,1]", cmd);

// =========== Declare global variables ===========
#if MRPT_HAS_NANOGUI

auto glVizICP = mrpt::opengl::CSetOfObjects::Create();

nanogui::Slider* slSelectorICP   = nullptr;
nanogui::Button *btnSelectorBack = nullptr, *btnSelectorForw = nullptr;

std::array<nanogui::Label*, 4> lbICPStats = {
    nullptr, nullptr, nullptr, nullptr};
nanogui::CheckBox* cbShowInitialPose = nullptr;

nanogui::TextBox *tbLogPose = nullptr, *tbInitialGuess = nullptr,
                 *tbInit2Final = nullptr, *tbCovariance = nullptr,
                 *tbConditionNumber = nullptr;

nanogui::Slider* slPointSize = nullptr;

nanogui::Slider* slGTPose[6] = {nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr};

std::vector<std::string>        layerNames_global, layerNames_local;
std::vector<nanogui::CheckBox*> cbPointLayers_global, cbPointLayers_local;

std::vector<mp2p_icp::LogRecord> logRecords;

static void rebuild_3d_view();

static void main_show_gui()
{
    using namespace std::string_literals;

    const std::string searchDir = argSearchDir.getValue();
    ASSERT_DIRECTORY_EXISTS_(searchDir);

    std::cout << "Searching in: '" << searchDir
              << "' for files with extension '" << argExtension.getValue()
              << "'" << std::endl;

    mrpt::system::CDirectoryExplorer::TFileInfoList files;
    mrpt::system::CDirectoryExplorer::explore(
        searchDir, FILE_ATTRIB_ARCHIVE, files);
    mrpt::system::CDirectoryExplorer::filterByExtension(
        files, argExtension.getValue());
    mrpt::system::CDirectoryExplorer::sortByName(files);

    std::cout << "Found " << files.size() << " ICP records." << std::endl;

    // load files:
    size_t filesLoaded = 0, filesFilteredOut = 0;
    for (const auto& file : files)
    {
        std::cout << "Loading: " << file.wholePath << "...\n";
        const auto& lr = logRecords.emplace_back(
            mp2p_icp::LogRecord::LoadFromFile(file.wholePath));

        // Obtain layer info from first entry:
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

        filesLoaded++;

        // Filter by quality:
        if (lr.icpResult.quality < argMinQuality.getValue())
        {
            ++filesFilteredOut;
            // Remove last one:
            logRecords.erase(logRecords.rbegin().base());
        }
    }
    std::cout << "Loaded " << logRecords.size() << " ICP records ("
              << filesLoaded << " actually loaded, " << filesFilteredOut
              << " filtered out)" << std::endl;

    ASSERT_(!logRecords.empty());

    /*
     * -------------------------------------------------------------------
     * Plot 3D:
     * --------------------------------------------------------------------
     */
    nanogui::init();

    mrpt::gui::CDisplayWindowGUI_Params cp;
    cp.maximized = true;
    mrpt::gui::CDisplayWindowGUI win(APP_NAME, 1024, 800, cp);

    // Add a background scene:
    auto scene = mrpt::opengl::COpenGLScene::Create();

    {
        auto glGrid = mrpt::opengl::CGridPlaneXY::Create();
        glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
        scene->insert(glGrid);
    }

    auto gl_base = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
    gl_base->setName("Origin");
    gl_base->enableShowName();
    scene->insert(gl_base);

    scene->insert(glVizICP);

    {
        std::lock_guard<std::mutex> lck(win.background_scene_mtx);
        win.background_scene = std::move(scene);
    }

    // Control GUI sub-window:
    {
        auto w = win.createManagedSubWindow("Control");
        w->setPosition({5, 25});
        w->requestFocus();
        w->setLayout(new nanogui::BoxLayout(
            nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
        w->setFixedWidth(450);

        cbShowInitialPose =
            w->add<nanogui::CheckBox>("Show at INITIAL GUESS pose");
        cbShowInitialPose->setCallback([=](bool) { rebuild_3d_view(); });

        {
            auto pn = w->add<nanogui::Widget>();
            pn->setLayout(new nanogui::GridLayout(
                nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill));

            pn->add<nanogui::Label>("Point size");

            slPointSize = pn->add<nanogui::Slider>();
            slPointSize->setRange({1.0f, 10.0f});
            slPointSize->setValue(2.0f);
            slPointSize->setCallback([&](float) { rebuild_3d_view(); });
        }

        //
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Label>(mrpt::format(
            "Select ICP record file (N=%u)",
            static_cast<unsigned int>(logRecords.size())));
        slSelectorICP = w->add<nanogui::Slider>();
        slSelectorICP->setRange({.0f, logRecords.size() - 1});
        slSelectorICP->setValue(0);
        slSelectorICP->setCallback([&](float /*v*/) { rebuild_3d_view(); });

        for (auto& lb : lbICPStats) lb = w->add<nanogui::Label>("  ");

        // navigation panel:
        {
            auto pn = w->add<nanogui::Widget>();
            pn->setLayout(
                new nanogui::BoxLayout(nanogui::Orientation::Horizontal));

            // shortcut:
            auto s = slSelectorICP;

            btnSelectorBack = pn->add<nanogui::Button>(
                "", ENTYPO_ICON_CONTROLLER_FAST_BACKWARD);
            btnSelectorBack->setCallback([=]() {
                if (s->value() > 0)
                {
                    s->setValue(s->value() - 1);
                    s->callback()(s->value());
                }
            });

            btnSelectorForw = pn->add<nanogui::Button>(
                "", ENTYPO_ICON_CONTROLLER_FAST_FORWARD);
            btnSelectorForw->setCallback([=]() {
                if (s->value() < s->range().second - 0.01f)
                {
                    s->setValue(s->value() + 1);
                    s->callback()(s->value());
                }
            });
        }

        //
        w->add<nanogui::Label>(" ");  // separator

        auto tabWidget = w->add<nanogui::TabWidget>();

        auto* tab1 = tabWidget->createTab("ICP");
        tab1->setLayout(new nanogui::GroupLayout());

        auto* tab2 = tabWidget->createTab("Uncertainty");
        tab2->setLayout(new nanogui::GroupLayout());

        auto* tab3 = tabWidget->createTab("Layers");
        tab3->setLayout(new nanogui::GroupLayout());

        tabWidget->setActiveTab(0);

        tab1->add<nanogui::Label>(
            "ICP result pose [x y z yaw(deg) pitch(deg) roll(deg)]:");
        tbLogPose = tab1->add<nanogui::TextBox>();
        tbLogPose->setFontSize(16);
        tbLogPose->setEditable(true);

        tab1->add<nanogui::Label>("Initial -> final pose change:");
        tbInit2Final = tab1->add<nanogui::TextBox>();
        tbInit2Final->setFontSize(16);
        tbInit2Final->setEditable(false);

        tab2->add<nanogui::Label>(
            "Uncertainty: diagonal sigmas (x y z yaw pitch roll)");
        tbCovariance = tab2->add<nanogui::TextBox>();
        tbCovariance->setFontSize(16);
        tbCovariance->setEditable(false);

        tab2->add<nanogui::Label>("Uncertainty: Covariance condition numbers");
        tbConditionNumber = tab2->add<nanogui::TextBox>();
        tbConditionNumber->setFontSize(16);
        tbConditionNumber->setEditable(false);

        const float handTunedRange[6] = {4.0,        4.0,         10.0,
                                         0.5 * M_PI, 0.25 * M_PI, 0.5};

        for (int i = 0; i < 6; i++)
        {
            slGTPose[i] = tab2->add<nanogui::Slider>();
            slGTPose[i]->setRange({-handTunedRange[i], handTunedRange[i]});
            slGTPose[i]->setValue(0.0f);

            slGTPose[i]->setCallback([=](float v) {
                const size_t idx = mrpt::round(slSelectorICP->value());
                auto&        lr  = logRecords.at(idx);

                auto p = lr.icpResult.optimal_tf.mean.asTPose();
                p[i]   = v;
                lr.icpResult.optimal_tf.mean = mrpt::poses::CPose3D(p);

                rebuild_3d_view();
            });
        }

        tab1->add<nanogui::Label>("Initial guess pose:");
        tbInitialGuess = tab1->add<nanogui::TextBox>();
        tbInitialGuess->setFontSize(16);
        tbInitialGuess->setEditable(true);

        // Save map buttons:
        auto lambdaSave = [&](const mp2p_icp::metric_map_t& m) {
            const std::string outFile = nanogui::file_dialog(
                {{"mm",
                  "mp2p_icp::metric_map_t binary serialized object (*.mm)"}},
                true /*save*/);
            if (outFile.empty()) return;
            m.save_to_file(outFile);
        };

        tab1->add<nanogui::Label>(" ");
        tab1->add<nanogui::Button>("Export 'local' map...")->setCallback([&]() {
            const size_t idx = mrpt::round(slSelectorICP->value());
            auto&        lr  = logRecords.at(idx);
            ASSERT_(lr.pcLocal);
            lambdaSave(*lr.pcLocal);
        });
        tab1->add<nanogui::Button>("Export 'global' map...")
            ->setCallback([&]() {
                const size_t idx = mrpt::round(slSelectorICP->value());
                auto&        lr  = logRecords.at(idx);
                ASSERT_(lr.pcGlobal);
                lambdaSave(*lr.pcGlobal);
            });

        // tab 3:

        tab3->add<nanogui::Label>("[GLOBAL point cloud] Show point layers:");
        cbPointLayers_global.resize(layerNames_global.size());
        for (size_t i = 0; i < layerNames_global.size(); i++)
        {
            cbPointLayers_global[i] =
                tab3->add<nanogui::CheckBox>(layerNames_global.at(i));
            cbPointLayers_global[i]->setChecked(true);
            cbPointLayers_global[i]->setCallback(
                [](bool) { rebuild_3d_view(); });
        }

        tab3->add<nanogui::Label>("[LOCAL point cloud] Show point layers:");
        cbPointLayers_local.resize(layerNames_local.size());
        for (size_t i = 0; i < layerNames_local.size(); i++)
        {
            cbPointLayers_local[i] =
                tab3->add<nanogui::CheckBox>(layerNames_local.at(i));
            cbPointLayers_local[i]->setChecked(true);
            cbPointLayers_local[i]->setCallback(
                [](bool) { rebuild_3d_view(); });
        }

        // ----
        w->add<nanogui::Label>(" ");  // separator
        w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
            ->setCallback([&win]() { win.setVisible(false); });

        win.setKeyboardCallback([&](int key, [[maybe_unused]] int scancode,
                                    int                  action,
                                    [[maybe_unused]] int modifiers) {
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

    win.performLayout();
    win.camera().setCameraPointing(8.0f, .0f, .0f);
    win.camera().setAzimuthDegrees(110.0f);
    win.camera().setElevationDegrees(15.0f);
    win.camera().setZoomDistance(30.0f);

    rebuild_3d_view();

    // Main loop
    // ---------------------
    win.drawAll();
    win.setVisible(true);
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
// onHighlightOneICPEntry
// ==============================
void rebuild_3d_view()
{
    using namespace std::string_literals;

    const size_t idx = mrpt::round(slSelectorICP->value());

    btnSelectorBack->setEnabled(!logRecords.empty() && idx > 0);
    btnSelectorForw->setEnabled(
        !logRecords.empty() && idx < logRecords.size() - 1);

    glVizICP->clear();

    const auto& lr = logRecords.at(idx);

    lbICPStats[0]->setCaption(mrpt::format(
        "ICP pair #%u, local: ID:%u%s, global: ID:%u%s",
        static_cast<unsigned int>(idx),
        static_cast<unsigned int>(lr.pcLocal->id ? lr.pcLocal->id.value() : 0),
        lr.pcLocal->label ? lr.pcLocal->label.value().c_str() : "",
        static_cast<unsigned int>(
            lr.pcGlobal->id ? lr.pcGlobal->id.value() : 0),
        lr.pcGlobal->label ? lr.pcGlobal->label.value().c_str() : ""));

    lbICPStats[1]->setCaption(mrpt::format(
        "Log quality: %.02f%% iters: %u Term.Reason: %s",
        100.0 * lr.icpResult.quality,
        static_cast<unsigned int>(lr.icpResult.nIterations),
        mrpt::typemeta::enum2str(lr.icpResult.terminationReason).c_str()));

    lbICPStats[2]->setCaption("Global: "s + lr.pcGlobal->contents_summary());
    lbICPStats[3]->setCaption("Local: "s + lr.pcLocal->contents_summary());

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

    if (cbShowInitialPose->checked())
    {
        relativePose.mean = mrpt::poses::CPose3D(lr.initialGuessLocalWrtGlobal);
    }
    else
    {
        relativePose = lr.icpResult.optimal_tf;
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

    auto glCornerFrom =
        mrpt::opengl::stock_objects::CornerXYZSimple(0.75f, 3.0f);
    glCornerFrom->setPose(poseFromCorner);
    glVizICP->insert(glCornerFrom);

    auto glCornerTo = mrpt::opengl::stock_objects::CornerXYZSimple(0.85f, 5.0f);
    glCornerTo->setPose(relativePose.mean);
    glVizICP->insert(glCornerTo);

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

    for (const auto& cbPL : cbPointLayers_global)
    {
        if (!cbPL->checked()) continue;  // hidden

        auto& rpL     = rpGlobal.points.perLayer[cbPL->caption()];
        rpL.pointSize = slPointSize->value();
    }

    {
        // Show all or selected layers:
        for (auto& rpL : rpGlobal.points.perLayer)
            rpL.second.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        auto glPts = lr.pcGlobal->get_visualization(rpGlobal);

        // Show all or selected layers:
        rpGlobal.points.allLayers.color =
            mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

        glVizICP->insert(glPts);
    }

    // LOCAL PC:
    mp2p_icp::render_params_t rpLocal;

    for (const auto& cbPL : cbPointLayers_local)
    {
        if (!cbPL->checked()) continue;  // hidden

        auto& rpL     = rpLocal.points.perLayer[cbPL->caption()];
        rpL.pointSize = slPointSize->value();
    }

    {
        // Show all or selected layers:
        for (auto& rpL : rpLocal.points.perLayer)
            rpL.second.color = mrpt::img::TColor(0x00, 0x00, 0xff, 0xff);

        auto glPts = lr.pcLocal->get_visualization(rpLocal);

        glPts->setPose(relativePose.mean);
        glVizICP->insert(glPts);
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
