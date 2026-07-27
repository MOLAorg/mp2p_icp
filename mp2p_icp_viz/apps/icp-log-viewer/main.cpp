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
 * @file   icp-log-viewer/main.cpp
 * @brief  GUI tool to analyze ICP log records (*.icplog files)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 15, 2021
 */

// Must come before the MRPT math headers below: without nanogui (which used to pull this in
// transitively), MatrixVectorBase::head<N>() otherwise instantiates against an incomplete
// Eigen::Map forward declaration.
#include <Eigen/Dense>

// The goal is to visualize these guys:
#include <mp2p_icp/LogRecord.h>
// using this:
#include <mrpt/imgui/CImGuiSceneView.h>

// other deps:
#include <mrpt/config.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/round.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <imgui.h>
#include <imgui_app_common/ImGuiAppShell.h>
#include <imgui_app_common/SimpleFileDialog.h>
#include <imgui_internal.h>  // DockBuilder* API (default docking layout)

#include <CLI/CLI.hpp>
#include <iostream>

#include "../libcfgpath/cfgpath.h"

namespace
{
constexpr const char* APP_NAME = "icp-log-viewer";

constexpr const char* MINI_VIEW_NAME = "small-view";

// =========== Declare supported cli switches ===========
CLI::App cmd{APP_NAME};

std::string  argExtension = "icplog";
std::string  argSearchDir = ".";
std::string  argSingleFile;
std::string  arg_plugins;
double       argAutoPlayPeriod = 0.1;
double       argMinQuality     = 0.0;
CLI::Option* argMinQualityOpt  = nullptr;

/** Lazily loads a *.icplog file on first access, and can drop it again to free memory. */
class DelayedLoadLog
{
 public:
  DelayedLoadLog() = default;
  DelayedLoadLog(const std::string& fileName, const std::string& shortFileName)
      : filename_(fileName), shortFileName_(shortFileName)
  {
  }

  mp2p_icp::LogRecord& get()
  {
    if (!log_)
    {
      log_.emplace();
      if (!log_->load_from_file(filename_))
      {
        log_.reset();
        THROW_EXCEPTION_FMT("Failed to load log file: '%s'", filename_.c_str());
      }
    }

    return log_.value();
  }

  void dispose() { log_.reset(); }

  const std::string& filename() const { return filename_; }
  const std::string& shortFileName() const { return shortFileName_; }

 private:
  std::optional<mp2p_icp::LogRecord> log_;
  std::string                        filename_;
  std::string                        shortFileName_;
};

/** All mutable application state (see the analogous struct in mm-viewer/main.cpp for the same
 *  immediate-mode-GUI rationale). */
struct AppState
{
  mp2p_icp_viz::ImGuiAppShell  shell;
  mrpt::imgui::CImGuiSceneView sceneView;
  mrpt::opengl::Scene::Ptr     scene    = mrpt::opengl::COpenGLScene::Create();
  mrpt::opengl::CSetOfObjects::Ptr glVizICP = mrpt::opengl::CSetOfObjects::Create();

  std::vector<DelayedLoadLog> logRecords;

  std::vector<std::string> layerNamesGlobal;
  std::vector<std::string> layerNamesLocal;

  std::map<std::string, bool> layerVisibleGlobal;
  std::map<std::string, bool> layerVisibleLocal;

  // Selector / navigation:
  int    selectorIdx       = 0;
  bool   isAutoPlayActive  = false;
  double lastAutoPlayTime  = 0.0;

  // ICP iteration slider:
  bool  showInitialPose     = false;
  float iterationValue      = 0.0f;
  int   iterationMax         = 0;
  bool  iterationEnabled     = false;
  std::string iterationCaption = "Show ICP iteration:";

  // View options:
  bool  viewOrtho           = false;
  bool  cameraFollowsLocal  = false;
  bool  viewVoxelsAsPoints  = true;
  bool  viewVoxelsFreeSpace = false;
  bool  colorizeLocalMap    = false;
  bool  colorizeGlobalMap   = false;
  bool  viewPriorEllipsoid  = true;
  float globalPointSize     = 2.0f;
  float localPointSize      = 2.0f;
  float midDepthField       = 1.0f;
  float thicknessDepthField = 3.0f;

  // Pairings:
  bool  viewPairings           = false;
  bool  viewPairingsPt2Pt      = true;
  bool  viewPairingsPt2Pl      = true;
  bool  viewPairingsPt2Ln      = true;
  bool  viewPairingsCov2Cov    = true;
  float pairingsPl2PlSize      = -1.0f;
  float pairingsPl2LnSize      = -1.0f;
  float pairingsCov2CovDecim   = 500.0f;  // stored as log10, see slider range below
  std::string pairingsSummary  = "None selected (mark one of the checkboxes below)";

  // Manual pose tweak (6-DoF):
  float gtPose[6] = {0, 0, 0, 0, 0, 0};

  // Text/stat fields:
  std::string statFileName;
  std::string statLogSummary;
  std::string statQuality;
  std::string statGlobalSummary;
  std::string statLocalSummary;
  std::string statInitialGuess;
  std::string statPriorMean       = "(none)";
  std::string statPriorInfo       = "(none)";
  std::string statLogPose;
  std::string statInit2Final;
  std::string statCovariance;
  std::string statConditionNumber;

  static constexpr size_t          kMaxDynVariables = 100;
  std::vector<std::string>         dynVariableLines;

  mp2p_icp_viz::SimpleFileDialog exportDialog;
  int                             exportWhich = 0;  // 0=local, 1=global

  std::optional<size_t> lastIdx;
};

AppState app;

template <class MATRIX>
double conditionNumber(const MATRIX& m)
{
  MATRIX              eVecs;
  std::vector<double> eVals;
  m.eig_symmetric(eVecs, eVals);
  return eVals.back() / eVals.front();
}

void processAutoPlay()
{
  if (!app.isAutoPlayActive)
  {
    return;
  }

  const double tNow = mrpt::Clock::nowDouble();
  if (tNow - app.lastAutoPlayTime < argAutoPlayPeriod)
  {
    return;
  }
  app.lastAutoPlayTime = tNow;

  if (app.selectorIdx < static_cast<int>(app.logRecords.size()) - 1)
  {
    app.selectorIdx++;
  }
}

/** Creates the small XYZ axis-corner overlay viewport once. See the note in agents.md: not
 * currently rendered under MRPT 2.x (`CImGuiSceneView` only renders the "main" viewport), kept
 * for when this project moves to MRPT 3.x. */
void ensureMiniCornerViewport()
{
  if (app.scene->getViewport(MINI_VIEW_NAME))
  {
    return;
  }

  auto gl_view = app.scene->createViewport(MINI_VIEW_NAME);
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

void updateMiniCornerView()
{
  auto gl_view = app.scene->getViewport(MINI_VIEW_NAME);
  if (!gl_view)
  {
    return;
  }
  mrpt::opengl::CCamera& view_cam = gl_view->getCamera();
  view_cam.setAzimuthDegrees(app.sceneView.camera().getAzimuthDegrees());
  view_cam.setElevationDegrees(app.sceneView.camera().getElevationDegrees());
  view_cam.setZoomDistance(5);
}

void handleKeyboard()
{
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput)
  {
    return;
  }

  int increment = 0;
  if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
  {
    increment = -1;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
  {
    increment = +1;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_PageDown, true))
  {
    increment = +100;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_PageUp, true))
  {
    increment = -100;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
  {
    app.isAutoPlayActive = !app.isAutoPlayActive;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_I, false))
  {
    app.showInitialPose = !app.showInitialPose;
  }

  if (increment != 0)
  {
    app.selectorIdx =
        std::clamp(app.selectorIdx + increment, 0, static_cast<int>(app.logRecords.size()) - 1);
  }
}

/** Equivalent to the old `rebuild_3d_view(bool regenerateMaps)`: called unconditionally every
 * frame; regeneration of the (expensive) point-cloud OpenGL representations is only actually
 * performed when the selected record or the relevant render params changed since the last call
 * (see `indexChanged`/`regenGlobal`/`regenLocal` below), mirroring mm-viewer's approach. */
void rebuild_3d_view()
try
{
  using namespace std::string_literals;

  const size_t idx = static_cast<size_t>(app.selectorIdx);
  if (idx >= app.logRecords.size())
  {
    return;
  }

  const bool indexChanged = !app.lastIdx.has_value() || *app.lastIdx != idx;
  if (indexChanged)
  {
    if (app.lastIdx.has_value())
    {
      app.logRecords.at(*app.lastIdx).dispose();
    }
    app.lastIdx = idx;
  }

  app.glVizICP->clear();

  // lazy load from disk happens in the "get()":
  const auto& lr = app.logRecords.at(idx).get();

  app.statFileName = app.logRecords.at(idx).shortFileName();

  app.statLogSummary = mrpt::format(
      "ICP log #%zu | Local: ID:%u%s | Global: ID:%u%s", idx,
      static_cast<unsigned int>(lr.pcLocal->id ? lr.pcLocal->id.value() : 0),
      lr.pcLocal->label ? lr.pcLocal->label.value().c_str() : "",
      static_cast<unsigned int>(lr.pcGlobal->id ? lr.pcGlobal->id.value() : 0),
      lr.pcGlobal->label ? lr.pcGlobal->label.value().c_str() : "");

  app.statQuality = mrpt::format(
      "Quality: %.02f%% | Iters: %u | Term.Reason: %s", 100.0 * lr.icpResult.quality,
      static_cast<unsigned int>(lr.icpResult.nIterations),
      mrpt::typemeta::enum2str(lr.icpResult.terminationReason).c_str());

  app.statGlobalSummary = "Global: "s + lr.pcGlobal->contents_summary();
  app.statLocalSummary  = "Local: "s + lr.pcLocal->contents_summary();

  app.statInitialGuess = lr.initialGuessLocalWrtGlobal.asString();

  if (lr.prior.has_value())
  {
    app.statPriorMean = lr.prior->mean.asString();

    std::string s;
    const auto  cov = mrpt::math::CMatrixDouble66(lr.prior->cov_inv.inverse());
    for (int i = 0; i < 6; i++)
    {
      const auto sigma = std::sqrt(cov(i, i));
      s += (i < 3) ? mrpt::format("%.02fm ", sigma) : mrpt::format("%.02fd ", mrpt::RAD2DEG(sigma));
    }
    app.statPriorInfo = s;
  }
  else
  {
    app.statPriorMean = "(none)";
    app.statPriorInfo = "(none)";
  }

  app.statLogPose = lr.icpResult.optimal_tf.mean.asString();

  // dyn variables:
  app.dynVariableLines.clear();
  for (const auto& [name, value] : lr.dynamicVariables)
  {
    app.dynVariableLines.push_back(mrpt::format("%s = %g", name.c_str(), value));
    if (app.dynVariableLines.size() >= AppState::kMaxDynVariables)
    {
      break;
    }
  }

  {
    const auto poseChange =
        lr.icpResult.optimal_tf.mean - mrpt::poses::CPose3D(lr.initialGuessLocalWrtGlobal);

    app.statInit2Final = mrpt::format(
        "|T|=%.03f [m]  |R|=%.03f [deg]", poseChange.norm(),
        mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(poseChange.getRotationMatrix()).norm()));
  }

  const auto                      poseFromCorner = mrpt::poses::CPose3D::Identity();
  mrpt::poses::CPose3DPDFGaussian relativePose;
  const mp2p_icp::Pairings*       pairsToViz = nullptr;

  if (!lr.iterationsDetails.has_value() || lr.iterationsDetails->empty())
  {
    app.iterationEnabled = false;

    if (app.showInitialPose)
    {
      relativePose.mean  = mrpt::poses::CPose3D(lr.initialGuessLocalWrtGlobal);
      app.iterationCaption = "Show ICP iteration: INITIAL";
    }
    else
    {
      relativePose        = lr.icpResult.optimal_tf;
      app.iterationCaption = "Show ICP iteration: FINAL";
    }

    if (app.viewPairings)
    {
      pairsToViz = &lr.icpResult.finalPairings;
    }
  }
  else
  {
    app.iterationEnabled = true;
    app.iterationMax     = static_cast<int>(lr.iterationsDetails->size()) - 1;

    if (indexChanged)
    {
      app.iterationValue = app.showInitialPose ? 0.0f : static_cast<float>(app.iterationMax);
    }

    auto   it    = lr.iterationsDetails->begin();
    size_t nIter = mrpt::round(app.iterationValue);
    mrpt::keep_min(nIter, lr.iterationsDetails->size() - 1);
    std::advance(it, static_cast<std::ptrdiff_t>(nIter));

    relativePose.mean = it->second.optimalPose;
    if (app.viewPairings)
    {
      pairsToViz = &it->second.pairings;
    }

    app.iterationCaption = "Show ICP iteration: "s + std::to_string(it->first) + "/"s +
                            std::to_string(lr.iterationsDetails->rbegin()->first);
  }

  {
    std::string s;
    for (int i = 0; i < 6; i++)
    {
      const auto sigma = std::sqrt(relativePose.cov(i, i));
      s += (i < 3) ? mrpt::format("%.02fm ", sigma) : mrpt::format("%.02fd ", mrpt::RAD2DEG(sigma));
    }
    s += mrpt::format(
        " det(XYZ)=%.02e det(rot)=%.02e", relativePose.cov.blockCopy<3, 3>(0, 0).det(),
        relativePose.cov.blockCopy<3, 3>(3, 3).det());
    app.statCovariance = s;
  }

  const mrpt::poses::CPosePDFGaussian pose2D(relativePose);

  app.statConditionNumber = mrpt::format(
      " cn{XYZ}=%.02f cn{SO(3)}=%.02f cn{SE(2)}=%.02f cn{SE(3)}=%.02f",
      conditionNumber(relativePose.cov.blockCopy<3, 3>(0, 0)),
      conditionNumber(relativePose.cov.blockCopy<3, 3>(3, 3)), conditionNumber(pose2D.cov),
      conditionNumber(relativePose.cov));

  // 3D objects -------------------
  auto glCornerFrom = mrpt::opengl::stock_objects::CornerXYZSimple(0.75f, 3.0f);
  glCornerFrom->setPose(poseFromCorner);
  app.glVizICP->insert(glCornerFrom);

  auto glCornerLocal = mrpt::opengl::stock_objects::CornerXYZSimple(0.85f, 5.0f);
  glCornerLocal->setPose(relativePose.mean);
  glCornerLocal->setName("Local");
  glCornerLocal->enableShowName(true);
  app.glVizICP->insert(glCornerLocal);

  auto glCornerToCov = mrpt::opengl::CEllipsoid3D::Create();
  glCornerToCov->set3DsegmentsCount(16);
  glCornerToCov->enableDrawSolid3D(true);
  glCornerToCov->setColor_u8(0xff, 0x00, 0x00, 0x40);
  glCornerToCov->setCovMatrixAndMean(
      relativePose.cov.blockCopy<3, 3>(0, 0), relativePose.mean.asVectorVal().head<3>());
  app.glVizICP->insert(glCornerToCov);

  if (lr.prior.has_value() && app.viewPriorEllipsoid)
  {
    const auto priorCov = mrpt::math::CMatrixDouble66(lr.prior->cov_inv.inverse());

    auto glPriorEllipsoid = mrpt::opengl::CEllipsoid3D::Create();
    glPriorEllipsoid->set3DsegmentsCount(16);
    glPriorEllipsoid->enableDrawSolid3D(true);
    glPriorEllipsoid->setColor_u8(0xff, 0xff, 0x00, 0x50);
    glPriorEllipsoid->setCovMatrixAndMean(
        priorCov.blockCopy<3, 3>(0, 0), lr.prior->mean.asVectorVal().head<3>());
    glPriorEllipsoid->setName("Prior");
    glPriorEllipsoid->enableShowName(true);
    app.glVizICP->insert(glPriorEllipsoid);
  }

  // GLOBAL PC:
  mp2p_icp::render_params_t rpGlobal;
  rpGlobal.points.visible = false;
  for (const auto& lyName : app.layerNamesGlobal)
  {
    if (!app.layerVisibleGlobal[lyName])
    {
      continue;
    }
    rpGlobal.points.visible = true;

    auto& rpL                       = rpGlobal.points.perLayer[lyName];
    rpL.pointSize                   = app.globalPointSize;
    rpL.render_voxelmaps_as_points  = app.viewVoxelsAsPoints;
    rpL.render_voxelmaps_free_space = app.viewVoxelsFreeSpace;

    if (app.colorizeGlobalMap)
    {
      auto& cm             = rpL.colorMode.emplace();
      cm.colorMap          = mrpt::img::TColormap::cmHOT;
      cm.recolorizeByField = "z";
    }
  }
  for (auto& rpL : rpGlobal.points.perLayer)
  {
    rpL.second.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);
  }
  rpGlobal.points.allLayers.color = mrpt::img::TColor(0xff, 0x00, 0x00, 0xff);

  static std::optional<mp2p_icp::render_params_t> prevRpGlobal;
  static mrpt::opengl::CSetOfObjects::Ptr          lastGlobalPts;

  if (indexChanged || !prevRpGlobal.has_value() || *prevRpGlobal != rpGlobal)
  {
    prevRpGlobal  = rpGlobal;
    lastGlobalPts = lr.pcGlobal->get_visualization(rpGlobal);
  }
  app.glVizICP->insert(lastGlobalPts);

  // LOCAL PC:
  mp2p_icp::render_params_t rpLocal;
  rpLocal.points.visible = false;
  for (const auto& lyName : app.layerNamesLocal)
  {
    if (!app.layerVisibleLocal[lyName])
    {
      continue;
    }
    rpLocal.points.visible = true;

    auto& rpL                       = rpLocal.points.perLayer[lyName];
    rpL.pointSize                   = app.localPointSize;
    rpL.render_voxelmaps_as_points  = app.viewVoxelsAsPoints;
    rpL.render_voxelmaps_free_space = app.viewVoxelsFreeSpace;

    if (app.colorizeLocalMap)
    {
      auto& cm             = rpL.colorMode.emplace();
      cm.colorMap          = mrpt::img::TColormap::cmHOT;
      cm.recolorizeByField = "z";
    }
  }
  for (auto& rpL : rpLocal.points.perLayer)
  {
    rpL.second.color = mrpt::img::TColor(0x00, 0x00, 0xff, 0xff);
  }

  static std::optional<mp2p_icp::render_params_t> prevRpLocal;
  static mrpt::opengl::CSetOfObjects::Ptr          lastLocalPts;

  if (indexChanged || !prevRpLocal.has_value() || *prevRpLocal != rpLocal)
  {
    prevRpLocal  = rpLocal;
    lastLocalPts = lr.pcLocal->get_visualization(rpLocal);
  }
  app.glVizICP->insert(lastLocalPts);
  lastLocalPts->setPose(relativePose.mean);

  // Global view options:
  auto& cam = app.sceneView.camera();
  cam.setProjectiveModel(!app.viewOrtho);

  if (app.cameraFollowsLocal)
  {
    const auto camLoc = relativePose.mean.translation().cast<float>();
    cam.setPointingAt(camLoc.x, camLoc.y, camLoc.z);
  }

  const auto depthFieldMid       = std::pow(10.0, app.midDepthField);
  const auto depthFieldThickness = std::pow(10.0, app.thicknessDepthField);
  const auto clipNear            = std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness);
  const auto clipFar             = depthFieldMid + 0.5 * depthFieldThickness;

  app.scene->getViewport("main")->setViewportClipDistances(
      static_cast<float>(clipNear), static_cast<float>(clipFar));

  // Pairings ------------------
  if (pairsToViz)
  {
    const double planeCovScale = std::pow(10.0, app.pairingsPl2PlSize);

    mp2p_icp::pairings_render_params_t rp;
    rp.pt2pt.visible        = app.viewPairingsPt2Pt;
    rp.pt2pl.visible        = app.viewPairingsPt2Pl;
    rp.pt2pl.planePatchSize = planeCovScale;

    rp.cov2cov.visible    = app.viewPairingsCov2Cov;
    rp.cov2cov.decimation = static_cast<std::size_t>(std::pow(10.0, app.pairingsCov2CovDecim));
    rp.cov2cov.covScale   = planeCovScale;

    rp.pt2ln.visible    = app.viewPairingsPt2Ln;
    rp.pt2ln.lineLength = std::pow(10.0, app.pairingsPl2LnSize);

    app.glVizICP->insert(pairsToViz->get_visualization(relativePose.mean, rp));
    app.pairingsSummary = pairsToViz->contents_summary();
  }
  else
  {
    app.pairingsSummary = "None selected (mark one of the checkboxes below)";
  }

  ensureMiniCornerViewport();
}
catch (const std::exception& e)
{
  std::cerr << "[rebuild_3d_view] Exception: " << mrpt::exception_to_str(e) << std::endl;
  app.statFileName = std::string("ERROR: ") + mrpt::exception_to_str(e).substr(0, 80);
}

void onExport(const std::string& outFile)
{
  const size_t idx = static_cast<size_t>(app.selectorIdx);
  auto&        lr  = app.logRecords.at(idx).get();

  const mp2p_icp::metric_map_t& m = (app.exportWhich == 0) ? *lr.pcLocal : *lr.pcGlobal;
  if (bool ok = m.save_to_file(outFile); !ok)
  {
    std::cerr << "Error saving file: " << outFile << "\n";
  }
}

/** "Control" window: file selector, navigation, iteration slider, summary stat lines. */
void renderControlPanel()
{
  ImGui::Begin("Control");

  ImGui::Checkbox("Show at INITIAL GUESS pose", &app.showInitialPose);

  ImGui::BeginDisabled(!app.iterationEnabled);
  ImGui::TextUnformatted(app.iterationCaption.c_str());
  ImGui::SetNextItemWidth(-1);
  ImGui::SliderFloat("##iterationSlider", &app.iterationValue, 0.0f, static_cast<float>(std::max(0, app.iterationMax)));
  ImGui::EndDisabled();

  ImGui::Separator();
  ImGui::Text("Select ICP record file (N=%u)", static_cast<unsigned int>(app.logRecords.size()));
  ImGui::SetNextItemWidth(-1);
  ImGui::SliderInt("##selectorIdx", &app.selectorIdx, 0, static_cast<int>(app.logRecords.size()) - 1);

  ImGui::TextUnformatted(app.statFileName.c_str());
  ImGui::TextUnformatted(app.statLogSummary.c_str());
  ImGui::TextUnformatted(app.statQuality.c_str());
  ImGui::TextWrapped("%s", app.statGlobalSummary.c_str());
  ImGui::TextWrapped("%s", app.statLocalSummary.c_str());

  ImGui::BeginDisabled(app.selectorIdx <= 0);
  if (ImGui::Button("<<"))
  {
    app.selectorIdx--;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(app.selectorIdx >= static_cast<int>(app.logRecords.size()) - 1);
  if (ImGui::Button(">>"))
  {
    app.selectorIdx++;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::Checkbox("Autoplay", &app.isAutoPlayActive);

  ImGui::End();
}

/** "Summary" window: ICP result pose, deltas, covariance, prior. */
void renderSummaryPanel()
{
  ImGui::Begin("Summary");

  ImGui::TextUnformatted("ICP result pose [x y z yaw(deg) pitch(deg) roll(deg)]:");
  ImGui::TextWrapped("%s", app.statLogPose.c_str());

  ImGui::TextUnformatted("Initial -> final pose change:");
  ImGui::TextWrapped("%s", app.statInit2Final.c_str());

  ImGui::TextUnformatted("Uncertainty: diagonal sigmas (x y z [m] yaw pitch roll [deg])");
  ImGui::TextWrapped("%s", app.statCovariance.c_str());

  ImGui::TextUnformatted("Uncertainty: Covariance condition numbers");
  ImGui::TextWrapped("%s", app.statConditionNumber.c_str());

  ImGui::Separator();
  ImGui::TextUnformatted("Initial guess pose:");
  ImGui::TextWrapped("%s", app.statInitialGuess.c_str());

  ImGui::TextUnformatted("Prior mean pose (if any):");
  ImGui::TextWrapped("%s", app.statPriorMean.c_str());

  ImGui::TextUnformatted("Prior: diagonal sigmas (x y z [m] yaw pitch roll [deg])");
  ImGui::TextWrapped("%s", app.statPriorInfo.c_str());

  ImGui::End();
}

/** "Variables" window: dynamic (YAML-expression) variables active in this ICP record. */
void renderVariablesPanel()
{
  ImGui::Begin("Variables");
  ImGui::TextUnformatted("Dynamic variables:");
  if (ImGui::BeginChild("##dynVars", ImVec2(0, 0), true))
  {
    for (const auto& line : app.dynVariableLines)
    {
      ImGui::TextUnformatted(line.c_str());
    }
  }
  ImGui::EndChild();
  ImGui::End();
}

/** "Maps" window: global/local layer visibility + export buttons. */
void renderMapsPanel()
{
  ImGui::Begin("Maps");

  if (ImGui::Button("Export 'local' map..."))
  {
    app.exportWhich = 0;
    app.exportDialog.open(mp2p_icp_viz::SimpleFileDialog::Mode::Save, {{"mm", "(*.mm)"}}, "Export local map");
  }
  ImGui::SameLine();
  if (ImGui::Button("Export 'global' map..."))
  {
    app.exportWhich = 1;
    app.exportDialog.open(mp2p_icp_viz::SimpleFileDialog::Mode::Save, {{"mm", "(*.mm)"}}, "Export global map");
  }
  if (auto path = app.exportDialog.render(); path.has_value())
  {
    onExport(*path);
  }

  ImGui::Separator();
  ImGui::TextUnformatted("[GLOBAL map] Visible layers:");
  ImGui::PushID("globalLayers");
  for (const auto& lyName : app.layerNamesGlobal)
  {
    ImGui::Checkbox(lyName.c_str(), &app.layerVisibleGlobal[lyName]);
  }
  ImGui::PopID();

  ImGui::Separator();
  ImGui::TextUnformatted("[LOCAL map] Visible layers:");
  ImGui::PushID("localLayers");
  for (const auto& lyName : app.layerNamesLocal)
  {
    ImGui::Checkbox(lyName.c_str(), &app.layerVisibleLocal[lyName]);
  }
  ImGui::PopID();

  ImGui::End();
}

/** "Pairings" window: pairing type toggles, sizes/decimation, summary. */
void renderPairingsPanel()
{
  ImGui::Begin("Pairings");

  ImGui::TextWrapped("%s", app.pairingsSummary.c_str());

  ImGui::Checkbox("View pairings", &app.viewPairings);

  ImGui::Checkbox("View: point-to-point", &app.viewPairingsPt2Pt);

  ImGui::Checkbox("View: cov-to-cov", &app.viewPairingsCov2Cov);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(140);
  ImGui::SliderFloat("Decimation", &app.pairingsCov2CovDecim, 0.0f, 3.0f);

  ImGui::Checkbox("View: point-to-plane", &app.viewPairingsPt2Pl);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(140);
  ImGui::SliderFloat("Plane size", &app.pairingsPl2PlSize, -4.0f, 2.0f);

  ImGui::Checkbox("View: point-to-line", &app.viewPairingsPt2Ln);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(140);
  ImGui::SliderFloat("Line length", &app.pairingsPl2LnSize, -2.0f, 2.0f);

  ImGui::End();
}

/** "View" window: point sizes, depth clip, ortho, camera-follow, voxel/colorize options. */
void renderViewPanel()
{
  ImGui::Begin("View");

  ImGui::SliderFloat("Global map point size", &app.globalPointSize, 1.0f, 10.0f);
  ImGui::SliderFloat("Local map point size", &app.localPointSize, 1.0f, 10.0f);

  ImGui::SliderFloat("Center depth clip plane", &app.midDepthField, -2.0f, 3.0f);
  ImGui::SliderFloat("Max-Min depth thickness", &app.thicknessDepthField, -2.0f, 6.0f);

  const auto depthFieldMid       = std::pow(10.0, app.midDepthField);
  const auto depthFieldThickness = std::pow(10.0, app.thicknessDepthField);
  const auto clipNear            = std::max(1e-2, depthFieldMid - 0.5 * depthFieldThickness);
  const auto clipFar             = depthFieldMid + 0.5 * depthFieldThickness;
  ImGui::Text("Frustum: near=%.02f far=%.02f", clipNear, clipFar);

  ImGui::Checkbox("Orthogonal view", &app.viewOrtho);
  ImGui::Checkbox("Camera follows 'local'", &app.cameraFollowsLocal);
  ImGui::Checkbox("Render voxel maps as point clouds", &app.viewVoxelsAsPoints);
  ImGui::Checkbox("Render free space of voxel maps", &app.viewVoxelsFreeSpace);
  ImGui::Checkbox("Recolorize local map", &app.colorizeLocalMap);
  ImGui::Checkbox("Recolorize global map", &app.colorizeGlobalMap);
  ImGui::Checkbox("View prior ellipsoid", &app.viewPriorEllipsoid);

  ImGui::End();
}

/** "Manual" window: 6-DoF manual nudge of the current record's optimal_tf pose. */
void renderManualPanel()
{
  ImGui::Begin("Manual");
  ImGui::TextUnformatted("Manual solution modification:");

  constexpr float           handTunedRange[6] = {4.0f, 4.0f, 10.0f, 0.5f * static_cast<float>(M_PI),
                                        0.25f * static_cast<float>(M_PI), 0.5f};
  constexpr const char*     labels[6]         = {"X", "Y", "Z", "Yaw", "Pitch", "Roll"};

  const size_t idx = static_cast<size_t>(app.selectorIdx);
  for (int i = 0; i < 6; i++)
  {
    if (ImGui::SliderFloat(labels[i], &app.gtPose[i], -handTunedRange[i], handTunedRange[i]) &&
        idx < app.logRecords.size())
    {
      auto& lr = app.logRecords.at(idx).get();
      auto  p  = lr.icpResult.optimal_tf.mean.asTPose();
      p[i]     = app.gtPose[i];
      lr.icpResult.optimal_tf.mean = mrpt::poses::CPose3D(p);
    }
  }

  ImGui::End();
}

void renderSceneWindow()
{
  ImGui::Begin("3D View");
  app.sceneView.render();
  ImGui::End();
}

void renderFrame()
{
  handleKeyboard();
  processAutoPlay();
  rebuild_3d_view();
  updateMiniCornerView();

  renderControlPanel();
  renderSummaryPanel();
  renderVariablesPanel();
  renderMapsPanel();
  renderPairingsPanel();
  renderViewPanel();
  renderManualPanel();
  renderSceneWindow();
}

int mainShowGui()
{
  mrpt::system::CDirectoryExplorer::TFileInfoList files;

  if (argSingleFile.empty())
  {
    const std::string searchDir = argSearchDir;
    ASSERT_DIRECTORY_EXISTS_(searchDir);

    std::cout << "Searching in: '" << searchDir << "' for files with extension '" << argExtension
               << "'" << std::endl;

    mrpt::system::CDirectoryExplorer::explore(searchDir, FILE_ATTRIB_ARCHIVE, files);
    mrpt::system::CDirectoryExplorer::filterByExtension(files, argExtension);
    mrpt::system::CDirectoryExplorer::sortByName(files);

    std::cout << "Found " << files.size() << " ICP records." << std::endl;
  }
  else
  {
    std::cout << "Loading one single log file: " << argSingleFile << std::endl;
    files.resize(1);
    files[0].wholePath = argSingleFile;
  }

  if (argMinQualityOpt && argMinQualityOpt->count() > 0)
  {
    const double minQ = argMinQuality;
    if (minQ < 0.0 || minQ > 1.0)
    {
      THROW_EXCEPTION_FMT("--min-quality must be in [0,1]. Got: %.03f", minQ);
    }
    std::cout << "Applying minimum quality filter: q >= " << minQ << std::endl;

    mrpt::system::CDirectoryExplorer::TFileInfoList filteredFiles;
    for (const auto& file : files)
    {
      mp2p_icp::LogRecord lr;
      if (!lr.load_from_file(file.wholePath))
      {
        std::cerr << "  Warning: could not load '" << file.wholePath << "'" << std::endl;
        continue;
      }
      if (lr.icpResult.quality >= minQ)
      {
        filteredFiles.push_back(file);
      }
      else
      {
        std::cout << "  Skipping (quality=" << lr.icpResult.quality << " < " << minQ
                   << "): " << file.name << std::endl;
      }
    }
    std::cout << "Quality filter: kept " << filteredFiles.size() << " / " << files.size()
               << " files." << std::endl;

    if (files.empty())
    {
      THROW_EXCEPTION_FMT(
          "No log files passed --min-quality=%.03f. Lower the threshold or check input logs.",
          minQ);
    }
    files = std::move(filteredFiles);
  }

  for (const auto& file : files)
  {
    app.logRecords.emplace_back(file.wholePath, file.name);
  }
  ASSERT_(!app.logRecords.empty());

  {
    const auto& lr = app.logRecords.front().get();
    if (app.layerNamesGlobal.empty() && lr.pcGlobal)
    {
      for (const auto& layer : lr.pcGlobal->layers)
      {
        app.layerNamesGlobal.push_back(layer.first);
        app.layerVisibleGlobal[layer.first] = true;
        std::cout << "Global point cloud: Found point layer='" << layer.first << "'\n";
      }
    }
    if (app.layerNamesLocal.empty() && lr.pcLocal)
    {
      for (const auto& layer : lr.pcLocal->layers)
      {
        app.layerNamesLocal.push_back(layer.first);
        app.layerVisibleLocal[layer.first] = true;
        std::cout << "Local point cloud: Found point layer='" << layer.first << "'\n";
      }
    }
  }

  if (!app.shell.init(APP_NAME, 1280, 800))
  {
    return 1;
  }

  // Default docking layout (only applied once, when no imgui.ini layout exists yet):
  // "Control" as a thin left strip, the option panels tabbed below it, "3D View" filling the
  // remaining (background) area.
  app.shell.setupDefaultLayout = [](unsigned int dockspaceId)
  {
    ImGuiID rightId = 0;
    ImGuiID leftId  = ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.30f, nullptr, &rightId);

    ImGuiID leftBottomId = 0;
    ImGuiID leftTopId =
        ImGui::DockBuilderSplitNode(leftId, ImGuiDir_Up, 0.4f, nullptr, &leftBottomId);

    ImGui::DockBuilderDockWindow("Control", leftTopId);
    ImGui::DockBuilderDockWindow("Summary", leftBottomId);
    ImGui::DockBuilderDockWindow("Variables", leftBottomId);
    ImGui::DockBuilderDockWindow("Maps", leftBottomId);
    ImGui::DockBuilderDockWindow("Pairings", leftBottomId);
    ImGui::DockBuilderDockWindow("View", leftBottomId);
    ImGui::DockBuilderDockWindow("Manual", leftBottomId);
    ImGui::DockBuilderDockWindow("3D View", rightId);
  };

  // Background scene:
  {
    auto glGrid = mrpt::opengl::CGridPlaneXY::Create();
    glGrid->setColor_u8(0xff, 0xff, 0xff, 0x10);
    app.scene->insert(glGrid);
  }

  auto glBase = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
  glBase->setName("Global");
  glBase->enableShowName();
  app.scene->insert(glBase);

  app.scene->insert(app.glVizICP);

  app.sceneView.setScene(app.scene);

  app.sceneView.camera().setPointingAt(8.0f, 0.0f, 0.0f);
  app.sceneView.camera().setAzimuthDegrees(110.0f);
  app.sceneView.camera().setElevationDegrees(15.0f);
  app.sceneView.camera().setZoomDistance(30.0f);

  // Load/save persistent UI+camera state across sessions:
  char appCfgFile[1024];
  ::get_user_config_file(appCfgFile, sizeof(appCfgFile), APP_NAME);
  mrpt::config::CConfigFile appCfg(appCfgFile);

  auto& cam                = app.sceneView.camera();
  app.colorizeLocalMap     = appCfg.read_bool("", "cbColorizeLocalMap", app.colorizeLocalMap);
  app.colorizeGlobalMap    = appCfg.read_bool("", "cbColorizeGlobalMap", app.colorizeGlobalMap);
  app.showInitialPose      = appCfg.read_bool("", "cbShowInitialPose", app.showInitialPose);
  app.viewOrtho            = appCfg.read_bool("", "cbViewOrtho", app.viewOrtho);
  app.cameraFollowsLocal   = appCfg.read_bool("", "cbCameraFollowsLocal", app.cameraFollowsLocal);
  app.viewVoxelsAsPoints   = appCfg.read_bool("", "cbViewVoxelsAsPoints", app.viewVoxelsAsPoints);
  app.viewPairings         = appCfg.read_bool("", "cbViewPairings", app.viewPairings);
  app.viewPairingsPt2Pt    = appCfg.read_bool("", "cbViewPairings_pt2pt", app.viewPairingsPt2Pt);
  app.viewPairingsPt2Pl    = appCfg.read_bool("", "cbViewPairings_pt2pl", app.viewPairingsPt2Pl);
  app.viewPairingsPt2Ln    = appCfg.read_bool("", "cbViewPairings_pt2ln", app.viewPairingsPt2Ln);
  app.viewPairingsCov2Cov  = appCfg.read_bool("", "cbViewPairings_cov2cov", app.viewPairingsCov2Cov);
  app.viewPriorEllipsoid   = appCfg.read_bool("", "cbViewPriorEllipsoid", app.viewPriorEllipsoid);

  app.pairingsPl2PlSize    = appCfg.read_float("", "slPairingsPl2PlSize", app.pairingsPl2PlSize);
  app.pairingsPl2LnSize    = appCfg.read_float("", "slPairingsPl2LnSize", app.pairingsPl2LnSize);
  app.pairingsCov2CovDecim = appCfg.read_float("", "slPairingsCov2CovDecimation", app.pairingsCov2CovDecim);
  app.globalPointSize      = appCfg.read_float("", "slGlobalPointSize", app.globalPointSize);
  app.localPointSize       = appCfg.read_float("", "slLocalPointSize", app.localPointSize);
  app.midDepthField        = appCfg.read_float("", "slMidDepthField", app.midDepthField);
  app.thicknessDepthField  = appCfg.read_float("", "slThicknessDepthField", app.thicknessDepthField);

  cam.setPointingAt(
      appCfg.read_float("", "cam_x", cam.getPointingAtX()),
      appCfg.read_float("", "cam_y", cam.getPointingAtY()),
      appCfg.read_float("", "cam_z", cam.getPointingAtZ()));
  cam.setAzimuthDegrees(appCfg.read_float("", "cam_az", cam.getAzimuthDegrees()));
  cam.setElevationDegrees(appCfg.read_float("", "cam_el", cam.getElevationDegrees()));
  cam.setZoomDistance(appCfg.read_float("", "cam_d", cam.getZoomDistance()));

  app.shell.run(&renderFrame);

  appCfg.write("", "cbColorizeLocalMap", app.colorizeLocalMap);
  appCfg.write("", "cbColorizeGlobalMap", app.colorizeGlobalMap);
  appCfg.write("", "cbShowInitialPose", app.showInitialPose);
  appCfg.write("", "cbViewOrtho", app.viewOrtho);
  appCfg.write("", "cbCameraFollowsLocal", app.cameraFollowsLocal);
  appCfg.write("", "cbViewVoxelsAsPoints", app.viewVoxelsAsPoints);
  appCfg.write("", "cbViewPairings", app.viewPairings);
  appCfg.write("", "cbViewPairings_pt2pt", app.viewPairingsPt2Pt);
  appCfg.write("", "cbViewPairings_pt2pl", app.viewPairingsPt2Pl);
  appCfg.write("", "cbViewPairings_pt2ln", app.viewPairingsPt2Ln);
  appCfg.write("", "cbViewPairings_cov2cov", app.viewPairingsCov2Cov);
  appCfg.write("", "cbViewPriorEllipsoid", app.viewPriorEllipsoid);

  appCfg.write("", "slPairingsPl2PlSize", app.pairingsPl2PlSize);
  appCfg.write("", "slPairingsPl2LnSize", app.pairingsPl2LnSize);
  appCfg.write("", "slPairingsCov2CovDecimation", app.pairingsCov2CovDecim);
  appCfg.write("", "slGlobalPointSize", app.globalPointSize);
  appCfg.write("", "slLocalPointSize", app.localPointSize);
  appCfg.write("", "slMidDepthField", app.midDepthField);
  appCfg.write("", "slThicknessDepthField", app.thicknessDepthField);

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
  cmd.add_option(
      "-e,--file-extension", argExtension, "Filename extension to look for. Default is `icplog`");
  cmd.add_option(
      "-d,--directory", argSearchDir, "Directory in which to search for *.icplog files.");
  cmd.add_option("-f,--file", argSingleFile, "Load just this one single log *.icplog file.");
  cmd.add_option(
      "-l,--load-plugins", arg_plugins, "One or more (comma separated) *.so files to load as plugins");
  cmd.add_option(
      "--autoplay-period", argAutoPlayPeriod,
      "The period (in seconds) between timestamps to load and show in autoplay mode.");
  argMinQualityOpt = cmd.add_option(
      "-q,--min-quality", argMinQuality,
      "Minimum ICP quality (range [0,1], i.e. 0%-100%) to load a log file. "
      "Files whose ICP result quality is below this threshold are skipped.");

  CLI11_PARSE(cmd, argc, argv);

  try
  {
    if (!arg_plugins.empty())
    {
      std::string errMsg;
      std::cout << "Loading plugin(s): " << arg_plugins << std::endl;
      if (!mrpt::system::loadPluginModules(arg_plugins, errMsg))
      {
        std::cerr << errMsg << std::endl;
        return 1;
      }
    }

    return mainShowGui();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exit due to exception:\n" << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
