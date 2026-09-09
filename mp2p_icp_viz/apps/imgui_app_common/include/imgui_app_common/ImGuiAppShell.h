#pragma once

#include <functional>
#include <string>

struct GLFWwindow;

namespace mp2p_icp_viz
{
/** Minimal GLFW + Dear ImGui (docking) application shell shared by
 *  mm-viewer and icp-log-viewer.
 *
 *  Owns the window, the GL context, and the ImGui/backends lifecycle.
 *  Callers only provide a per-frame render callback; a full-window
 *  passthrough dockspace is already set up before it is invoked, so callers
 *  just open ImGui windows and let the user dock them freely.
 */
class ImGuiAppShell
{
   public:
    ImGuiAppShell() = default;
    ~ImGuiAppShell();

    ImGuiAppShell(const ImGuiAppShell&)            = delete;
    ImGuiAppShell& operator=(const ImGuiAppShell&) = delete;

    /** Creates the GLFW window, GL context, and ImGui/backends state.
     *  Returns false on failure (details logged to stderr). */
    bool init(const std::string& windowTitle, int width = 1280, int height = 800);

    /** Runs the main loop until the window is closed or requestClose() is
     *  called. `renderFrame` is invoked once per frame between
     *  `ImGui::NewFrame()` and `ImGui::Render()`. */
    void run(const std::function<void()>& renderFrame);

    /** Releases ImGui/backends state and destroys the GLFW window.
     *  Safe to call more than once. */
    void shutdown();

    /** Requests that run() stop its loop after the current frame. */
    void requestClose();

    GLFWwindow* windowHandle() const { return window_; }

    /** Optional hook, called once with the dockspace's ImGuiID, only the very first time the
     *  app runs with no saved docking layout (no `imgui.ini` entry yet) -- lets each app define
     *  its own default panel arrangement (which is app-specific, so it does not belong in this
     *  shared shell) via `ImGui::DockBuilderSplitNode()`/`DockBuilderDockWindow()`. Once the
     *  user rearranges panels, their layout is persisted and this hook is not called again. */
    std::function<void(unsigned int dockspaceId)> setupDefaultLayout;

   private:
    GLFWwindow* window_ = nullptr;
    std::string iniPath_;
    bool        imguiInitialized_ = false;
};

}  // namespace mp2p_icp_viz
