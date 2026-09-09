#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <imgui_app_common/ImGuiAppShell.h>
#include <imgui_internal.h>  // DockBuilder* API (default layout on first run)

#include <cstdio>

#include "../../libcfgpath/cfgpath.h"

namespace mp2p_icp_viz
{
namespace
{
void glfwErrorCallback(int error, const char* description)
{
    std::fprintf(stderr, "[ImGuiAppShell] GLFW error %d: %s\n", error, description);
}
}  // namespace

ImGuiAppShell::~ImGuiAppShell() { shutdown(); }

bool ImGuiAppShell::init(const std::string& windowTitle, int width, int height)
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
    {
        std::fprintf(stderr, "[ImGuiAppShell] glfwInit() failed\n");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(width, height, windowTitle.c_str(), nullptr, nullptr);
    if (!window_)
    {
        std::fprintf(stderr, "[ImGuiAppShell] glfwCreateWindow() failed\n");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Persist the docking layout in a per-app imgui.ini file, under the user's config
    // directory (e.g. ~/.config/<windowTitle>/), not the current working directory.
    char configDir[MAX_PATH];
    get_user_config_folder(configDir, sizeof(configDir), windowTitle.c_str());
    iniPath_       = configDir[0] != '\0' ? std::string(configDir) + windowTitle + ".imgui.ini"
                                          : windowTitle + ".imgui.ini";
    io.IniFilename = iniPath_.c_str();

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    imguiInitialized_ = true;

    return true;
}

void ImGuiAppShell::run(const std::function<void()>& renderFrame)
{
    while (window_ && !glfwWindowShouldClose(window_))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);

        const ImGuiWindowFlags hostFlags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoBackground;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("##DockSpaceRoot", nullptr, hostFlags);
        ImGui::PopStyleVar(3);

        const ImGuiID dockspaceId = ImGui::GetID("MainDockSpace");

        if (setupDefaultLayout && !ImGui::DockBuilderGetNode(dockspaceId))
        {
            ImGui::DockBuilderRemoveNode(dockspaceId);
            ImGui::DockBuilderAddNode(
                dockspaceId, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->Size);

            setupDefaultLayout(dockspaceId);

            ImGui::DockBuilderFinish(dockspaceId);
        }

        ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::End();

        if (renderFrame)
        {
            renderFrame();
        }

        ImGui::Render();

        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(window_, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }
}

void ImGuiAppShell::requestClose()
{
    if (window_)
    {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void ImGuiAppShell::shutdown()
{
    if (imguiInitialized_)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        imguiInitialized_ = false;
    }
    if (window_)
    {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
    }
}

}  // namespace mp2p_icp_viz
