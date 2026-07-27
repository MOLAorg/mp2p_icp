#include <imgui.h>
#include <imgui_app_common/SimpleFileDialog.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace mp2p_icp_viz
{
namespace
{
constexpr const char* kPopupId = "SimpleFileDialog##popup";

bool matchesFilter(const std::filesystem::directory_entry& entry, const FileDialogFilter& filter)
{
    if (filter.extension.empty())
    {
        return true;
    }
    auto ext = entry.path().extension().string();
    if (!ext.empty() && ext.front() == '.')
    {
        ext.erase(0, 1);
    }
    return ext == filter.extension;
}
}  // namespace

void SimpleFileDialog::open(
    Mode mode, const std::vector<FileDialogFilter>& filters, const std::string& title,
    const std::string& initialDir)
{
    mode_              = mode;
    filters_           = filters;
    selectedFilterIdx_ = 0;
    title_             = title.empty() ? (mode == Mode::Open ? "Open file" : "Save file") : title;
    typedName_.clear();
    errorMsg_.clear();

    std::error_code       ec;
    std::filesystem::path startDir = !initialDir.empty()    ? std::filesystem::path(initialDir)
                                     : !currentDir_.empty() ? std::filesystem::path(currentDir_)
                                                            : std::filesystem::current_path(ec);
    if (!std::filesystem::is_directory(startDir, ec))
    {
        startDir = std::filesystem::current_path(ec);
    }
    currentDir_ = startDir.string();

    open_ = true;
    ImGui::OpenPopup(kPopupId);
}

std::optional<std::string> SimpleFileDialog::render()
{
    if (!open_)
    {
        return std::nullopt;
    }

    std::optional<std::string> result;

    ImGui::SetNextWindowSize(ImVec2(560, 420), ImGuiCond_FirstUseEver);
    if (ImGui::BeginPopupModal(kPopupId, nullptr, ImGuiWindowFlags_NoSavedSettings))
    {
        ImGui::TextUnformatted(title_.c_str());
        ImGui::Separator();

        if (ImGui::Button("Up"))
        {
            const std::filesystem::path p(currentDir_);
            if (p.has_parent_path() && p.parent_path() != p)
            {
                currentDir_ = p.parent_path().string();
            }
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(currentDir_.c_str());

        ImGui::Separator();

        if (ImGui::BeginChild(
                "##fileList", ImVec2(0, -80), true, ImGuiWindowFlags_HorizontalScrollbar))
        {
            std::error_code                               ec;
            std::vector<std::filesystem::directory_entry> dirs;
            std::vector<std::filesystem::directory_entry> files;
            for (const auto& entry : std::filesystem::directory_iterator(currentDir_, ec))
            {
                if (entry.is_directory())
                {
                    dirs.push_back(entry);
                }
                else if (
                    filters_.empty() ||
                    static_cast<size_t>(selectedFilterIdx_) >= filters_.size() ||
                    matchesFilter(entry, filters_[static_cast<size_t>(selectedFilterIdx_)]))
                {
                    files.push_back(entry);
                }
            }
            std::sort(
                dirs.begin(), dirs.end(),
                [](const auto& a, const auto& b)
                { return a.path().filename() < b.path().filename(); });
            std::sort(
                files.begin(), files.end(),
                [](const auto& a, const auto& b)
                { return a.path().filename() < b.path().filename(); });

            for (const auto& d : dirs)
            {
                const std::string label = std::string("[") + d.path().filename().string() + "]";
                if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick))
                {
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    {
                        currentDir_ = d.path().string();
                    }
                }
            }
            for (const auto& f : files)
            {
                const std::string name       = f.path().filename().string();
                const bool        isSelected = (name == typedName_);
                if (ImGui::Selectable(
                        name.c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick))
                {
                    typedName_ = name;
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    {
                        result = (std::filesystem::path(currentDir_) / name).string();
                    }
                }
            }
        }
        ImGui::EndChild();

        if (!filters_.empty())
        {
            const std::string& previewLabel =
                filters_[static_cast<size_t>(selectedFilterIdx_)].description;
            if (ImGui::BeginCombo("Filter", previewLabel.c_str()))
            {
                for (size_t i = 0; i < filters_.size(); i++)
                {
                    const bool isSelected = (static_cast<int>(i) == selectedFilterIdx_);
                    if (ImGui::Selectable(filters_[i].description.c_str(), isSelected))
                    {
                        selectedFilterIdx_ = static_cast<int>(i);
                    }
                }
                ImGui::EndCombo();
            }
        }

        char buf[512];
        std::snprintf(buf, sizeof(buf), "%s", typedName_.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##fileName", buf, sizeof(buf)))
        {
            typedName_ = buf;
        }

        if (!errorMsg_.empty())
        {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", errorMsg_.c_str());
        }

        const char* confirmLabel = (mode_ == Mode::Open) ? "Open" : "Save";
        if (ImGui::Button(confirmLabel))
        {
            if (typedName_.empty())
            {
                errorMsg_ = "Please select or type a file name.";
            }
            else
            {
                const auto candidate = std::filesystem::path(currentDir_) / typedName_;
                if (mode_ == Mode::Open && !std::filesystem::exists(candidate))
                {
                    errorMsg_ = "File does not exist.";
                }
                else
                {
                    result = candidate.string();
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            open_ = false;
            ImGui::CloseCurrentPopup();
        }

        if (result.has_value())
        {
            open_ = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
    else
    {
        // Popup was dismissed by other means (e.g. Esc key).
        open_ = false;
    }

    return result;
}

}  // namespace mp2p_icp_viz
