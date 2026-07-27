#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mp2p_icp_viz
{
/** One selectable file-type filter, e.g. {"mm", "Metric maps (*.mm)"}.
 *  An empty `extension` means "any file". */
struct FileDialogFilter
{
    std::string extension;
    std::string description;
};

/** Minimal, self-contained ImGui file browser (no native/system dialog, no
 *  extra third-party dependency beyond Dear ImGui itself), used for both
 *  "open" and "save" prompts by mm-viewer and icp-log-viewer.
 *
 *  Usage:
 *  \code
 *    dialog.open(SimpleFileDialog::Mode::Open, {{"mm", "Metric maps (*.mm)"}});
 *    // every frame:
 *    if (auto path = dialog.render(); path.has_value()) { ... use *path ... }
 *  \endcode
 */
class SimpleFileDialog
{
   public:
    enum class Mode
    {
        Open,
        Save
    };

    /** Opens the dialog, starting at `initialDir` (or the current directory,
     *  or the last directory this dialog was used in, if empty). */
    void open(
        Mode mode, const std::vector<FileDialogFilter>& filters, const std::string& title = "",
        const std::string& initialDir = "");

    /** Must be called every frame while isOpen(). Returns the selected path
     *  once the user confirms (Open/Save), or nullopt otherwise (still open,
     *  or cancelled -- check isOpen() to tell those apart). */
    std::optional<std::string> render();

    bool isOpen() const { return open_; }

   private:
    bool                          open_ = false;
    Mode                          mode_ = Mode::Open;
    std::string                   title_;
    std::vector<FileDialogFilter> filters_;
    int                           selectedFilterIdx_ = 0;

    std::string currentDir_;
    std::string typedName_;
    std::string errorMsg_;

    // Unique per instance (derived from `this`), so two SimpleFileDialog objects never share the
    // same ImGui popup ID -- otherwise ImGui would treat them as the very same modal popup.
    std::string popupId_;
};

}  // namespace mp2p_icp_viz
