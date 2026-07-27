#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <optional>

namespace mp2p_icp_viz
{
/** Runs a computation on a background thread and lets the caller poll for the result once per
 *  frame without blocking.
 *
 *  Motivation: a GLFW/ImGui main loop must keep calling `glfwPollEvents()` (and swapping
 *  buffers) at a steady cadence, or the window manager's own responsiveness watchdog concludes
 *  the process is hung and offers to kill it -- even though the app would happily keep
 *  processing input if given the chance. Long synchronous operations on the main thread (e.g.
 *  loading a large map file, or building its OpenGL visualization) starve that loop. Running
 *  them here instead keeps the main loop free to keep pumping events while the result is
 *  awaited via poll().
 */
template <typename T>
class AsyncTask
{
   public:
    void start(std::function<T()> fn)
    {
        future_  = std::async(std::launch::async, std::move(fn));
        running_ = true;
    }

    bool isRunning() const { return running_; }

    /** Call once per frame. Returns the result exactly once, on the first poll() after the task
     *  finished; nullopt every other time (including while still running). */
    std::optional<T> poll()
    {
        if (!running_)
        {
            return std::nullopt;
        }
        if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            return std::nullopt;
        }
        running_ = false;
        return future_.get();
    }

   private:
    std::future<T> future_;
    bool           running_ = false;
};

}  // namespace mp2p_icp_viz
