// Copyright 2026 b-robotized GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \author Nikola Banovic

#ifndef REALTIME_TOOLS__SYNC_SIGNAL_HPP_
#define REALTIME_TOOLS__SYNC_SIGNAL_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>

namespace realtime_tools
{

/*
* Used to synchronize threads of hardware interface read calls with update() functions of async controllers.
*
* Slave async hardware interfaces run on robot clock by blocking in read() function.
* We want to be able to attach n async controllers to be woken up and their update() functions run after the read() finishes.
*
* This class enables this functionality.
*
* Example flow:
*   HW read() -> signal_read_finished() -> controllers wake
*   -> controller update() -> signal_update_finished()
*   -> (all controllers done) -> HW write() proceeds
*
*/
class SyncSignal
{
public:
  /// HW read -> Controller update signal path
  void signal_read_finished()
  {
    auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    last_signal_read_finished_time_.store(now, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      completed_updates_ = 0;
      cycle_counter_++;
    }
    cv_hw_to_ctrl_.notify_all();
  }

  /// called by the controller to wait for the next read_finished signal.
  /// @return The cycle counter if the signal was received, std::nullopt if timed out.
  std::optional<uint64_t> wait_for_signal_read_finished(std::chrono::nanoseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    uint64_t cycle_count_current = cycle_counter_;
    bool signaled = cv_hw_to_ctrl_.wait_for(
      lock, timeout, [this, cycle_count_current] { return cycle_counter_ > cycle_count_current; });

    if (!signaled) {
      return std::nullopt;
    }
    return cycle_counter_;
  }

  /// Controller update ->  HW write signal path

  /// register that a controller as requires synchronization. Called during activation.
  void register_controller()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    num_updates_to_wait_on_++;
  }

  /// Unregister a controller. Called on controller deactivation/error/shutdown.
  void unregister_controller()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (num_updates_to_wait_on_ > 0) {
      num_updates_to_wait_on_--;
    }
    // If hardware is waiting on updates and this was the last one, wake it up.
    if (completed_updates_ >= num_updates_to_wait_on_) {
      cv_ctrl_to_hw_.notify_one();
    }
  }

  /// called by controller after update() finishes.
  void signal_update_finished()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    completed_updates_++;
    if (completed_updates_ >= num_updates_to_wait_on_) {
      auto now = std::chrono::steady_clock::now().time_since_epoch().count();
      last_signal_update_finished_time_.store(now, std::memory_order_relaxed);
      cv_ctrl_to_hw_.notify_one();
    }
  }

  /// called by hardware to wait for all registered controllers to finish update().
  /// @return true if all updates completed, std::nullopt if timed out.
  std::optional<bool> wait_for_signal_updates_finished(std::chrono::nanoseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    bool completed = cv_ctrl_to_hw_.wait_for(
      lock, timeout, [this] { return completed_updates_ >= num_updates_to_wait_on_; });

    if (!completed) {
      return std::nullopt;
    }
    return true;
  }

  int get_num_updates_hw_waits_on() const { return num_updates_to_wait_on_; }

  uint64_t get_last_signal_read_finished_time() const
  {
    return static_cast<uint64_t>(last_signal_read_finished_time_.load(std::memory_order_relaxed));
  }

  uint64_t get_last_signal_update_finished_time() const
  {
    return static_cast<uint64_t>(last_signal_update_finished_time_.load(std::memory_order_relaxed));
  }

  uint64_t get_cycle_counter() const { return cycle_counter_; }

private:
  std::mutex mutex_;
  std::condition_variable cv_hw_to_ctrl_;
  std::condition_variable cv_ctrl_to_hw_;

  int num_updates_to_wait_on_ = 0;
  int completed_updates_ = 0;

  uint64_t cycle_counter_ = 0;  // prevents spurious wakeups.
  std::atomic<int64_t> last_signal_read_finished_time_{0};
  std::atomic<int64_t> last_signal_update_finished_time_{0};
};

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__SYNC_SIGNAL_HPP_
