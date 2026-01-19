#ifndef REALTIME_TOOLS__SYNC_SIGNAL
#define REALTIME_TOOLS__SYNC_SIGNAL

#include <mutex>
#include <condition_variable>
#include <atomic>

namespace realtime_tools 
{

/*
Used to synchronize threads of hardware interface read calls with update() functions of async controllers.

Slave async hardware interfaces run on robot clock by blocking in read() function.
We want to be able to attach n async controllers to be woken up and their update() functions run after the read() finishes.

This class enables this functionality.
*/
class SyncSignal{
public:

    // HW read -> Controller update signal path
    void signal_read_finished() {
        auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        last_signal_read_finished_time_.store(now, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            completed_updates_ = 0;
            cycle_counter_++;
        }
        cv_hw_to_ctrl_.notify_all();
    }

    uint64_t wait_for_signal_read_finished(){
        std::unique_lock<std::mutex> lock(mutex_);

        uint64_t cycle_count_current = cycle_counter_;
        cv_hw_to_ctrl_.wait(lock,
            [this, cycle_count_current]{
                return cycle_counter_ > cycle_count_current;
            }
        );

        return cycle_counter_;
    }

    // Controller update ->  HW write signal path
    void register_controller() {
        std::lock_guard<std::mutex> lock(mutex_);
        num_updates_to_wait_on_++;
    }

    void unregister_controller() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (num_updates_to_wait_on_ > 0) num_updates_to_wait_on_--;
        // if a controller which hw is waiting on is deactivated, wake the hardware up
        if (completed_updates_ >= num_updates_to_wait_on_) {
            cv_ctrl_to_hw_.notify_one();
        }
    }

    void signal_update_finished() {
        std::lock_guard<std::mutex> lock(mutex_);
        completed_updates_++;
        if (completed_updates_ >= num_updates_to_wait_on_) {
            auto now = std::chrono::steady_clock::now().time_since_epoch().count();
            last_signal_update_finished_time_.store(now, std::memory_order_relaxed);

            cv_ctrl_to_hw_.notify_one();
        }
    }

    bool wait_for_signal_updates_finished(std::chrono::nanoseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_ctrl_to_hw_.wait_for(lock, timeout, [this] { 
            return completed_updates_ >= num_updates_to_wait_on_; 
        });
    }

    int get_num_updates_hw_waits_on() {
        return num_updates_to_wait_on_;
    }

    uint64_t get_last_signal_read_finished_time() const {
        return last_signal_read_finished_time_.load(std::memory_order_relaxed);
    }

    uint64_t get_last_signal_update_finished_time() const {
        return last_signal_update_finished_time_.load(std::memory_order_relaxed);
    }

    uint64_t get_cycle_counter() const {
        return cycle_counter_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_hw_to_ctrl_;
    std::condition_variable cv_ctrl_to_hw_;

    int num_updates_to_wait_on_ = 0;
    int completed_updates_ = 0;

    uint64_t cycle_counter_ = 0; // prevents spurious wakeups.
    std::atomic<int64_t> last_signal_read_finished_time_{0};
    std::atomic<int64_t> last_signal_update_finished_time_{0};
};


} // namespace hardware_interface

#endif  // REALTIME_TOOLS__SYNC_SIGNAL