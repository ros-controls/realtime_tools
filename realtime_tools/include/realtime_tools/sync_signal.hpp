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

    void signal_read_finished() {
        auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        last_signal_time_.store(now, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cycle_counter_++;
        }
        cv_.notify_all();
    }

    uint64_t wait_for_signal_read_finished(){
        std::unique_lock<std::mutex> lock(mutex_);

        uint64_t cycle_count_current = cycle_counter_;
        cv_.wait(lock,
            [this, cycle_count_current]{
                return cycle_counter_ > cycle_count_current;
            }
        );

        return cycle_counter_;
    }

    int64_t get_last_signal_time() const {
        return last_signal_time_.load(std::memory_order_relaxed);
    }

    int64_t get_cycle_counter() const {
        return cycle_counter_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    uint64_t cycle_counter_ = 0; // prevents spurious wakeups.
    std::atomic<int64_t> last_signal_time_{0};
};


} // namespace hardware_interface

#endif  // REALTIME_TOOLS__SYNC_SIGNAL