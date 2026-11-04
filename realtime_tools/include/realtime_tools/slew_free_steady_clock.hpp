// Copyright 2026 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#ifndef REALTIME_TOOLS__SLEW_FREE_STEADY_CLOCK_HPP_
#define REALTIME_TOOLS__SLEW_FREE_STEADY_CLOCK_HPP_

#include <chrono>
#include <ctime>

namespace realtime_tools
{
/**
 * @brief A steady clock whose rate is never adjusted (slewed) by NTP/PTP/adjtime().
 *
 * std::chrono::steady_clock maps to CLOCK_MONOTONIC on Linux, which never jumps but whose
 * frequency is continuously corrected by time daemons. On an RT loop this shows up as period
 * and dt jitter that does not come from the hardware. This clock reads CLOCK_MONOTONIC_RAW,
 * the undisciplined hardware counter, so measured durations reflect the local oscillator only.
 *
 * Satisfies the C++ TrivialClock requirements, so it can be used with std::chrono arithmetic.
 * now() is noexcept, lock-free and allocation-free (vDSO call on modern Linux kernels).
 *
 * @note The epoch is unspecified (boot time on Linux) and differs from CLOCK_MONOTONIC, so do
 * not mix its time points with std::chrono::steady_clock or ROS time.
 * @note clock_nanosleep() does not accept CLOCK_MONOTONIC_RAW; use this clock to measure time,
 * not to schedule absolute wake-ups.
 * @note On platforms without CLOCK_MONOTONIC_RAW (e.g. Windows, whose QueryPerformanceCounter
 * is not slewed), this falls back to std::chrono::steady_clock.
 */
class SlewFreeSteadyClock
{
public:
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<SlewFreeSteadyClock>;

  static constexpr bool is_steady = true;

  /// @return The current time of the raw monotonic hardware clock.
  static time_point now() noexcept
  {
#ifdef CLOCK_MONOTONIC_RAW
    timespec ts{};
    // Cannot fail: the clock id is valid and ts is a valid pointer.
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return time_point(std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec));
#else
    return time_point(
      std::chrono::duration_cast<duration>(std::chrono::steady_clock::now().time_since_epoch()));
#endif
  }
};
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__SLEW_FREE_STEADY_CLOCK_HPP_
