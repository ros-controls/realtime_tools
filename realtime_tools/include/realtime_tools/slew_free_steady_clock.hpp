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
#include <thread>

namespace realtime_tools
{
/**
 * @brief A steady clock whose rate is never adjusted (slewed) by NTP/PTP/adjtime().
 *
 * std::chrono::steady_clock maps to CLOCK_MONOTONIC on Linux, which never jumps but whose
 * rate is continuously corrected by time daemons (up to +-500 ppm via frequency adjustment and
 * up to +-10% via tick adjustment, e.g. chrony fast slewing). On an RT loop this shows up as period
 * and dt jitter that does not come from the hardware. This clock reads CLOCK_MONOTONIC_RAW,
 * the undisciplined hardware counter, so measured durations reflect the local oscillator only.
 *
 * Satisfies the C++ TrivialClock requirements, so it can be used with std::chrono arithmetic.
 * now() is noexcept, lock-free and allocation-free (vDSO call on modern Linux kernels).
 *
 * @note The epoch is unspecified (boot time on Linux) and differs from CLOCK_MONOTONIC, so do
 * not mix its time points with std::chrono::steady_clock or ROS time.
 * @note Use the static sleep_for()/sleep_until() members to sleep against this clock, not the
 * std::this_thread ones (see sleep_until()).
 * @see doc/slew_free_steady_clock.rst for slew magnitudes and loop-frequency guidance.
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

  /**
   * @brief Block the calling thread until this clock reaches @p abs_time.
   *
   * Never returns before @p abs_time as measured by this clock; returns immediately if it is
   * already in the past. Wake-up latency is that of std::this_thread::sleep_for.
   *
   * @note Do not use std::this_thread::sleep_until with this clock: for steady clocks libstdc++
   * sleeps once on CLOCK_MONOTONIC and never re-checks, so it can return early by the NTP slew.
   */
  template <class Duration>
  static void sleep_until(const std::chrono::time_point<SlewFreeSteadyClock, Duration> & abs_time)
  {
    // clock_nanosleep() rejects CLOCK_MONOTONIC_RAW, so sleep relative on the OS clock and
    // re-check against the raw clock. The kernel bounds slew to +-10% (ADJ_TICK; ADJ_FREQUENCY is
    // limited to 500 ppm), so the residual shrinks by >= 10x per pass. In steady state the first
    // wake-up latency already exceeds the slew error and no extra pass runs.
    for (auto t = now(); t < abs_time; t = now()) {
      std::this_thread::sleep_for(abs_time - t);
    }
  }

  /**
   * @brief Block the calling thread for at least @p rel_time as measured by this clock.
   *
   * Equivalent to sleep_until(now() + rel_time), with rel_time rounded up to nanoseconds.
   * Returns immediately for zero or negative durations.
   */
  template <class Rep, class Period>
  static void sleep_for(const std::chrono::duration<Rep, Period> & rel_time)
  {
    if (rel_time > rel_time.zero()) {
      sleep_until(now() + std::chrono::ceil<duration>(rel_time));
    }
  }
};
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__SLEW_FREE_STEADY_CLOCK_HPP_
