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

#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <thread>
#include <type_traits>

#include "realtime_tools/slew_free_steady_clock.hpp"

using realtime_tools::SlewFreeSteadyClock;

static_assert(SlewFreeSteadyClock::is_steady, "clock must be steady");
static_assert(
  std::is_same_v<SlewFreeSteadyClock::duration, std::chrono::nanoseconds>,
  "clock must have nanosecond resolution");
static_assert(noexcept(SlewFreeSteadyClock::now()), "now() must be noexcept for RT use");

TEST(SlewFreeSteadyClockTest, NowIsMonotonic)
{
  auto prev = SlewFreeSteadyClock::now();
  for (int i = 0; i < 100000; ++i) {
    const auto cur = SlewFreeSteadyClock::now();
    ASSERT_GE(cur, prev);
    prev = cur;
  }
}

TEST(SlewFreeSteadyClockTest, MeasuresElapsedTime)
{
  const auto start = SlewFreeSteadyClock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto elapsed = SlewFreeSteadyClock::now() - start;
  EXPECT_GE(elapsed, std::chrono::milliseconds(10));
  EXPECT_LT(elapsed, std::chrono::seconds(1));
}

#ifdef CLOCK_MONOTONIC_RAW
// The whole point of the clock: it must read the raw (non NTP-slewed) hardware counter,
// so its value must be bracketed by two direct CLOCK_MONOTONIC_RAW reads.
TEST(SlewFreeSteadyClockTest, ReadsMonotonicRawSource)
{
  auto read_raw = []() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec);
  };
  const auto before = read_raw();
  const auto now = SlewFreeSteadyClock::now().time_since_epoch();
  const auto after = read_raw();
  EXPECT_LE(before, now);
  EXPECT_LE(now, after);
}
#endif

TEST(SlewFreeSteadyClockTest, SleepForNeverReturnsEarly)
{
  const auto start = SlewFreeSteadyClock::now();
  SlewFreeSteadyClock::sleep_for(std::chrono::milliseconds(10));
  const auto elapsed = SlewFreeSteadyClock::now() - start;
  EXPECT_GE(elapsed, std::chrono::milliseconds(10));
  EXPECT_LT(elapsed, std::chrono::seconds(1));
}

TEST(SlewFreeSteadyClockTest, SleepUntilNeverReturnsEarly)
{
  const auto target = SlewFreeSteadyClock::now() + std::chrono::milliseconds(10);
  SlewFreeSteadyClock::sleep_until(target);
  const auto now = SlewFreeSteadyClock::now();
  EXPECT_GE(now, target);
  EXPECT_LT(now - target, std::chrono::seconds(1));
}

TEST(SlewFreeSteadyClockTest, SleepReturnsImmediatelyForPastOrNonPositive)
{
  const auto start = SlewFreeSteadyClock::now();
  SlewFreeSteadyClock::sleep_until(start - std::chrono::seconds(1));
  SlewFreeSteadyClock::sleep_for(std::chrono::nanoseconds(0));
  SlewFreeSteadyClock::sleep_for(std::chrono::milliseconds(-10));
  EXPECT_LT(SlewFreeSteadyClock::now() - start, std::chrono::milliseconds(5));
}

TEST(SlewFreeSteadyClockTest, SleepAcceptsAnyDurationType)
{
  const auto start = SlewFreeSteadyClock::now();
  SlewFreeSteadyClock::sleep_for(std::chrono::duration<double, std::milli>(2.5));
  EXPECT_GE(SlewFreeSteadyClock::now() - start, std::chrono::microseconds(2500));
}
