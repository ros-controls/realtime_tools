// Copyright (c) 2025, Brian Jin
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

// Author: Brian Jin

#include <benchmark/benchmark.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <test_msgs/msg/empty.hpp>

#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/utils/publisher_interface.hpp>
#include <realtime_tools/wait_free_realtime_publisher.hpp>

namespace
{
template <class MessageT>
class BenchmarkPublisher : public realtime_tools::utils::PublisherInterface<MessageT>
{
public:
  void publish(const MessageT &) override { publish_count_++; }

  double count() const { return publish_count_; }

private:
  double publish_count_{0.0};
};
}  // namespace

static void BM_RealtimePublisher(benchmark::State & state)
{
  auto pub = std::make_shared<BenchmarkPublisher<test_msgs::msg::Empty>>();
  realtime_tools::RealtimePublisher<test_msgs::msg::Empty> rt_pub(pub);

  test_msgs::msg::Empty msg;
  for (auto _ : state) {
    rt_pub.try_publish(msg);
  }

  state.counters["num_publishes"] = pub->count();
}
// Register the function as a benchmark
BENCHMARK(BM_RealtimePublisher);

// Define another benchmark
template <std::size_t Capacity>
static void BM_WaitFreeRealtimePublisher(benchmark::State & state)
{
  auto poll_duration_us = state.range(0);
  auto pub = std::make_shared<BenchmarkPublisher<test_msgs::msg::Empty>>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty, Capacity> rt_pub(
    pub, std::chrono::microseconds(poll_duration_us));

  test_msgs::msg::Empty msg;
  for (auto _ : state) {
    rt_pub.push(msg);
  }

  state.counters["num_publishes"] = pub->count();
}

// Custom args generator
static void MicrosecondSweepArgs(benchmark::internal::Benchmark * b)
{
  std::vector<int> microsecond_sweep = {1, 10, 50, 100};
  for (auto micro : microsecond_sweep) {
    b->Arg(micro);
  }
}

BENCHMARK_TEMPLATE(BM_WaitFreeRealtimePublisher, 1)->Apply(MicrosecondSweepArgs);
BENCHMARK_TEMPLATE(BM_WaitFreeRealtimePublisher, 2)->Apply(MicrosecondSweepArgs);
BENCHMARK_TEMPLATE(BM_WaitFreeRealtimePublisher, 5)->Apply(MicrosecondSweepArgs);
BENCHMARK_TEMPLATE(BM_WaitFreeRealtimePublisher, 10)->Apply(MicrosecondSweepArgs);

static void BM_ManualWaitFreeRealtimePublisher(benchmark::State & state)
{
  auto pub = std::make_shared<BenchmarkPublisher<test_msgs::msg::Empty>>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty, 2> rt_pub(
    pub, std::chrono::microseconds(1));

  test_msgs::msg::Empty msg;
  for (auto _ : state) {
    rt_pub.push(msg);
  }

  state.counters["num_publishes"] = pub->count();
}

BENCHMARK(BM_ManualWaitFreeRealtimePublisher);

BENCHMARK_MAIN();
