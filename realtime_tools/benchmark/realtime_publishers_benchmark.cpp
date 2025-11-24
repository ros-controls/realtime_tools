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

#include <memory>
#include <string>

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
static void BM_WaitFreeRealtimePublisher(benchmark::State & state)
{
  auto pub = std::make_shared<BenchmarkPublisher<test_msgs::msg::Empty>>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(pub);

  test_msgs::msg::Empty msg;
  for (auto _ : state) {
    rt_pub.push(msg);
  }

  state.counters["num_publishes"] = pub->count();
}
BENCHMARK(BM_WaitFreeRealtimePublisher);

BENCHMARK_MAIN();
