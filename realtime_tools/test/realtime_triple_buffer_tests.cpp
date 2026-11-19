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

#include <gmock/gmock.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "realtime_tools/realtime_triple_buffer.hpp"

using realtime_tools::RealtimeTripleBuffer;

TEST(RealtimeTripleBuffer, default_constructed_has_no_new_data)
{
  RealtimeTripleBuffer<int> buffer;
  EXPECT_FALSE(buffer.has_new_data());
  EXPECT_EQ(buffer.read(), 0);
}

TEST(RealtimeTripleBuffer, initial_value_is_read_back)
{
  RealtimeTripleBuffer<std::string> buffer(std::string("init"));
  EXPECT_FALSE(buffer.has_new_data());
  EXPECT_EQ(buffer.read(), "init");
}

TEST(RealtimeTripleBuffer, initial_value_preallocates_every_slot)
{
  // Every slot must be a copy of the initial value, so a reader/writer never sees a smaller
  // container than the one passed in (no reallocation needed when writing in place).
  RealtimeTripleBuffer<std::vector<double>> buffer(std::vector<double>(10, 1.0));
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(buffer.write_buffer().size(), 10u);
    buffer.write_buffer()[0] = static_cast<double>(i);
    buffer.publish();
    EXPECT_EQ(buffer.read().size(), 10u);
    EXPECT_DOUBLE_EQ(buffer.read()[0], static_cast<double>(i));
  }
}

TEST(RealtimeTripleBuffer, write_then_read)
{
  RealtimeTripleBuffer<int> buffer;
  buffer.write(42);
  EXPECT_TRUE(buffer.has_new_data());
  EXPECT_EQ(buffer.read(), 42);
  EXPECT_FALSE(buffer.has_new_data());
  // Reading again without a new write keeps returning the latest value
  EXPECT_EQ(buffer.read(), 42);
}

TEST(RealtimeTripleBuffer, only_latest_value_is_kept)
{
  RealtimeTripleBuffer<int> buffer;
  for (int i = 1; i <= 10; ++i) {
    buffer.write(i);
  }
  EXPECT_TRUE(buffer.has_new_data());
  EXPECT_EQ(buffer.read(), 10);
  EXPECT_FALSE(buffer.has_new_data());
}

TEST(RealtimeTripleBuffer, write_in_place_and_publish)
{
  RealtimeTripleBuffer<std::vector<int>> buffer(std::vector<int>(3, 0));
  auto & slot = buffer.write_buffer();
  slot = {1, 2, 3};
  EXPECT_FALSE(buffer.has_new_data()) << "Nothing is visible to the reader before publish()";
  buffer.publish();
  EXPECT_TRUE(buffer.has_new_data());
  EXPECT_THAT(buffer.read(), ::testing::ElementsAre(1, 2, 3));
}

TEST(RealtimeTripleBuffer, move_only_type)
{
  RealtimeTripleBuffer<std::unique_ptr<int>> buffer;
  buffer.write(std::make_unique<int>(7));
  ASSERT_NE(buffer.read(), nullptr);
  EXPECT_EQ(*buffer.read(), 7);
}

TEST(RealtimeTripleBuffer, read_slot_is_not_overwritten_by_writer)
{
  // The slot handed out by read() must stay untouched by the writer until the next read()
  RealtimeTripleBuffer<int> buffer;
  buffer.write(1);
  const int & latest = buffer.read();
  buffer.write(2);
  buffer.write(3);
  buffer.write(4);
  EXPECT_EQ(latest, 1);
  EXPECT_EQ(buffer.read(), 4);
}

namespace
{
struct Sample
{
  uint64_t a{0};
  uint64_t b{0};
  uint64_t c{0};
};
}  // namespace

TEST(RealtimeTripleBuffer, concurrent_reader_never_sees_torn_or_old_data)
{
  // The writer publishes {n, 2n, 3n} with increasing n. A torn read would break the field relation,
  // and a reordering bug would make n go backwards.
  constexpr uint64_t kIterations = 200000;
  RealtimeTripleBuffer<Sample> buffer;
  std::atomic<bool> writer_done{false};

  std::thread writer([&]() {
    for (uint64_t n = 1; n <= kIterations; ++n) {
      auto & slot = buffer.write_buffer();
      slot.a = n;
      slot.b = 2 * n;
      slot.c = 3 * n;
      buffer.publish();
    }
    writer_done = true;
  });

  uint64_t last_seen = 0;
  uint64_t torn = 0;
  uint64_t backwards = 0;
  while (true) {
    // Load the flag before reading so the final read is guaranteed to see the last publish()
    const bool done = writer_done.load();
    const Sample & s = buffer.read();
    if (s.b != 2 * s.a || s.c != 3 * s.a) {
      ++torn;
    }
    if (s.a < last_seen) {
      ++backwards;
    }
    last_seen = s.a;
    if (done) {
      break;
    }
  }
  writer.join();

  EXPECT_EQ(torn, 0u);
  EXPECT_EQ(backwards, 0u);
  EXPECT_EQ(last_seen, kIterations);
}
