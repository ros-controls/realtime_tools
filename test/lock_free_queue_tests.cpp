// Copyright 2025 PAL Robotics S.L.
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
#include <thread>

#include <realtime_tools/lock_free_queue.hpp>

using realtime_tools::LockFreeMPMCQueue;
using realtime_tools::LockFreeSPSCQueue;

class DefaultConstructable
{
public:
  DefaultConstructable() : number_(42) {}
  int number_;
};

TEST(LockFreeSPSCQueue, default_construct)
{
  {
    LockFreeSPSCQueue<DefaultConstructable> buffer(10);
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
  {
    LockFreeSPSCQueue<DefaultConstructable, 10> buffer;
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
}

TEST(LockFreeSPSCQueue, initialize_value)
{
  LockFreeSPSCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  ASSERT_TRUE(buffer.push(3.14)) << "Buffer should have space for one element";
  ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
  double obj1;
  ASSERT_TRUE(buffer.pop(obj1));
  ASSERT_DOUBLE_EQ(3.14, obj1);
  ASSERT_EQ(0, buffer.size()) << "Buffer should be empty";
  ASSERT_TRUE(buffer.empty());
}

TEST(LockFreeSPSCQueue, test_push)
{
  LockFreeSPSCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(i, buffer.size());
    ASSERT_EQ(10, buffer.capacity());
  }
  ASSERT_FALSE(buffer.push(11)) << "Buffer should not have space for element as size is 10";
  ASSERT_EQ(10, buffer.size());
  ASSERT_FALSE(buffer.empty());
}

TEST(LockFreeSPSCQueue, test_pop)
{
  LockFreeSPSCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(i, buffer.size());
  }
  for (auto i = 1; i <= 10; i++) {
    double obj1;
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(i, obj1);
    ASSERT_EQ(10 - i, buffer.size());
  }
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  double obj1;
  ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
}

TEST(LockFreeSPSCQueue, test_bounded_push)
{
  {
    LockFreeSPSCQueue<double, 10> queue;
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 25; i++) {
      ASSERT_TRUE(queue.bounded_push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(std::min(i, 10), queue.size());
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_EQ(10, queue.size());
    ASSERT_EQ(10, queue.capacity());

    // when we start popping, the pop-ed elements should start from 16
    for (auto i = 1u; i <= queue.capacity(); i++) {
      double obj1;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i + 15, obj1);
      ASSERT_EQ(queue.capacity() - i, queue.size());
    }
    double obj1;
    ASSERT_FALSE(queue.pop(obj1));
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
  }
  {
    LockFreeSPSCQueue<double> queue(10);
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 25; i++) {
      ASSERT_TRUE(queue.bounded_push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(std::min(i, 10), queue.size());
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_EQ(10, queue.size());
    ASSERT_EQ(10, queue.capacity());

    // when we start popping, the pop-ed elements should start from 16
    for (auto i = 1u; i <= queue.capacity(); i++) {
      double obj1;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i + 15, obj1);
      ASSERT_EQ(queue.capacity() - i, queue.size());
    }
    double obj1;
    ASSERT_FALSE(queue.pop(obj1));
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
  }
}

TEST(LockFreeSPSCQueue, test_lockfree_queue_push)
{
  LockFreeSPSCQueue<int> queue(100);
  int producer_count = 0;
  std::atomic_int consumer_count(0);
  std::atomic_bool done(false);

  const int iterations = 1000000;

  std::thread producer([&]() {
    for (int i = 0; i < iterations; ++i) {
      while (!queue.push(i)) {
        std::this_thread::yield();
      }
      ++producer_count;
    }
  });

  std::thread consumer([&]() {
    int value;
    while (!done) {
      while (queue.pop(value)) {
        ++consumer_count;
      }
    }
    while (queue.pop(value)) {
      ++consumer_count;
    }
  });

  producer.join();
  done = true;
  consumer.join();

  ASSERT_EQ(producer_count, consumer_count);
  ASSERT_EQ(producer_count, iterations);
  ASSERT_EQ(consumer_count, iterations);
}

// TEST(LockFreeSPSCQueue, test_lockfree_queue_bounded_push)
// {
//   LockFreeSPSCQueue<int, 100> queue;
//   std::atomic_int producer_count = 0;
//   std::atomic_int consumer_count(0);
//   std::atomic_bool done(false);

//   const int iterations = 1000000;

//   std::thread producer([&]() {
//     for (auto j = 0; j < iterations; ++j) {
//       ASSERT_TRUE(queue.bounded_push(j));
//       ASSERT_EQ(100, queue.capacity());
//       ++producer_count;
//     }
//   });

//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   std::cerr << "producer_count: " << producer_count << std::endl;

//   std::thread consumer([&]() {
//     int value;
//     while (!done) {
//       while (queue.pop(value)) {
//         ++consumer_count;
//         std::this_thread::yield();
//       }
//     }
//     while (queue.pop(value)) {
//       ++consumer_count;
//     }
//   });

//   producer.join();
//   done = true;
//   consumer.join();

//   std::cerr << "producer_count: " << producer_count << std::endl;
//   std::cerr << "consumer_count: " << consumer_count << std::endl;
//   std::cerr << "iterations: " << iterations << std::endl;
//   ASSERT_GT(producer_count, consumer_count);
//   ASSERT_EQ(producer_count, iterations);
//   ASSERT_GT(iterations, consumer_count);
// }

TEST(LockFreeMPMCQueue, default_construct)
{
  {
    LockFreeMPMCQueue<DefaultConstructable> buffer(10);
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
  {
    LockFreeMPMCQueue<DefaultConstructable, 10> buffer;
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
}

TEST(LockFreeMPMCQueue, initialize_value)
{
  LockFreeMPMCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  ASSERT_TRUE(buffer.push(3.14)) << "Buffer should have space for one element";
  double obj1;
  ASSERT_TRUE(buffer.pop(obj1));
  ASSERT_DOUBLE_EQ(3.14, obj1);
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
}

TEST(LockFreeMPMCQueue, test_push)
{
  LockFreeMPMCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(10, buffer.capacity());
  }
  ASSERT_FALSE(buffer.push(11)) << "Buffer should not have space for element as size is 10";
  ASSERT_FALSE(buffer.empty());
}

TEST(LockFreeMPMCQueue, test_pop)
{
  LockFreeMPMCQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
  }
  for (auto i = 1; i <= 10; i++) {
    double obj1;
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(i, obj1);
  }
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  double obj1;
  ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
}

TEST(LockFreeMPMCQueue, test_bounded_push)
{
  {
    LockFreeMPMCQueue<double, 10> queue;
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 25; i++) {
      ASSERT_TRUE(queue.bounded_push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_EQ(10, queue.capacity());

    // when we start popping, the pop-ed elements should start from 16
    for (auto i = 1u; i <= queue.capacity(); i++) {
      double obj1;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i + 15, obj1);
    }
    double obj1;
    ASSERT_FALSE(queue.pop(obj1));
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
  }
  {
    LockFreeMPMCQueue<double> queue(10);
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 25; i++) {
      ASSERT_TRUE(queue.bounded_push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_EQ(10, queue.capacity());

    // when we start popping, the pop-ed elements should start from 16
    for (auto i = 1u; i <= queue.capacity(); i++) {
      double obj1;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i + 15, obj1);
    }
    double obj1;
    ASSERT_FALSE(queue.pop(obj1));
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
  }
}

TEST(LockFreeMPMCQueue, test_lockfree_queue_push)
{
  LockFreeMPMCQueue<int, 100> queue;
  int producer_count = 0;
  std::atomic_int consumer_count(0);
  std::atomic_bool done(false);

  const int iterations = 1000000;

  std::thread producer([&]() {
    for (int i = 0; i < iterations; ++i) {
      while (!queue.push(i)) {
        std::this_thread::yield();
      }
      ++producer_count;
    }
  });

  std::thread consumer([&]() {
    int value;
    while (!done) {
      while (queue.pop(value)) {
        ++consumer_count;
      }
    }
    while (queue.pop(value)) {
      ++consumer_count;
    }
  });

  producer.join();
  done = true;
  consumer.join();

  ASSERT_EQ(producer_count, consumer_count);
  ASSERT_EQ(producer_count, iterations);
  ASSERT_EQ(consumer_count, iterations);
}

TEST(LockFreeMPMCQueue, test_lockfree_queue_bounded_push)
{
  LockFreeMPMCQueue<int, 100> queue;
  std::atomic_int producer_count = 0;
  std::atomic_int consumer_count(0);
  std::atomic_bool done(false);

  const int iterations = 1000000;

  std::thread producer([&]() {
    for (auto j = 0; j < iterations; ++j) {
      ASSERT_TRUE(queue.bounded_push(j));
      ASSERT_EQ(100, queue.capacity());
      ++producer_count;
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cerr << "producer_count: " << producer_count << std::endl;

  std::thread consumer([&]() {
    int value;
    while (!done) {
      while (queue.pop(value)) {
        ++consumer_count;
        std::this_thread::yield();
      }
    }
    while (queue.pop(value)) {
      ++consumer_count;
    }
  });

  producer.join();
  done = true;
  consumer.join();

  std::cerr << "producer_count: " << producer_count << std::endl;
  std::cerr << "consumer_count: " << consumer_count << std::endl;
  std::cerr << "iterations: " << iterations << std::endl;
  ASSERT_GT(producer_count, consumer_count);
  ASSERT_EQ(producer_count, iterations);
  ASSERT_GT(iterations, consumer_count);
}
