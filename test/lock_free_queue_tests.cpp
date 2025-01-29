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
  const auto default_construct_test_lambda = [](auto & queue) {
    DefaultConstructable obj1;
    ASSERT_EQ(10, queue.capacity());
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_FALSE(queue.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(queue.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, queue.capacity());
    ASSERT_EQ(1, queue.size()) << "Buffer should have one element";
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_TRUE(queue.is_lock_free());
    ASSERT_EQ(10, queue.capacity());
    ASSERT_EQ(42, obj1.number_);
  };

  LockFreeSPSCQueue<DefaultConstructable> queue_1(10);
  LockFreeSPSCQueue<DefaultConstructable, 10> queue_2;
  default_construct_test_lambda(queue_1);
  default_construct_test_lambda(queue_2);
}

TEST(LockFreeSPSCQueue, initialize_value)
{
  const auto initialize_value_test = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_TRUE(queue.push(3.14)) << "Buffer should have space for one element";
    ASSERT_EQ(1, queue.size()) << "Buffer should have one element";
    ASSERT_TRUE(queue.is_lock_free());
    double obj1;
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_DOUBLE_EQ(3.14, obj1);
    ASSERT_EQ(0, queue.size()) << "Buffer should be empty";
    ASSERT_TRUE(queue.empty());
    ASSERT_TRUE(queue.push(2.71)) << "Buffer should have space for one element";
    ASSERT_EQ(1, queue.size()) << "Buffer should have one element";
    int obj2;
    ASSERT_TRUE(queue.pop(obj2));
    ASSERT_EQ(2, obj2);
    ASSERT_EQ(0, queue.size()) << "Buffer should be empty";
    ASSERT_TRUE(queue.empty());
    ASSERT_TRUE(queue.push(6));
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_EQ(6, obj1);
  };

  LockFreeSPSCQueue<double, 10> queue_1;
  LockFreeSPSCQueue<double> queue_2(10);
  initialize_value_test(queue_1);
  initialize_value_test(queue_2);
}

TEST(LockFreeSPSCQueue, test_push)
{
  const auto push_test_lambda = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(i, queue.size());
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_FALSE(queue.push(11)) << "Buffer should not have space for element as size is 10";
    ASSERT_EQ(10, queue.size());
    ASSERT_FALSE(queue.empty());
  };

  LockFreeSPSCQueue<double, 10> queue_1;
  LockFreeSPSCQueue<double> queue_2(10);
  push_test_lambda(queue_1);
  push_test_lambda(queue_2);
}

TEST(LockFreeSPSCQueue, test_pop)
{
  const auto pop_test_lambda = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(i, queue.size());
    }
    for (auto i = 1; i <= 10; i++) {
      double obj1 = -10;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i, obj1);
      ASSERT_EQ(10 - i, queue.size());
    }
    double obj1;
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_FALSE(queue.pop(obj1)) << "Buffer should be empty";
  };

  LockFreeSPSCQueue<double, 10> queue_1;
  LockFreeSPSCQueue<double> queue_2(10);
  pop_test_lambda(queue_1);
  pop_test_lambda(queue_2);
}

TEST(LockFreeSPSCQueue, test_get_latest)
{
  const auto get_latest_test = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(i, queue.size());
    }
    double obj1 = -10;
    ASSERT_TRUE(queue.get_latest(obj1));
    ASSERT_EQ(10, obj1);
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_EQ(0, queue.size());
    ASSERT_FALSE(queue.get_latest(obj1));
  };

  LockFreeSPSCQueue<double, 10> queue_1;
  LockFreeSPSCQueue<double> queue_2(10);
  get_latest_test(queue_1);
  get_latest_test(queue_2);
}

TEST(LockFreeSPSCQueue, test_bounded_push)
{
  const auto bounded_push_test = [](auto & queue) {
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
  };

  LockFreeSPSCQueue<double, 10> queue_1;
  LockFreeSPSCQueue<double> queue_2(10);
  bounded_push_test(queue_1);
  bounded_push_test(queue_2);
}

TEST(LockFreeSPSCQueue, test_lockfree_queue_push)
{
  const auto lockfree_queue_push_test = [](auto & queue) {
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
  };

  LockFreeSPSCQueue<int> queue_1(100);
  LockFreeSPSCQueue<int, 100> queue_2;
  lockfree_queue_push_test(queue_1);
  lockfree_queue_push_test(queue_2);
}

TEST(LockFreeMPMCQueue, default_construct)
{
  const auto default_construct_test = [](auto & queue) {
    DefaultConstructable obj1;
    ASSERT_EQ(10, queue.capacity());
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_FALSE(queue.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(queue.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, queue.capacity());
    ASSERT_TRUE(queue.is_lock_free());
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_EQ(10, queue.capacity());
    ASSERT_EQ(42, obj1.number_);
  };

  LockFreeMPMCQueue<DefaultConstructable, 10> queue_1;
  LockFreeMPMCQueue<DefaultConstructable> queue_2(10);
  default_construct_test(queue_1);
  default_construct_test(queue_2);
}

TEST(LockFreeMPMCQueue, initialize_value)
{
  const auto initialize_value_test = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_TRUE(queue.push(3.14)) << "Buffer should have space for one element";
    ASSERT_TRUE(queue.is_lock_free());
    double obj1;
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_DOUBLE_EQ(3.14, obj1);
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_TRUE(queue.push(2.71)) << "Buffer should have space for one element";
    int obj2;
    ASSERT_TRUE(queue.pop(obj2));
    ASSERT_EQ(2, obj2);
    ASSERT_TRUE(queue.empty());
    ASSERT_TRUE(queue.push(6));
    ASSERT_TRUE(queue.pop(obj1));
    ASSERT_EQ(6, obj1);
  };

  LockFreeMPMCQueue<double, 10> queue_1;
  LockFreeMPMCQueue<double> queue_2(10);
  initialize_value_test(queue_1);
  initialize_value_test(queue_2);
}

TEST(LockFreeMPMCQueue, test_push)
{
  const auto push_test_lambda = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_FALSE(queue.push(11)) << "Buffer should not have space for element as size is 10";
    ASSERT_FALSE(queue.empty());
  };

  LockFreeMPMCQueue<double, 10> queue_1;
  LockFreeMPMCQueue<double> queue_2(10);
  push_test_lambda(queue_1);
  push_test_lambda(queue_2);
}

TEST(LockFreeMPMCQueue, test_pop)
{
  const auto pop_test_lambda = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
    }
    for (auto i = 1; i <= 10; i++) {
      double obj1;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i, obj1);
    }
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    double obj1;
    ASSERT_FALSE(queue.pop(obj1)) << "Buffer should be empty";
  };

  LockFreeMPMCQueue<double, 10> queue_1;
  LockFreeMPMCQueue<double> queue_2(10);
  pop_test_lambda(queue_1);
  pop_test_lambda(queue_2);
}

TEST(LockFreeMPMCQueue, test_get_latest)
{
  const auto get_latest_test_lambda = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 10; i++) {
      ASSERT_TRUE(queue.push(i)) << "Buffer should have space for element as size is 10";
    }
    ASSERT_FALSE(queue.empty()) << "Buffer should not be empty";
    double obj1;
    ASSERT_TRUE(queue.get_latest(obj1));
    ASSERT_EQ(10, obj1);
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    ASSERT_FALSE(queue.get_latest(obj1));
  };

  LockFreeMPMCQueue<double, 10> queue_1;
  LockFreeMPMCQueue<double> queue_2(10);
  get_latest_test_lambda(queue_1);
  get_latest_test_lambda(queue_2);
}

TEST(LockFreeMPMCQueue, test_bounded_push)
{
  const auto bounded_push_test = [](auto & queue) {
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
    for (auto i = 1; i <= 25; i++) {
      ASSERT_TRUE(queue.bounded_push(i)) << "Buffer should have space for element as size is 10";
      ASSERT_EQ(10, queue.capacity());
    }
    ASSERT_EQ(10, queue.capacity());

    // when we start popping, the pop-ed elements should start from 16
    for (auto i = 1u; i <= queue.capacity(); i++) {
      double obj1 = -10.0;
      ASSERT_TRUE(queue.pop(obj1));
      ASSERT_EQ(i + 15, obj1);
    }
    double obj1;
    ASSERT_FALSE(queue.pop(obj1));
    ASSERT_TRUE(queue.empty()) << "Buffer should be empty";
  };

  LockFreeMPMCQueue<double, 10> queue_1;
  LockFreeMPMCQueue<double> queue_2(10);
  bounded_push_test(queue_1);
  bounded_push_test(queue_2);
}

TEST(LockFreeMPMCQueue, test_lockfree_queue_push)
{
  const auto test_queue = [](auto & queue) {
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
  };

  LockFreeMPMCQueue<int, 100> queue_1;
  LockFreeMPMCQueue<int> queue_2(100);
  test_queue(queue_1);
  test_queue(queue_2);
}

TEST(LockFreeMPMCQueue, test_lockfree_queue_bounded_push)
{
  const auto bounded_push_test = [](auto & queue) {
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
      while (!done && queue.pop(value)) {
        ++consumer_count;
        std::this_thread::yield();
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
  };

  LockFreeMPMCQueue<int, 100> queue_1;
  LockFreeMPMCQueue<int> queue_2(100);
  bounded_push_test(queue_1);
  bounded_push_test(queue_2);
}
