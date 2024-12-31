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

/// \author Sai Kishor Kothakota

#ifndef REALTIME_TOOLS__LOCK_FREE_QUEUE_HPP_
#define REALTIME_TOOLS__LOCK_FREE_QUEUE_HPP_

#include <chrono>
#include <mutex>
#include <type_traits>
#include <utility>

#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/stack.hpp>

namespace
{
// Trait to check if the capacity is set
template <typename T>
struct has_capacity : std::false_type
{
};

template <typename T, std::size_t Capacity>
struct has_capacity<boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>>>
: std::true_type
{
};

template <typename T, std::size_t Capacity>
struct has_capacity<boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>>> : std::true_type
{
};

template <typename T, std::size_t Capacity, bool FixedSize>
struct has_capacity<boost::lockfree::queue<
  T, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<FixedSize>>> : std::true_type
{
};

template <typename T>
struct is_spsc_queue : std::false_type
{
};

template <typename T>
struct is_spsc_queue<boost::lockfree::spsc_queue<T>> : std::true_type
{
};

template <typename T, std::size_t Capacity>
struct is_spsc_queue<boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>>>
: std::true_type
{
};

// Default case: no capacity
template <typename T>
struct get_boost_lockfree_queue_capacity
{
  static constexpr std::size_t value = 0;  // Default to 0 if capacity is not defined
};

// Specialization for queues with capacity
template <typename T, std::size_t Capacity>
struct get_boost_lockfree_queue_capacity<
  boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>>>
{
  static constexpr std::size_t value = Capacity;
};

// Specialization for queues with capacity
template <typename T, std::size_t Capacity>
struct get_boost_lockfree_queue_capacity<
  boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>>>
{
  static constexpr std::size_t value = Capacity;
};

// Specialization for queues with capacity
template <typename T, std::size_t Capacity, bool FixedSize>
struct get_boost_lockfree_queue_capacity<
  boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<FixedSize>>>
{
  static constexpr std::size_t value = Capacity;
};

}  // namespace

namespace realtime_tools
{
template <typename DataType, typename LockFreeSPSCContainer>
class LockFreeSPSCQueueBase
{
public:
  using T = DataType;

  // enable this constructor only if the queue has capacity set
  template <
    bool HasCapacity = has_capacity<LockFreeSPSCContainer>::value,
    typename std::enable_if_t<HasCapacity, int> = 0>
  LockFreeSPSCQueueBase()
  : capacity_(get_boost_lockfree_queue_capacity<LockFreeSPSCContainer>::value)
  {
  }

  // enable this constructor only if the queue has no capacity set
  template <
    bool HasCapacity = has_capacity<LockFreeSPSCContainer>::value,
    typename std::enable_if_t<!HasCapacity, int> = 1>
  explicit LockFreeSPSCQueueBase(std::size_t capacity) : data_queue_(capacity), capacity_(capacity)
  {
  }

  virtual ~LockFreeSPSCQueueBase() = default;

  [[nodiscard]] bool pop(T & data) { return data_queue_.pop(data); }

  template <typename U>
  std::enable_if_t<std::is_convertible_v<T, U>, bool> push(const U & data)
  {
    return data_queue_.push(data);
  }

  template <typename U>
  std::enable_if_t<std::is_convertible_v<T, U>, bool> bounded_push(const U & data)
  {
    if (!data_queue_.push(data)) {
      // data_queue_.pop();
      // return data_queue_.push(data);
      T dummy;
      data_queue_.pop(dummy);
      return data_queue_.push(data);
    }
    return true;
  }

  [[nodiscard]] bool push(const T & data) { return data_queue_.push(data); }

  template <
    bool IsSPSCQueue = is_spsc_queue<LockFreeSPSCContainer>::value,
    typename std::enable_if_t<IsSPSCQueue, int> = 0>
  bool empty() const { return data_queue_.read_available() == 0; }

  template <
    bool IsSPSCQueue = is_spsc_queue<LockFreeSPSCContainer>::value,
    typename std::enable_if_t<!IsSPSCQueue, int> = 1>
  bool empty() const { return data_queue_.empty(); }

  size_t capacity() const { return capacity_; }

  template <
    bool IsSPSCQueue = is_spsc_queue<LockFreeSPSCContainer>::value,
    typename std::enable_if_t<IsSPSCQueue, int> = 0>
  std::size_t size() const { return data_queue_.read_available(); }

  const LockFreeSPSCContainer & get_lockfree_container() const { return data_queue_; }

  LockFreeSPSCContainer & get_lockfree_container() { return data_queue_; }

private:
  LockFreeSPSCContainer data_queue_;
  std::size_t capacity_;
};  // class

template <class DataType, std::size_t Capacity = 0>
using LockFreeQueue = std::conditional_t<
  Capacity == 0, LockFreeSPSCQueueBase<DataType, boost::lockfree::queue<DataType>>,
  LockFreeSPSCQueueBase<
    DataType, boost::lockfree::queue<DataType, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<true>>>>;

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__LOCK_FREE_QUEUE_HPP_
