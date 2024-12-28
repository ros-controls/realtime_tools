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
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/stack.hpp>
#include <type_traits>

namespace
{
  // Trait to check if the capacity is set
template <typename T>
struct has_capacity : std::false_type {};

template <typename T, std::size_t Capacity>
struct has_capacity<boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>>> : std::true_type {};

template <typename T, std::size_t Capacity>
struct has_capacity<boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>>> : std::true_type {};

template <typename T, std::size_t Capacity, bool FixedSize>
struct has_capacity<boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<FixedSize>>> : std::true_type {};
}

namespace realtime_tools
{
template <typename DataType, typename LockFreeSPSCContainer>
class LockFreeSPSCQueueBase
{
public:
  using T = DataType;

  // enable this constructor only if the queue has capacity set
  template <bool HasCapacity = has_capacity<LockFreeSPSCContainer>::value, typename std::enable_if_t<HasCapacity, int> = 0>
  LockFreeSPSCQueueBase()
  {
  }

  // enable this constructor only if the queue has no capacity set
  template <bool HasCapacity = has_capacity<LockFreeSPSCContainer>::value, typename std::enable_if_t<HasCapacity, int> = 1>
  LockFreeSPSCQueueBase(std::size_t capacity)
  : data_queue_(capacity)
  {
  }

  /**
   * @brief Constructor for objects that don't have
   * a default constructor
   * @param data The object to use as default value
   */
  explicit LockFreeSPSCQueueBase(const T & data)
  {
  }

  virtual ~LockFreeSPSCQueueBase() = default;

  [[nodiscard]] bool pop(T & data)
  {
    return data_queue_.pop(data);
  }

  [[nodiscard]] bool push(const T & data)
  {
    return data_queue_.push(data);
  }

  bool empty() const
  {
    return data_queue_.empty();
  }

  const LockFreeSPSCContainer& get_lockfree_container() const
  {
    return data_queue_;
  }

 LockFreeSPSCContainer& get_lockfree_container()
  {
    return data_queue_;
  }

private:
LockFreeSPSCContainer data_queue_;
};  // class

template <class DataType, std::size_t Capacity>
using LockFreeBuffer = LockFreeSPSCQueueBase<DataType, boost::lockfree::spsc_queue<DataType, boost::lockfree::capacity<Capacity>>>;

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__LOCK_FREE_QUEUE_HPP_
