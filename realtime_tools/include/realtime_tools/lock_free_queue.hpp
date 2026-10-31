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

namespace realtime_tools
{
namespace detail
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

// Traits to check if the queue is spsc_queue
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

// Traits to get the capacity of the queue
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
struct get_boost_lockfree_queue_capacity<boost::lockfree::queue<
  T, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<FixedSize>>>
{
  static constexpr std::size_t value = Capacity;
};

}  // namespace detail

/**
 * @brief A base class for lock-free queues.
 *
 * This class provides a base implementation for lock-free queues with various functionalities
 * such as pushing, popping, and checking the state of the queue. It supports both single-producer
 * single-consumer (SPSC) and multiple-producer multiple-consumer (MPMC) queues.
 *
 * @tparam DataType The type of data to be stored in the queue.
 * @tparam LockFreeContainer The underlying lock-free container type - Typically boost::lockfree::spsc_queue or boost::lockfree::queue with their own template parameters
 */
template <typename DataType, typename LockFreeContainer>
class LockFreeQueueBase
{
public:
  using T = DataType;

  /**
   * @brief Construct a new LockFreeQueueBase object
   * @note This constructor is enabled only if the LockFreeContainer has a capacity set
   */
  template <
    bool HasCapacity = detail::has_capacity<LockFreeContainer>::value,
    typename std::enable_if_t<HasCapacity, int> = 0>
  LockFreeQueueBase() : capacity_(detail::get_boost_lockfree_queue_capacity<LockFreeContainer>::value)
  {
  }

  /**
   * @brief Construct a new LockFreeQueueBase object
   * @param capacity Capacity of the queue
   * @note This constructor is enabled only if the LockFreeContainer has no capacity set
   */
  template <
    bool HasCapacity = detail::has_capacity<LockFreeContainer>::value,
    typename std::enable_if_t<!HasCapacity, int> = 1>
  explicit LockFreeQueueBase(std::size_t capacity) : data_queue_(capacity), capacity_(capacity)
  {
  }

  /**
   * @brief Pop the data from the queue
   * @param data Data to be popped
   * @return true If the data is popped successfully
   * @return false If the queue is empty or the data could not be popped
   */
  [[nodiscard]] bool pop(T & data) { return data_queue_.pop(data); }

  /**
   * @brief Pop the data from the queue
   * @param data Data to be popped
   * @return true If the data is popped successfully
   * @return false If the queue is empty or the data could not be popped
   * @note This function is enabled only if the data type is convertible to the template type of the queue
   */
  template <typename U>
  [[nodiscard]] std::enable_if_t<std::is_convertible_v<T, U>, bool> pop(U & data)
  {
    return data_queue_.pop(data);
  }

  /**
   * @brief Push the data into the queue
   * @param data Data to be pushed
   * @return true If the data is pushed successfully
   * @return false If the queue is full or the data could not be pushed
   */

  [[nodiscard]] bool push(const T & data)
  {
    if constexpr (detail::is_spsc_queue<LockFreeContainer>::value) {
      return data_queue_.push(data);
    } else {
      return data_queue_.bounded_push(data);
    }
  }

  /**
   * @brief Push the data into the queue
   * @param data Data to be pushed
   * @return true If the data is pushed successfully
   * @return false If the queue is full or the data could not be pushed
   * @note This function is enabled only if the data type is convertible to the template type of the queue (U -> T)
   */
  template <typename U>
  [[nodiscard]] std::enable_if_t<std::is_convertible_v<U, T>, bool> push(const U & data)
  {
    if constexpr (detail::is_spsc_queue<LockFreeContainer>::value) {
      return data_queue_.push(data);
    } else {
      return data_queue_.bounded_push(data);
    }
  }

  /**
   * @brief The bounded_push function pushes the data into the queue and pops the oldest data if the queue is full
   * @param data Data to be pushed
   * @return true If the data is pushed successfully
   * @return false If the data could not be pushed
   * @note This function is enabled only if the data type is convertible to the template type of the
   * queue (U -> T)
   * @note For a SPSC Queue, to be used in single-threaded applications only (see the warning below)
   * @warning For a SPSC Queue, NEVER call this from the producer thread while another thread pops.
   * To make room it calls pop(), which only the consumer is allowed to call, so the two threads
   * race on the read index (undefined behavior: lost or duplicated elements). It is only safe when
   * one thread does all the pushing and popping, so it is not suitable for RT <-> non-RT exchange.
   * Alternatives for that case:
   * - Use push() and handle the failure (e.g. count the dropped elements and report them
   *   from the non-RT side).
   * - If only the most recent value matters, use RealtimeThreadsafeBox or RealtimeBuffer.
   * - If dropping the oldest element across threads is required, use LockFreeMPMCQueue. Its
   *   bounded_push is thread-safe, but pop and push are separate steps, so it can still return
   *   false under contention.
   */
  template <typename U>
  [[nodiscard]] std::enable_if_t<std::is_convertible_v<U, T>, bool> bounded_push(const U & data)
  {
    if (!push(data)) {
      T dummy;
      data_queue_.pop(dummy);
      return push(data);
    }
    return true;
  }

  /**
   * @brief Get the latest data from the queue
   * @param data Data to be filled with the latest data
   * @return true If the data is filled with the latest data, false otherwise
   * @return false If the queue is empty
   * @note This function consumes all the data in the queue until the last data and returns the last element of the queue
   * @note For a SPSC Queue, it must be called from the consumer thread. The older elements are
   * discarded without being copied, so only the latest element is copied out.
   * For other queues, every element is copied into data, the last one wins.
   */
  [[nodiscard]] bool get_latest(T & data)
  {
    if constexpr (detail::is_spsc_queue<LockFreeContainer>::value) {
      // read_available() can only grow while we run (the producer only adds), so skipping while
      // more than one element is available never drops the element that is popped below
      while (data_queue_.read_available() > 1) {
        data_queue_.consume_one([](const T &) {});
      }
      return data_queue_.pop(data);
    } else {
      return data_queue_.consume_all([&data](const T & d) { data = d; }) > 0;
    }
  }

  /**
   * @brief Check if the queue is empty
   * @return true If the queue is empty
   * @return false If the queue is not empty
   * @note The result is only accurate, if no other thread modifies the queue. Therefore, it is rarely practical to use this value in program logic.
   * @note For a SPSC Queue, it should only be called from the consumer thread where pop is called. If need to be called from producer thread, use write_available() instead.
   */
  bool empty() const
  {
    if constexpr (detail::is_spsc_queue<LockFreeContainer>::value) {
      return data_queue_.read_available() == 0;
    } else {
      return data_queue_.empty();
    }
  }

  /**
   * @brief Get the capacity of the queue
   * @return std::size_t Capacity of the queue
   */
  size_t capacity() const { return capacity_; }

  /**
   * @brief Get the size of the queue
   * @return std::size_t Size of the queue
   * @note This function is enabled only if the queue is a spsc_queue
   */
  template <
    bool IsSPSCQueue = detail::is_spsc_queue<LockFreeContainer>::value,
    typename std::enable_if_t<IsSPSCQueue, int> = 0>
  std::size_t size() const
  {
    return data_queue_.read_available();
  }

  /**
   * @brief The method to check if the queue is lock free
   * @return true If the queue is lock free, false otherwise
   * @warning It only checks, if the queue head and tail nodes and the freelist can
   * be modified in a lock-free manner. On most platforms, the whole implementation
   * is lock-free, if this is true. Using c++0x-style atomics, there is no possibility
   * to provide a completely accurate implementation, because one would need to test
   * every internal node, which is impossible if further nodes will be allocated from
   * the operating system.
   * @link https://www.boost.org/doc/libs/1_74_0/doc/html/boost/lockfree/queue.html
   */
  bool is_lock_free() const
  {
    return (detail::is_spsc_queue<LockFreeContainer>::value) || data_queue_.is_lock_free();
  }

  /**
   * @brief Get the lockfree container
   * @return const LockFreeContainer& Reference to the lockfree container
   */
  const LockFreeContainer & get_lockfree_container() const { return data_queue_; }

  /**
   * @brief Get the lockfree container
   * @return LockFreeContainer& Reference to the lockfree container
   */
  LockFreeContainer & get_lockfree_container() { return data_queue_; }

private:
  LockFreeContainer data_queue_;
  std::size_t capacity_;
};  // class

/**
 * @brief Lock-free Single Producer Single Consumer Queue
 * @tparam DataType Type of the data to be stored in the queue
 * @tparam Capacity Capacity of the queue
 */
template <class DataType, std::size_t Capacity = 0>
using LockFreeSPSCQueue = std::conditional_t<
  Capacity == 0, LockFreeQueueBase<DataType, boost::lockfree::spsc_queue<DataType>>,
  LockFreeQueueBase<
    DataType, boost::lockfree::spsc_queue<DataType, boost::lockfree::capacity<Capacity>>>>;

/**
 * @brief Lock-free Multiple Producer Multiple Consumer Queue
 * @tparam DataType Type of the data to be stored in the queue
 * @tparam Capacity Capacity of the queue
 * @tparam FixedSize Fixed size of the queue
 */
template <class DataType, std::size_t Capacity = 0, bool FixedSize = true>
using LockFreeMPMCQueue = std::conditional_t<
  Capacity == 0,
  LockFreeQueueBase<
    DataType, boost::lockfree::queue<DataType, boost::lockfree::fixed_sized<FixedSize>>>,
  LockFreeQueueBase<
    DataType,
    boost::lockfree::queue<
      DataType, boost::lockfree::capacity<Capacity>, boost::lockfree::fixed_sized<FixedSize>>>>;

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__LOCK_FREE_QUEUE_HPP_
