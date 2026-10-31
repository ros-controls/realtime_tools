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

#ifndef REALTIME_TOOLS__LOCK_FREE_POOLED_QUEUE_HPP_
#define REALTIME_TOOLS__LOCK_FREE_POOLED_QUEUE_HPP_

#include <array>
#include <cstddef>
#include <utility>

#include "realtime_tools/lock_free_queue.hpp"

namespace realtime_tools
{
/**
 * @brief Wait-free SPSC queue for large or move-only types, backed by a preallocated pool.
 *
 * The elements live in a fixed pool of Capacity slots. Only slot indices travel through two
 * LockFreeSPSCQueues: one with the filled slots (producer -> consumer) and one with the free slots
 * (consumer -> producer). Elements are therefore never copied by the queue, can be move-only and are
 * written/read in place with produce()/consume().
 *
 * @tparam T Element type. Must be default constructible (all slots are constructed up front).
 * @tparam Capacity Number of slots in the pool, must be > 0.
 *
 * @note Exactly one producer thread (push/produce) and exactly one consumer thread (pop/consume).
 * empty() may only be called from the consumer thread.
 * @note The queue holds Capacity * sizeof(T) bytes inline, so allocate it on the heap or as a member
 * if it is large.
 */
template <typename T, std::size_t Capacity>
class LockFreePooledSPSCQueue
{
  static_assert(Capacity > 0, "Capacity must be greater than zero");

public:
  /// Construct the queue with all slots free.
  LockFreePooledSPSCQueue()
  {
    for (std::size_t i = 0; i < Capacity; ++i) {
      (void)free_.push(i);  // cannot fail, the free queue has room for every slot
    }
  }

  /**
   * @brief Fill a free slot in place and publish it to the consumer.
   * @param fill Callable invoked as fill(T &) with the slot to write. It must not throw: the slot
   * would be lost from the pool otherwise.
   * @return true If the slot was published, false if all slots are in use (nothing is called).
   * @note Producer thread only.
   */
  template <typename Fill>
  [[nodiscard]] bool produce(Fill && fill)
  {
    std::size_t index;
    if (!free_.pop(index)) {
      return false;
    }
    fill(slots_[index]);
    return filled_.push(index);  // cannot fail, at most Capacity indices exist
  }

  /**
   * @brief Move the value into a free slot and publish it to the consumer.
   * @return true If pushed, false if all slots are in use.
   * @note Producer thread only. Uses T's move assignment into the preallocated slot, so use
   * produce() instead if that assignment can allocate or deallocate.
   */
  [[nodiscard]] bool push(T && value)
  {
    return produce([&value](T & slot) { slot = std::move(value); });
  }

  /**
   * @brief Read the oldest published slot in place and release it back to the pool.
   * @param use Callable invoked as use(T &) with the slot to read. It must not throw.
   * @return true If a slot was consumed, false if the queue is empty (nothing is called).
   * @note Consumer thread only.
   */
  template <typename Use>
  [[nodiscard]] bool consume(Use && use)
  {
    std::size_t index;
    if (!filled_.pop(index)) {
      return false;
    }
    use(slots_[index]);
    return free_.push(index);  // cannot fail, at most Capacity indices exist
  }

  /**
   * @brief Move the oldest element out of the queue.
   * @return true If an element was popped, false if the queue is empty.
   * @note Consumer thread only.
   */
  [[nodiscard]] bool pop(T & out)
  {
    return consume([&out](T & slot) { out = std::move(slot); });
  }

  /// @return true if there is nothing to consume. Consumer thread only.
  bool empty() const { return filled_.empty(); }

private:
  std::array<T, Capacity> slots_;
  LockFreeSPSCQueue<std::size_t, Capacity> filled_;
  LockFreeSPSCQueue<std::size_t, Capacity> free_;
};
}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__LOCK_FREE_POOLED_QUEUE_HPP_
