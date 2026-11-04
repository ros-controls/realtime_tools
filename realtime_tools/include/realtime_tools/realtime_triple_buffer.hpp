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

#ifndef REALTIME_TOOLS__REALTIME_TRIPLE_BUFFER_HPP_
#define REALTIME_TOOLS__REALTIME_TRIPLE_BUFFER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace realtime_tools
{
/**
 * @brief Wait-free single-producer single-consumer channel that always holds the latest value.
 *
 * Three slots of T are preallocated: one owned by the writer, one owned by the reader and one
 * "middle" slot holding the latest published value. publish() and read() only swap slot indices
 * with a single atomic exchange, so neither side ever blocks, fails or allocates, independently of
 * what the other side is doing. Intermediate values are dropped: the reader always gets the most
 * recently published one.
 *
 * Typical use: a hardware driver thread publishes the latest sensor sample and the control loop
 * picks it up in read(), or a non-realtime callback publishes the latest command for update().
 * Unlike RealtimeThreadSafeBox::try_get(), the realtime side can never miss an update because of
 * lock contention.
 *
 * @tparam T Value type. Copy/move-assignment is done by the caller (write()) and never by the buffer
 * itself, so T can be move-only.
 *
 * @note Exactly one writer thread (write_buffer(), publish(), write()) and exactly one reader thread
 * (read(), has_new_data()). Use LockFreeQueue if every value matters or there are several producers.
 */
template <typename T>
class RealtimeTripleBuffer
{
public:
  /// Default-construct all three slots.
  RealtimeTripleBuffer() = default;

  /**
   * @brief Copy @p initial_value into all three slots.
   *
   * Useful to preallocate dynamically sized types (e.g. a std::vector sized to the number of
   * joints), so that writing into write_buffer() later on never reallocates.
   * @param initial_value Value returned by read() until the first publish().
   */
  explicit RealtimeTripleBuffer(const T & initial_value)
  : slots_{{initial_value, initial_value, initial_value}}
  {
  }

  RealtimeTripleBuffer(const RealtimeTripleBuffer &) = delete;
  RealtimeTripleBuffer & operator=(const RealtimeTripleBuffer &) = delete;
  RealtimeTripleBuffer(RealtimeTripleBuffer &&) = delete;
  RealtimeTripleBuffer & operator=(RealtimeTripleBuffer &&) = delete;

  /**
   * @brief Writer side: the slot to fill before calling publish(). Wait-free.
   *
   * The slot contains an old value (one published earlier, or the initial value), so the writer
   * must overwrite every field it cares about. Writing in place avoids copying large messages.
   * @return Reference to the writer's slot, valid until the next publish().
   */
  T & write_buffer() { return slots_[back_]; }

  /**
   * @brief Writer side: make the content of write_buffer() the latest value. Wait-free.
   *
   * If the reader did not consume the previously published value, that value is dropped.
   */
  void publish()
  {
    // Hand our slot over as the new middle and take the old middle as our next back slot.
    // acq_rel: release makes the slot content visible to the reader, acquire ensures the reader
    // has finished with the slot we get back before we write into it.
    back_ = state_.exchange(static_cast<uint8_t>(back_ | kNewDataFlag), std::memory_order_acq_rel) &
            kIndexMask;
  }

  /**
   * @brief Writer side: copy @p value into write_buffer() and publish() it. Wait-free.
   *
   * Allocation-free as long as the assignment of T is (e.g. a vector of equal or smaller size).
   */
  void write(const T & value)
  {
    write_buffer() = value;
    publish();
  }

  /// @copydoc write(const T &)
  void write(T && value)
  {
    write_buffer() = std::move(value);
    publish();
  }

  /**
   * @brief Reader side: whether a value was published since the last read(). Wait-free.
   */
  bool has_new_data() const { return (state_.load(std::memory_order_acquire) & kNewDataFlag) != 0; }

  /**
   * @brief Reader side: get the latest published value. Wait-free.
   *
   * If a new value was published, the reader's slot is swapped with it; otherwise the value from the
   * previous read() (or the initial value) is returned again.
   * @return Reference to the reader's slot, valid and not modified by the writer until the next
   * read().
   */
  const T & read()
  {
    if (has_new_data()) {
      // Give our slot back as the middle (clearing the flag) and take the latest published one.
      front_ = state_.exchange(front_, std::memory_order_acq_rel) & kIndexMask;
    }
    return slots_[front_];
  }

private:
  static constexpr uint8_t kIndexMask = 0x3;
  static constexpr uint8_t kNewDataFlag = 0x4;

  std::array<T, 3> slots_{};
  // Middle slot index (bits 0-1) and the "new data" flag (bit 2), shared by both threads
  std::atomic<uint8_t> state_{1};
  // Only touched by the reader
  uint8_t front_{0};
  // Only touched by the writer
  uint8_t back_{2};

#if defined(__cpp_lib_atomic_is_always_lock_free)
  static_assert(
    std::atomic<uint8_t>::is_always_lock_free,
    "RealtimeTripleBuffer requires a lock-free std::atomic<uint8_t> to be wait-free");
#endif
};

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__REALTIME_TRIPLE_BUFFER_HPP_
