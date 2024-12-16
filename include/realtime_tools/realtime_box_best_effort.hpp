// Copyright (c) 2024, Lennart Nachtigall
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: Lennart Nachtigall

#ifndef REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_HPP_
#define REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_HPP_

#include <functional>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <utility>

#include <rcpputils/pointer_traits.hpp>

namespace realtime_tools
{

template <typename T>
constexpr auto is_ptr_or_smart_ptr = rcpputils::is_pointer<T>::value;

/*!
    A Box that ensures thread safe access to the boxed contents.
    Access is best effort. If it can not lock it will return.

    NOTE about pointers:
    You can use pointers with this box but the access will be different.
    Only use the get/set methods that take function pointer for accessing the internal value.
*/
template <class T, typename mutex_type = std::mutex>
class RealtimeBoxBestEffort
{
  static_assert(
    std::is_same_v<mutex_type, std::mutex> || std::is_same_v<mutex_type, std::recursive_mutex>);
  static_assert(std::is_copy_constructible_v<T>, "Passed type must be copy constructible");

public:
  using mutex_t = mutex_type;
  using type = T;
  // Provide various constructors
  constexpr explicit RealtimeBoxBestEffort(const T & init = T{}) : value_(init) {}
  constexpr explicit RealtimeBoxBestEffort(const T && init) : value_(std::move(init)) {}

  // Only enabled for types that can be constructed from an initializer list
  template <typename U = T>
  constexpr RealtimeBoxBestEffort(
    const std::initializer_list<U> & init,
    std::enable_if_t<std::is_constructible_v<U, std::initializer_list<U>>>)
  : value_(init)
  {
  }

  /**
   * @brief set a new content with best effort
   * @return false if mutex could not be locked
   * @note disabled for pointer types
   */
  template <typename U = T>
  typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, bool> trySet(const T & value)
  {
    std::unique_lock<mutex_t> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return false;
    }
    value_ = value;
    return true;
  }
  /**
   * @brief access the content readable with best effort
   * @return false if the mutex could not be locked
   * @note only safe way to access pointer type content (rw)
   */
  bool trySet(const std::function<void(T &)> & func)
  {
    std::unique_lock<mutex_t> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return false;
    }

    func(value_);
    return true;
  }
  /**
   * @brief get the content with best effort
   * @return std::nullopt if content could not be access, otherwise the content is returned
   */
  template <typename U = T>
  [[nodiscard]] typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, std::optional<U>> tryGet() const
  {
    std::unique_lock<mutex_t> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return std::nullopt;
    }
    return value_;
  }
  /**
   * @brief access the content (r) with best effort
   * @return false if the mutex could not be locked
   * @note only safe way to access pointer type content (r)
   */
  bool tryGet(const std::function<void(const T &)> & func)
  {
    std::unique_lock<mutex_t> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return false;
    }

    func(value_);
    return true;
  }

  /**
   * @brief set the content and wait until the mutex could be locked (RealtimeBox behavior)
   * @return true
   */
  template <typename U = T>
  typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, void> set(const T & value)
  {
    std::lock_guard<mutex_t> guard(lock_);
    // cppcheck-suppress missingReturn
    value_ = value;
  }
  /**
   * @brief access the content (rw) and wait until the mutex could locked
   */
  void set(const std::function<void(T &)> & func)
  {
    std::lock_guard<mutex_t> guard(lock_);
    func(value_);
  }

  /**
   * @brief get the content and wait until the mutex could be locked (RealtimeBox behaviour)
   * @return copy of the value
   */
  template <typename U = T>
  [[nodiscard]] typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, U> get() const
  {
    std::lock_guard<mutex_t> guard(lock_);
    return value_;
  }
  /**
   * @brief get the content and wait until the mutex could be locked
   * @note same signature as in the existing RealtimeBox<T>
   */
  template <typename U = T>
  typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, void> get(T & in) const
  {
    std::lock_guard<mutex_t> guard(lock_);
    // cppcheck-suppress missingReturn
    in = value_;
  }
  /**
   * @brief access the content (r) and wait until the mutex could be locked
   * @note only safe way to access pointer type content (r)
   * @note same signature as in the existing RealtimeBox<T>
   */
  void get(const std::function<void(const T &)> & func)
  {
    std::lock_guard<mutex_t> guard(lock_);
    func(value_);
  }

  /**
   * @brief provide a custom assignment operator for easier usage
   * @note only to be used from non-RT!
   */
  template <typename U = T>
  typename std::enable_if_t<!is_ptr_or_smart_ptr<U>, void> operator=(const T & value)
  {
    set(value);
  }

  /**
   * @brief provide a custom conversion operator
   * @note Can only be used from non-RT!
   */
  template <typename U = T, typename = typename std::enable_if_t<!is_ptr_or_smart_ptr<U>>>
  [[nodiscard]] operator T() const
  {
    // Only makes sense with the getNonRT method otherwise we would return an std::optional
    return get();
  }
  /**
   * @brief provide a custom conversion operator
   * @note Can be used from non-RT and RT contexts
   */
  template <typename U = T, typename = typename std::enable_if_t<!is_ptr_or_smart_ptr<U>>>
  [[nodiscard]] operator std::optional<T>() const
  {
    return tryGet();
  }

  // In case one wants to actually use a pointer
  // in this implementation we allow accessing the lock directly.
  // Note: Be careful with lock.unlock().
  // It may only be called from the thread that locked the mutex!
  [[nodiscard]] const mutex_t & getMutex() const { return lock_; }
  [[nodiscard]] mutex_t & getMutex() { return lock_; }

private:
  T value_;
  mutable mutex_t lock_;
};
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_HPP_
