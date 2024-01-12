#pragma once

#include <mutex>

#include <initializer_list>
#include <optional>
namespace realtime_tools
{

/*!
    A Box that ensures thread safe access to the boxed contents. 
    Access is best effort. If it can not lock it will return.

    Note: Be carefull with pointers as this implementation will actually just copy the pointer
    See the tests for an example on how to work with pointers
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
  //Provide various constructors

  constexpr explicit RealtimeBoxBestEffort(const T & init = T{}) : value_(init) {}
  constexpr explicit RealtimeBoxBestEffort(const T && init) : value_(std::move(init)) {}

  //Only enabled for types that can be constructed from an initializer list
  template <typename U = T>
  constexpr RealtimeBoxBestEffort(
    const std::initializer_list<U> & init,
    std::enable_if_t<std::is_constructible_v<U, std::initializer_list>>)
  : value_(init)
  {
  }

  /**
     * @brief set a new content with best effort
     * @return false if mutex could not be locked
    */
  bool set(const T & value)
  {
    std::unique_lock<std::mutex> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return false;
    }
    value_ = value;
    return true;
  }
  /**
     * @brief get the content with best effort
     * @return std::nullopt if content could not be access, otherwise the content is returned
    */
  [[nodiscard]] std::optional<T> get() const
  {
    std::unique_lock<std::mutex> guard(lock_, std::defer_lock);
    if (!guard.try_lock()) {
      return std::nullopt;
    }
    return value_;
  }

  /**
     * @brief set the content and wait until the mutex could be locked (RealtimeBox behaviour)
     * @return true
    */
  bool setNonRT(const T & value)
  {
    std::lock_guard<std::mutex> guard(lock_);
    value_ = value;
    //Also return a bool in order to mimic the behaviour from 'set'
    return true;
  }
  /**
     * @brief get the content and wait until the mutex could be locked (RealtimeBox behaviour)
     * @return copy of the value
    */
  T getNonRT() const
  {
    std::lock_guard<std::mutex> guard(lock_);
    return value_;
  }
  //Make the usage easier by providing a custom assignment operator and a custom conversion operator
  //Only to be used from non-RT!
  void operator=(const T & value) { set(value); }

  operator T() const
  {
    //Only makes sense with the getNonRT method otherwise we would return an std::optional
    return getNonRT();
  }
  operator std::optional<T>() const { return get(); }

  //In case one wants to actually use a pointer in this implementation we allow accessing the lock directly.
  //Note: Be carefull with lock.unlock(). It may only be called from the thread that locked the mutext!
  const mutex_t & getMutex() const { return lock_; }
  mutex_t & getMutex() { return lock_; }

private:
  T value_;
  mutable mutex_t lock_;
};
}  // namespace realtime_tools