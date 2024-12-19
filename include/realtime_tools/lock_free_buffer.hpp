// Copyright 2024 PAL Robotics S.L.
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

#ifndef REALTIME_TOOLS__LOCK_FREE_BUFFER_HPP_
#define REALTIME_TOOLS__LOCK_FREE_BUFFER_HPP_

#include <chrono>
#include <mutex>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/stack.hpp>

namespace realtime_tools
{
template <class DataType, class LockFreeContainer>
class LockFreeBufferBase
{
public:
  LockFreeBufferBase() : new_data_available_(false)
  {
  }

  /**
   * @brief Constructor for objects that don't have
   * a default constructor
   * @param data The object to use as default value
   */
  explicit LockFreeBufferBase(const T & data)
  {
  }

  virtual ~LockFreeBufferBase() = default;

  T readFromRT()
  {
    return realtime_data_.pop();
  }

  T readFromNonRT() const
  {
  }

  void writeFromNonRT(const T & data)
  {
  }

  void initialize(const T & data)
  {
    data_buffer_.push(data);
  }

  void reset()
  {
  }

private:
LockFreeContainer data_buffer_;
};  // class

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__LOCK_FREE_BUFFER_HPP_
