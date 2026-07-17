// Copyright (c) 2025, Brian Jin
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

// Author: Brian Jin

#ifndef REALTIME_TOOLS__UTILS__PUBLISHER_INTERFACE_HPP_
#define REALTIME_TOOLS__UTILS__PUBLISHER_INTERFACE_HPP_

#include "rclcpp/publisher.hpp"

namespace realtime_tools::utils
{
template <class MessageT>
class PublisherInterface
{
public:
  virtual void publish(const MessageT & msg) = 0;
};

template <class MessageT>
class ROSPublisherWrapper : public PublisherInterface<MessageT>
{
public:
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;
  explicit ROSPublisherWrapper(PublisherSharedPtr publisher) : publisher_(publisher) {}

  void publish(const MessageT & msg) override { publisher_->publish(msg); }

private:
  PublisherSharedPtr publisher_;
};

}  // namespace realtime_tools::utils

#endif  // REALTIME_TOOLS__UTILS__PUBLISHER_INTERFACE_HPP_
