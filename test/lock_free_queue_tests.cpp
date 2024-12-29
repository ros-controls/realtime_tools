// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
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

#include <gmock/gmock.h>
#include <realtime_tools/lock_free_queue.hpp>

using realtime_tools::LockFreeQueue;

class DefaultConstructable
{
public:
  DefaultConstructable() : number_(42) {}
  ~DefaultConstructable() {}
  int number_;
};

TEST(LockFreeQueue, default_construct)
{
  {
    LockFreeQueue<DefaultConstructable> buffer(10);
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
  {
    LockFreeQueue<DefaultConstructable, 10> buffer;
    DefaultConstructable obj1;
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
    ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
    ASSERT_TRUE(buffer.push(DefaultConstructable())) << "Buffer should have space for one element";
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(10, buffer.capacity());
    ASSERT_EQ(42, obj1.number_);
  }
}

TEST(LockFreeQueue, initialize_value)
{
  LockFreeQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  ASSERT_TRUE(buffer.push(3.14)) << "Buffer should have space for one element";
  ASSERT_EQ(1, buffer.size()) << "Buffer should have one element";
  double obj1;
  ASSERT_TRUE(buffer.pop(obj1));
  ASSERT_DOUBLE_EQ(3.14, obj1);
  ASSERT_EQ(0, buffer.size()) << "Buffer should be empty";
}

TEST(LockFreeQueue, test_push)
{
  LockFreeQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(i, buffer.size());
    ASSERT_EQ(10, buffer.capacity());
  }
  ASSERT_FALSE(buffer.push(11)) << "Buffer should not have space for element as size is 10";
  ASSERT_EQ(10, buffer.size());
}

TEST(LockFreeQueue, test_pop)
{
  LockFreeQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 10; i++) {
    ASSERT_TRUE(buffer.push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(i, buffer.size());
  }
  for (auto i = 1; i <= 10; i++) {
    double obj1;
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(i, obj1);
    ASSERT_EQ(10 - i, buffer.size());
  }
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  double obj1;
  ASSERT_FALSE(buffer.pop(obj1)) << "Buffer should be empty";
}

TEST(LockFreeQueue, test_bounded_push)
{
  LockFreeQueue<double, 10> buffer;
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
  for (auto i = 1; i <= 25; i++) {
    ASSERT_TRUE(buffer.bounded_push(i)) << "Buffer should have space for element as size is 10";
    ASSERT_EQ(std::min(i, 10), buffer.size());
    ASSERT_EQ(10, buffer.capacity());
  }
  ASSERT_EQ(10, buffer.size());
  ASSERT_EQ(10, buffer.capacity());

  // when we start popping, the pop-ed elements should start from 16
  for (auto i = 1; i <= buffer.capacity(); i++) {
    double obj1;
    ASSERT_TRUE(buffer.pop(obj1));
    ASSERT_EQ(i + 15, obj1);
    ASSERT_EQ(buffer.capacity() - i, buffer.size());
  }
  double obj1;
  ASSERT_FALSE(buffer.pop(obj1));
  ASSERT_TRUE(buffer.empty()) << "Buffer should be empty";
}

// TEST(LockFreeQueue, write_read_non_rt)
// {
//   LockFreeQueue<int> buffer(42);

//   buffer.writeFromNonRT(28);
//   EXPECT_EQ(28, *buffer.readFromNonRT());
// }

// TEST(LockFreeQueue, initRT)
// {
//   LockFreeQueue<int> buffer(42);
//   buffer.initRT(28);
//   EXPECT_EQ(28, *buffer.readFromRT());
// }
