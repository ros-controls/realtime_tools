/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gmock/gmock.h>
#include <realtime_tools/realtime_buffer.h>

using realtime_tools::RealtimeBuffer;

class DefaultConstructable
{
  public:
    DefaultConstructable() : number_(42) {};
    ~DefaultConstructable() {};
    int number_;
};

TEST(RealtimeBuffer, default_construct)
{
  RealtimeBuffer<DefaultConstructable> buffer;
  EXPECT_EQ(42, buffer.readFromRT()->number_);
}

TEST(RealtimeBuffer, initial_value)
{
  RealtimeBuffer<double> buffer(3.14);
  EXPECT_DOUBLE_EQ(3.14, *buffer.readFromRT());
}

TEST(RealtimeBuffer, copy_construct)
{
  const RealtimeBuffer<char> buffer('a');
  RealtimeBuffer<char> buffer_copy(buffer);
  EXPECT_EQ('a', *buffer_copy.readFromRT());
}

TEST(RealtimeBuffer, assignment_operator)
{
  const RealtimeBuffer<char> buffer('a');
  RealtimeBuffer<char> buffer2('z');

  EXPECT_EQ('z', *buffer2.readFromRT());
  buffer2 = buffer;
  EXPECT_EQ('a', *buffer2.readFromRT());
}

TEST(RealtimeBuffer, write_read_non_rt)
{
  RealtimeBuffer<int> buffer(42);

  buffer.writeFromNonRT(28);
  EXPECT_EQ(28, *buffer.readFromNonRT());
}

TEST(RealtimeBuffer, initRT)
{
  RealtimeBuffer<int> buffer(42);
  buffer.initRT(28);
  EXPECT_EQ(28, *buffer.readFromRT());
}
