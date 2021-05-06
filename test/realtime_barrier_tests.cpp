/*
 * Copyright (c) 2021, FlyingEinstein.com
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
#include <realtime_tools/realtime_barrier.hpp>

using realtime_tools::MemoryBarrier;
using realtime_tools::ReadBarrier;
using realtime_tools::WriteBarrier;

class DefaultConstructable
{
public:
  DefaultConstructable()
  : number_(42) {}
  ~DefaultConstructable() {}
  int number_;
};

/*
 * MemoryBarriers - base thread barrier mechanisms
 */

TEST(MemoryBarrier, default_construct)
{
  MemoryBarrier<DefaultConstructable> buffer;
  decltype(buffer)::DirectAccess<> da(buffer);
  EXPECT_EQ(42, da->number_);
}

TEST(MemoryBarrier, default_construct_new_data_flag_is_false)
{
  MemoryBarrier<DefaultConstructable> buffer;
  EXPECT_FALSE(buffer.new_data_available());
}

TEST(MemoryBarrier, initialize_new_data)
{
  MemoryBarrier<DefaultConstructable> buffer;
  DefaultConstructable x;
  x.number_ = 12;
  buffer.initialize(x);
  EXPECT_FALSE(buffer.new_data_available());    // should still be false
  decltype(buffer)::DirectAccess<> da(buffer);
  EXPECT_EQ(12, da->number_);
}


/*
 * MemoryBarrier::DirectAccess - directly manipulate a memory barrier
 */

TEST(DirectAccess, new_data_available_flag)
{
  MemoryBarrier<DefaultConstructable> buffer;
  decltype(buffer)::DirectAccess<> da(buffer);
  EXPECT_EQ(42, da->number_);
  EXPECT_FALSE(da.new_data_available());
  da.new_data_available(true);
  EXPECT_TRUE(da.new_data_available());
}


TEST(DirectAccess, reset_releases_resource)
{
  MemoryBarrier<DefaultConstructable> buffer;
  decltype(buffer)::DirectAccess<realtime_tools::realtime> rt(buffer);
  EXPECT_NE(rt.get(), nullptr);
  EXPECT_TRUE(rt);
  rt->number_ = 12;
  EXPECT_EQ(12, rt->number_);
  rt.reset();
  EXPECT_EQ(rt.get(), nullptr);
  EXPECT_FALSE(rt);
  decltype(buffer)::DirectAccess<realtime_tools::realtime> rt2(buffer);
  EXPECT_NE(rt2.get(), nullptr);
  EXPECT_TRUE(rt2);
  EXPECT_EQ(12, rt2->number_);
}

TEST(DirectAccess, swap_buffers)
{
  MemoryBarrier<DefaultConstructable> buffer;
  decltype(buffer)::DirectAccess<realtime_tools::non_realtime> nrt(buffer);
  nrt->number_ = 24;
  nrt.reset();    // required or RT request below will cause a deadlock

  decltype(buffer)::DirectAccess<realtime_tools::realtime> rt(buffer);
  rt->number_ = 12;
  rt.swap();
  EXPECT_EQ(24, rt->number_);
  rt.reset();     // required to prevent deadlock again

  // now check non-realtime
  decltype(buffer)::DirectAccess<realtime_tools::non_realtime> nrt2(buffer);
  EXPECT_EQ(12, nrt2->number_);
}

TEST(DirectAccess, realtime_fails_on_locked_resource)
{
  MemoryBarrier<DefaultConstructable> buffer;
  decltype(buffer)::DirectAccess<realtime_tools::non_realtime> nrt(buffer);

  decltype(buffer)::DirectAccess<realtime_tools::realtime> rt(buffer);
  EXPECT_EQ(rt.get(), nullptr);
  EXPECT_FALSE(rt);
}


/*
 * Read Barriers
 */

TEST(ReadBarrier, default_construct)
{
  ReadBarrier<DefaultConstructable> buffer;
  decltype(buffer)::MemoryBarrierType::DirectAccess<> da(buffer);
  EXPECT_EQ(42, da->number_);
}

TEST(ReadBarrier, write_from_RT_to_nRT)
{
  // write state to non-RT
  ReadBarrier<DefaultConstructable> buffer;
  decltype(buffer)::MemoryBarrierType::DirectAccess<> da_writer(buffer);
  da_writer->number_ = 8;                     // using smart_ptr-like semantics
  da_writer.new_data_available(true);   // indicate to non-RT there is new state data
  da_writer.reset();                          // unlock the state barrier
}

TEST(ReadBarrier, read_on_nRT)
{
  // write state to non-RT
  ReadBarrier<DefaultConstructable> buffer;

  // repeat RT write
  decltype(buffer)::MemoryBarrierType::DirectAccess<> da_writer(buffer);
  da_writer->number_ = 8;                     // using smart_ptr-like semantics
  da_writer.new_data_available(true);   // indicate to non-RT there is new state data
  da_writer.reset();                          // unlock the state barrier

  // now test the read from nRT thread
  // get the data from RT thread and write into our user
  DefaultConstructable state;
  EXPECT_TRUE(buffer.memory().new_data_available());
  EXPECT_TRUE(buffer.pull(state));
  EXPECT_EQ(8, state.number_);
}


/*
 * Write Barriers
 */

TEST(WriteBarrier, default_construct)
{
  WriteBarrier<DefaultConstructable> buffer;
  decltype(buffer)::MemoryBarrierType::DirectAccess<> da(buffer.memory());
  EXPECT_EQ(42, da->number_);
}

TEST(WriteBarrier, write_to_RT_from_nRT)
{
  WriteBarrier<DefaultConstructable> buffer;

  DefaultConstructable command;
  command.number_ = 52;

  // write commands to RT thread
  EXPECT_FALSE(buffer.memory().new_data_available());
  EXPECT_TRUE(buffer.push(command));
  EXPECT_TRUE(buffer.memory().new_data_available());
}

TEST(WriteBarrier, read_on_RT)
{
  WriteBarrier<DefaultConstructable> buffer;

  DefaultConstructable command;
  command.number_ = 52;

  // write commands to RT thread
  EXPECT_TRUE(buffer.push(command));

  // confirm read from RT
  decltype(buffer)::MemoryBarrierType::DirectAccess<> da_reader(buffer);
  EXPECT_TRUE(da_reader.new_data_available());
  EXPECT_EQ(da_reader->number_, 52);             // using smart_ptr-like semantics
}
