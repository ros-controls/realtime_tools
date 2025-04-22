// Copyright (c) 2024, Lennart Nachtigall
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

#include <gmock/gmock.h>

#include <array>

#include "realtime_tools/realtime_threadsafe_box.hpp"

struct DefaultConstructable
{
  int a = 10;
  std::string str = "hallo";
};

struct NonDefaultConstructable
{
  NonDefaultConstructable(int a_, const std::string & str_) : a(a_), str(str_) {}
  int a;
  std::string str;
};

struct FromInitializerList
{
  FromInitializerList(std::initializer_list<int> list)
  {
    std::copy(list.begin(), list.end(), data.begin());
  }
  std::array<int, 3> data;
};

using realtime_tools::RealtimeThreadsafeBox;

TEST(RealtimeThreadsafeBox, empty_construct)
{
  RealtimeThreadsafeBox<DefaultConstructable> box;

  auto value = box.get();
  EXPECT_EQ(value.a, 10);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeThreadsafeBox, default_construct)
{
  DefaultConstructable data;
  data.a = 100;

  RealtimeThreadsafeBox<DefaultConstructable> box(data);

  auto value = box.get();
  EXPECT_EQ(value.a, 100);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeThreadsafeBox, non_default_constructable)
{
  RealtimeThreadsafeBox<NonDefaultConstructable> box(NonDefaultConstructable(-10, "hello"));

  auto value = box.get();
  EXPECT_EQ(value.a, -10);
  EXPECT_EQ(value.str, "hello");
}
TEST(RealtimeThreadsafeBox, standard_get)
{
  RealtimeThreadsafeBox<DefaultConstructable> box(DefaultConstructable{1000});

  DefaultConstructable data;
  box.get(data);
  EXPECT_EQ(data.a, 1000);
  data.a = 10000;

  box.set(data);

  auto value = box.get();
  EXPECT_EQ(value.a, 10000);
}

TEST(RealtimeThreadsafeBox, initializer_list)
{
  RealtimeThreadsafeBox<FromInitializerList> box({1, 2, 3});

  auto value = box.get();
  EXPECT_EQ(value.data[0], 1);
  EXPECT_EQ(value.data[1], 2);
  EXPECT_EQ(value.data[2], 3);
}

TEST(RealtimeThreadsafeBox, assignment_operator)
{
  DefaultConstructable data;
  data.a = 1000;
  RealtimeThreadsafeBox<DefaultConstructable> box;
  // Assignment operator is always non RT!
  box = data;

  auto value = box.get();
  EXPECT_EQ(value.a, 1000);
}
TEST(RealtimeThreadsafeBox, typecast_operator)
{
  RealtimeThreadsafeBox<DefaultConstructable> box(DefaultConstructable{100, ""});

  // Use non RT access
  DefaultConstructable data = box;

  EXPECT_EQ(data.a, 100);

  // Use RT access -> returns std::nullopt if the mutex could not be locked
  std::optional<DefaultConstructable> rt_data_access = box.try_get();

  if (rt_data_access) {
    EXPECT_EQ(rt_data_access->a, 100);
  }
}

TEST(RealtimeThreadsafeBox, pointer_type)
{
  int a = 100;
  int * ptr = &a;

  RealtimeThreadsafeBox<int *> box(ptr);
  // This does not and should not compile!
  // auto value = box.get();

  // Instead access it via a passed function.
  // This assures that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });

  box.try_get([](const auto & i) { EXPECT_EQ(*i, 200); });
}

TEST(RealtimeThreadsafeBox, smart_ptr_type)
{
  std::shared_ptr<int> ptr = std::make_shared<int>(100);

  RealtimeThreadsafeBox<std::shared_ptr<int>> box(ptr);

  // Instead access it via a passed function.
  // This assures that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });

  box.try_set([](const auto & p) { *p = 10; });

  box.try_get([](const auto & p) { EXPECT_EQ(*p, 10); });

  // Test that we are able to set the nullptr for pointer types
  RealtimeThreadsafeBox<std::shared_ptr<int>> box2;
  box2.set(nullptr);
}

// These are the tests from the old RealtimeThreadsafeBox implementation
// They are therefore suffixed with _existing

class DefaultConstructable_existing
{
public:
  DefaultConstructable_existing() : number_(42) {}
  ~DefaultConstructable_existing() {}
  int number_;
};

TEST(RealtimeThreadsafeBox, default_construct_existing)
{
  DefaultConstructable_existing thing;
  thing.number_ = 5;

  RealtimeThreadsafeBox<DefaultConstructable_existing> box;
  box.get(thing);

  EXPECT_EQ(42, thing.number_);
}

TEST(RealtimeThreadsafeBox, initial_value_existing)
{
  RealtimeThreadsafeBox<double> box(3.14);
  double num = 0.0;
  box.get(num);
  EXPECT_DOUBLE_EQ(3.14, num);
}

TEST(RealtimeThreadsafeBox, set_and_get_existing)
{
  RealtimeThreadsafeBox<char> box('a');

  {
    const char input = 'z';
    box.set(input);
  }

  char output = 'a';
  box.get(output);
  EXPECT_EQ('z', output);
}

TEST(RealtimeThreadsafeBox, copy_assign)
{
  RealtimeThreadsafeBox<char> box_a('a');
  RealtimeThreadsafeBox<char> box_b('b');

  // Assign b to a -> a should now contain b
  box_a = box_b;

  EXPECT_EQ('b', box_a.try_get().value());
}
TEST(RealtimeThreadsafeBox, copy)
{
  RealtimeThreadsafeBox<char> box_b('b');
  RealtimeThreadsafeBox<char> box_a(box_b);

  EXPECT_EQ('b', box_a.try_get().value());
}

TEST(RealtimeThreadsafeBox, move_assign)
{
  RealtimeThreadsafeBox<char> box_a('a');
  RealtimeThreadsafeBox<char> box_b('b');

  // Move  b to a -> a should now contain b
  box_a = std::move(box_b);

  EXPECT_EQ('b', box_a.try_get().value());
}
TEST(RealtimeThreadsafeBox, move)
{
  RealtimeThreadsafeBox<char> box_b('b');
  RealtimeThreadsafeBox<char> box_a(std::move(box_b));

  EXPECT_EQ('b', box_a.try_get().value());
}
