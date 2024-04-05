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

#include <gmock/gmock.h>
#include <realtime_tools/realtime_box.h>

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

using namespace realtime_tools;

TEST(RealtimeBox, empty_construct)
{
  RealtimeBox<DefaultConstructable> box;

  auto value = box.get();
  EXPECT_EQ(value.a, 10);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeBox, default_construct)
{
  DefaultConstructable data;
  data.a = 100;

  RealtimeBox<DefaultConstructable> box(data);

  auto value = box.get();
  EXPECT_EQ(value.a, 100);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeBox, non_default_constructable)
{
  RealtimeBox<NonDefaultConstructable> box(NonDefaultConstructable(-10, "hello"));

  auto value = box.get();
  EXPECT_EQ(value.a, -10);
  EXPECT_EQ(value.str, "hello");
}
TEST(RealtimeBox, standard_get)
{
  RealtimeBox<DefaultConstructable> box(DefaultConstructable{.a = 1000});

  DefaultConstructable data;
  box.get(data);
  EXPECT_EQ(data.a, 1000);
  data.a = 10000;

  box.set(data);

  auto value = box.get();
  EXPECT_EQ(value.a, 10000);
}

TEST(RealtimeBox, initializer_list)
{
  RealtimeBox<FromInitializerList> box({1, 2, 3});

  auto value = box.get();
  EXPECT_EQ(value.data[0], 1);
  EXPECT_EQ(value.data[1], 2);
  EXPECT_EQ(value.data[2], 3);
}

TEST(RealtimeBox, assignment_operator)
{
  DefaultConstructable data;
  data.a = 1000;
  RealtimeBox<DefaultConstructable> box;
  //Assignment operator is always non RT!
  box = data;

  auto value = box.get();
  EXPECT_EQ(value.a, 1000);
}
TEST(RealtimeBox, typecast_operator)
{
  RealtimeBox box(DefaultConstructable{.a = 100, .str = ""});

  //Use non RT access
  DefaultConstructable data = box;

  EXPECT_EQ(data.a, 100);

  //Use RT access -> returns std::nullopt if the mutex could not be locked
  std::optional<DefaultConstructable> rt_data_access = box;

  if (rt_data_access) {
    EXPECT_EQ(rt_data_access->a, 100);
  }
}

TEST(RealtimeBox, pointer_type)
{
  int a = 100;
  int * ptr = &a;

  RealtimeBox box(ptr);
  //This does not and should not compile!
  //auto value = box.get();

  //Instead access it via a passed function. This assues that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });

  box.tryGet([](const auto & i) { EXPECT_EQ(*i, 200); });
}

TEST(RealtimeBox, smart_ptr_type)
{
  std::shared_ptr<int> ptr = std::make_shared<int>(100);

  RealtimeBox box(ptr);
  //This does not and should not compile!
  //auto value = box.get();

  //Instead access it via a passed function. This assues that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });

    box.trySet([](const auto & p) { *p = 10; });

  box.tryGet([](const auto & p) { EXPECT_EQ(*p, 10); });
}

//These are the tests from the old RealtimeBox implementation
//They are therefore suffixed with _existing

class DefaultConstructable_existing
{
public:
  DefaultConstructable_existing() : number_(42) {}
  ~DefaultConstructable_existing() {}
  int number_;
};

TEST(RealtimeBox, default_construct_existing)
{
  DefaultConstructable_existing thing;
  thing.number_ = 5;

  RealtimeBox<DefaultConstructable_existing> box;
  box.get(thing);

  EXPECT_EQ(42, thing.number_);
}

TEST(RealtimeBox, initial_value_existing)
{
  RealtimeBox<double> box(3.14);
  double num = 0.0;
  box.get(num);
  EXPECT_DOUBLE_EQ(3.14, num);
}

TEST(RealtimeBox, set_and_get_existing)
{
  RealtimeBox<char> box('a');

  {
    const char input = 'z';
    box.set(input);
  }

  char output = 'a';
  box.get(output);
  EXPECT_EQ('z', output);
}
