#include <gmock/gmock.h>
#include <realtime_tools/realtime_box_best_effort.h>

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

TEST(RealtimeBoxBestEffort, empty_construct)
{
  RealtimeBoxBestEffort<DefaultConstructable> box;

  auto value = box.get();
  EXPECT_EQ(value.a, 10);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeBoxBestEffort, default_construct)
{
  DefaultConstructable data;
  data.a = 100;

  RealtimeBoxBestEffort<DefaultConstructable> box(data);

  auto value = box.get();
  EXPECT_EQ(value.a, 100);
  EXPECT_EQ(value.str, "hallo");
}

TEST(RealtimeBoxBestEffort, non_default_constructable)
{
  RealtimeBoxBestEffort<NonDefaultConstructable> box(NonDefaultConstructable(-10, "hello"));

  auto value = box.get();
  EXPECT_EQ(value.a, -10);
  EXPECT_EQ(value.str, "hello");
}

TEST(RealtimeBoxBestEffort, initializer_list)
{
  RealtimeBoxBestEffort<FromInitializerList> box({1, 2, 3});

  auto value = box.get();
  EXPECT_EQ(value.data[0], 1);
  EXPECT_EQ(value.data[1], 2);
  EXPECT_EQ(value.data[2], 3);
}

TEST(RealtimeBoxBestEffort, assignment_operator)
{
  DefaultConstructable data;
  data.a = 1000;
  RealtimeBoxBestEffort<DefaultConstructable> box;
  //Assignement operator is always non RT!
  box = data;

  auto value = box.get();
  EXPECT_EQ(value.a, 1000);
}
TEST(RealtimeBoxBestEffort, typecast_operator)
{
  RealtimeBoxBestEffort box(DefaultConstructable{.a = 100, .str = ""});

  //Use non RT access
  DefaultConstructable data = box;

  EXPECT_EQ(data.a, 100);

  //Use RT access -> returns std::nullopt if the mutex could not be locked
  std::optional<DefaultConstructable> rt_data_access = box;

  if (rt_data_access) {
    EXPECT_EQ(rt_data_access->a, 100);
  }
}

TEST(RealtimeBoxBestEffort, pointer_type)
{
  int a = 100;
  int * ptr = &a;

  RealtimeBoxBestEffort box(ptr);
  //This does not and should not compile!
  //auto value = box.get();

  //Instead access it via a passed function. This assues that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });

  box.tryGet([](const auto &i){EXPECT_EQ(*i,200);});
}

TEST(RealtimeBoxBestEffort, smart_ptr_type)
{
  std::shared_ptr<int> ptr = std::make_shared<int>(100);

  RealtimeBoxBestEffort box(ptr);
  //This does not and should not compile!
  //auto value = box.get();

  //Instead access it via a passed function. This assues that we access the data within the scope of the lock
  box.get([](const auto & i) { EXPECT_EQ(*i, 100); });

  box.set([](auto & i) { *i = 200; });

  box.get([](const auto & i) { EXPECT_EQ(*i, 200); });
}