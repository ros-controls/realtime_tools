cmake_minimum_required(VERSION 3.16)
project(realtime_tools LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  Threads
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(realtime_tools SHARED
  src/realtime_clock.cpp
)
target_compile_features(realtime_tools PUBLIC cxx_std_17)
target_include_directories(realtime_tools PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/realtime_tools>
)
ament_target_dependencies(realtime_tools PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# A library to detect a realtime kernel and set thread priority, if one is found
add_library(thread_priority SHARED
  src/thread_priority.cpp
)
target_compile_features(thread_priority PUBLIC cxx_std_17)
target_include_directories(thread_priority PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/realtime_tools>
)
ament_target_dependencies(thread_priority PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Unit Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gmock(realtime_box_tests test/realtime_box_tests.cpp)
  target_link_libraries(realtime_box_tests realtime_tools)

  ament_add_gmock(realtime_buffer_tests test/realtime_buffer_tests.cpp)
  target_link_libraries(realtime_buffer_tests realtime_tools)

  ament_add_gmock(realtime_clock_tests test/realtime_clock_tests.cpp)
  target_link_libraries(realtime_clock_tests realtime_tools)

  ament_add_gmock(realtime_publisher_tests_non_polling
                  test/realtime_publisher_non_polling.test
                  test/realtime_publisher_tests_non_polling.cpp)
  target_link_libraries(realtime_publisher_tests_non_polling realtime_tools)
  ament_target_dependencies(realtime_publisher_tests_non_polling test_msgs)

  ament_add_gmock(realtime_publisher_tests
                  test/realtime_publisher.test
                  test/realtime_publisher_tests.cpp)
  target_link_libraries(realtime_publisher_tests realtime_tools)
  ament_target_dependencies(realtime_publisher_tests test_msgs)

  ament_add_gmock(realtime_server_goal_handle_tests
                  test/realtime_server_goal_handle.test
                  test/realtime_server_goal_handle_tests.cpp)
  target_link_libraries(realtime_server_goal_handle_tests realtime_tools)
  ament_target_dependencies(realtime_server_goal_handle_tests test_msgs)
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include/realtime_tools
)
install(TARGETS realtime_tools thread_priority
  EXPORT export_realtime_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_realtime_tools HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
