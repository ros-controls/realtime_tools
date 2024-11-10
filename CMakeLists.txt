cmake_minimum_required(VERSION 3.16)
project(realtime_tools LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
  -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  Threads
  rcpputils
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(realtime_tools SHARED
  src/realtime_clock.cpp
  src/realtime_helpers.cpp
)
target_compile_features(realtime_tools PUBLIC cxx_std_17)
target_include_directories(realtime_tools PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/realtime_tools>
)
ament_target_dependencies(realtime_tools PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
if(UNIX)
  target_link_libraries(realtime_tools PUBLIC cap)
endif()

# A library to detect a realtime kernel and set thread priority, if one is found
add_library(thread_priority SHARED
  src/realtime_helpers.cpp
)
target_compile_features(thread_priority PUBLIC cxx_std_17)
target_include_directories(thread_priority PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/realtime_tools>
)
ament_target_dependencies(thread_priority PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
if(UNIX)
  target_link_libraries(thread_priority PUBLIC cap)
endif()

# Unit Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gmock(thread_priority_tests test/thread_priority_tests.cpp)
  target_link_libraries(thread_priority_tests thread_priority)

  ament_add_gmock(realtime_box_tests test/realtime_box_tests.cpp)
  target_link_libraries(realtime_box_tests realtime_tools)

  ament_add_gmock(realtime_box_best_effort_tests test/realtime_box_best_effort_tests.cpp)
  target_link_libraries(realtime_box_best_effort_tests realtime_tools)

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

  ament_add_gmock(test_async_function_handler test/test_async_function_handler.cpp)
  target_link_libraries(test_async_function_handler realtime_tools thread_priority)
  ament_target_dependencies(test_async_function_handler lifecycle_msgs rclcpp_lifecycle)
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
