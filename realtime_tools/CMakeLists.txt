cmake_minimum_required(VERSION 3.16)
project(realtime_tools LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
  # set the same behavior for windows as it is on linux
  export_windows_symbols()
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

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost COMPONENTS headers)
if(NOT Boost_headers_FOUND)
  find_package(Boost REQUIRED)
endif()

add_library(realtime_tools SHARED
  src/realtime_helpers.cpp
)
target_compile_features(realtime_tools PUBLIC cxx_std_17)
target_include_directories(realtime_tools PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/realtime_tools>
)
target_link_libraries(realtime_tools PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action rcpputils::rcpputils Threads::Threads Boost::boost)
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
target_link_libraries(thread_priority PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action rcpputils::rcpputils Threads::Threads)
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

  ament_add_gmock(realtime_thread_safe_box_tests test/realtime_thread_safe_box_tests.cpp)
  target_link_libraries(realtime_thread_safe_box_tests realtime_tools)

  ament_add_gmock(realtime_buffer_tests test/realtime_buffer_tests.cpp)
  target_link_libraries(realtime_buffer_tests realtime_tools)

  ament_add_gmock(lock_free_queue_tests test/lock_free_queue_tests.cpp)
  target_link_libraries(lock_free_queue_tests realtime_tools Boost::boost)
  if(UNIX)
    target_link_libraries(lock_free_queue_tests atomic)
  endif()

  ament_add_gmock(realtime_publisher_tests
                  test/realtime_publisher.test
                  test/realtime_publisher_tests.cpp)
  target_link_libraries(realtime_publisher_tests realtime_tools ${test_msgs_TARGETS})

  ament_add_gmock(realtime_server_goal_handle_tests
                  test/realtime_server_goal_handle.test
                  test/realtime_server_goal_handle_tests.cpp)
  target_link_libraries(realtime_server_goal_handle_tests realtime_tools ${test_msgs_TARGETS})

  ament_add_gmock(test_async_function_handler test/test_async_function_handler.cpp)
  target_link_libraries(test_async_function_handler
                        realtime_tools
                        thread_priority
                        ${lifecycle_msgs_TARGETS}
                        rclcpp_lifecycle::rclcpp_lifecycle)

  if(NOT WIN32)
    ament_add_gmock(realtime_mutex_tests test/realtime_mutex_tests.cpp)
    target_link_libraries(realtime_mutex_tests realtime_tools)
  endif()
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
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_package()
