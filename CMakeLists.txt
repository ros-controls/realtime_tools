cmake_minimum_required(VERSION 3.5)
project(realtime_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} src/realtime_clock.cpp)
ament_target_dependencies(${PROJECT_NAME} "rclcpp")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Unit Tests
if (BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

#  ament_add_gtest(realtime_box_tests test/realtime_box_tests.cpp)
#  if (TARGET realtime_box_tests)
#    target_link_libraries(realtime_box_tests ${PROJECT_NAME})
#  endif()
#
#  ament_add_gtest(realtime_buffer_tests test/realtime_buffer_tests.cpp)
#  if (TARGET realtime_buffer_tests)
#    target_link_libraries(realtime_buffer_tests ${PROJECT_NAME})
#  endif()

  ament_add_gmock(realtime_clock_tests test/realtime_clock_tests.cpp)
  target_link_libraries(realtime_clock_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  # find_package(rostest REQUIRED)
  # add_rostest_gtest(realtime_publisher_tests test/realtime_publisher.test test/realtime_publisher_tests.cpp)
  # if (TARGET realtime_publisher_tests)
  #   find_package(std_msgs REQUIRED)
  #   target_link_libraries(realtime_publisher_tests ${PROJECT_NAME} ${std_msgs_LIBRARIES})
  #   target_include_directories(realtime_publisher_tests PRIVATE ${std_msgs_INCLUDE_DIRS})
  # endif()

  # add_rostest_gtest(realtime_server_goal_handle_tests test/realtime_server_goal_handle.test test/realtime_server_goal_handle_tests.cpp)
  # if (TARGET realtime_server_goal_handle_tests)
  #   find_package(actionlib REQUIRED)
  #   target_link_libraries(realtime_server_goal_handle_tests ${PROJECT_NAME} ${actionlib_LIBRARIES})
  #   target_include_directories(realtime_publisher_tests PRIVATE ${actionlib_INCLUDE_DIRS})
  # endif()
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rclcpp)

ament_package()
