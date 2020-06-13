cmake_minimum_required(VERSION 3.5)
project(realtime_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} src/realtime_clock.cpp)
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "rclcpp_action")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Unit Tests
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(LANGUAGE C++)
  # Renable the below once we have a global variable
  # supporting the language feature for ament_uncrustify
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gmock(realtime_box_tests test/realtime_box_tests.cpp)
  target_link_libraries(realtime_box_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  ament_add_gmock(realtime_buffer_tests test/realtime_buffer_tests.cpp)
  target_link_libraries(realtime_buffer_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  ament_add_gmock(realtime_clock_tests test/realtime_clock_tests.cpp)
  target_link_libraries(realtime_clock_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  ament_add_gmock(realtime_publisher_tests test/realtime_publisher.test test/realtime_publisher_tests.cpp)
  target_link_libraries(realtime_publisher_tests ${PROJECT_NAME} ${test_msgs_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
  target_include_directories(realtime_publisher_tests PRIVATE ${test_msgs_INCLUDE_DIRS})

  ament_add_gmock(realtime_server_goal_handle_tests test/realtime_server_goal_handle.test test/realtime_server_goal_handle_tests.cpp)
  target_link_libraries(realtime_server_goal_handle_tests ${PROJECT_NAME} ${test_msgs_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
  target_include_directories(realtime_server_goal_handle_tests PRIVATE ${test_msgs_INCLUDE_DIRS})
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
ament_export_dependencies(rclcpp_action)

ament_package()
