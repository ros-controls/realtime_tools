cmake_minimum_required(VERSION 2.8.3)
project(realtime_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Threads REQUIRED)

include_directories(include)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME} src/realtime_clock.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})

# Unit Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gmock(realtime_box_tests test/realtime_box_tests.cpp)
  target_link_libraries(realtime_box_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  catkin_add_gmock(realtime_buffer_tests test/realtime_buffer_tests.cpp)
  target_link_libraries(realtime_buffer_tests ${PROJECT_NAME} ${GMOCK_MAIN_LIBRARIES})

  catkin_add_gmock(realtime_clock_tests test/realtime_clock_tests.cpp)
  target_link_libraries(realtime_clock_tests ${PROJECT_NAME})

  find_package(rostest REQUIRED)
  add_rostest_gmock(realtime_publisher_tests test/realtime_publisher.test test/realtime_publisher_tests.cpp)
  find_package(std_msgs REQUIRED)
  target_link_libraries(realtime_publisher_tests ${PROJECT_NAME} ${std_msgs_LIBRARIES})
  target_include_directories(realtime_publisher_tests PRIVATE ${std_msgs_INCLUDE_DIRS})

  add_rostest_gmock(realtime_server_goal_handle_tests test/realtime_server_goal_handle.test test/realtime_server_goal_handle_tests.cpp)
  find_package(actionlib REQUIRED)
  target_link_libraries(realtime_server_goal_handle_tests ${PROJECT_NAME} ${actionlib_LIBRARIES})
  target_include_directories(realtime_server_goal_handle_tests PRIVATE ${actionlib_INCLUDE_DIRS})
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
