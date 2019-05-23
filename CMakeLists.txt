cmake_minimum_required(VERSION 2.8.3)
project(realtime_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

find_package(Boost COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME} src/realtime_clock.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Unit Tests
catkin_add_gtest(realtime_box_tests test/realtime_box_tests.cpp)
if (TARGET realtime_box_tests)
  target_link_libraries(realtime_box_tests PRIVATE ${PROJECT_NAME})
endif()

catkin_add_gtest(realtime_buffer_tests test/realtime_buffer_tests.cpp)
if (TARGET realtime_buffer_tests)
  target_link_libraries(realtime_buffer_tests PRIVATE ${PROJECT_NAME})
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
