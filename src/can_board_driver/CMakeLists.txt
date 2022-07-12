cmake_minimum_required(VERSION 2.8.3)
project(can_board_driver)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  asr408_msgs
  h9_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs asr408_msgs h9_msgs
  LIBRARIES asr408_driver h9_driver
  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  asr408_driver/include
  h9_driver/include

)
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_library(asr408_driver
  asr408_driver/src/asr408_driver.cpp
)
add_dependencies(asr408_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(asr408_driver
  ${catkin_LIBRARIES}
)

add_library(h9_driver
  h9_driver/src/h9_driver.cpp
)
add_dependencies(h9_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(h9_driver
  ${catkin_LIBRARIES}
)

add_executable (${PROJECT_NAME}_node src/can_board_driver_node.cpp src/can_board_driver.cpp src/socket.cpp)
add_dependencies(${PROJECT_NAME}_node 
  ${catkin_EXPORTED_TARGETS}
  asr408_driver
  h9_driver
)
target_link_libraries (${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  asr408_driver
  h9_driver
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS asr408_driver h9_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/asr408_driver/ include/h9_driver/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
