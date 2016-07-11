cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  nodelet
  pluginlib
  message_filters
  diagnostic_msgs
  diagnostic_updater
)


include_directories(
  include
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(rplidar_client_demo src/client.cpp)
target_link_libraries(rplidar_client_demo ${catkin_LIBRARIES})

add_library(rplidar_nodelet src/rplidar_nodelet.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidar_nodelet ${catkin_LIBRARIES})

add_executable(rplidar_node src/rplidar_node.cpp)
target_link_libraries(rplidar_node rplidar_nodelet ${catkin_LIBRARIES})

###############################################################################
# Installs
###############################################################################

install(TARGETS rplidar_client_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS rplidar_node        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS rplidar_nodelet
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
