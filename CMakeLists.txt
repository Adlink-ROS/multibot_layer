cmake_minimum_required(VERSION 2.8.3)
project(multibot_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multibot_layers
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(multibot_layer src/multibot_layer.cpp)


install(FILES costmap_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS multibot_layer
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
