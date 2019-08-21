cmake_minimum_required(VERSION 2.8.3)
project(crop_hull_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  tf2_ros
  tf2_eigen
  visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES crop_hull_filter
  CATKIN_DEPENDS nodelet pcl_ros pluginlib roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(crop_hull_filter
  src/crop_hull_filter.cpp
)
target_link_libraries(crop_hull_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(crop_hull_filter)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS crop_hull_filter
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)