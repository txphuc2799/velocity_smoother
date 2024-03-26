cmake_minimum_required(VERSION 3.0.2)
project(velocity_smoother)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pluginlib 
  nodelet 
  ecl_threads 
  dynamic_reconfigure 
  geometry_msgs 
  nav_msgs)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/VelocitySmoother.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Nodelet library
add_library(${PROJECT_NAME} src/velocity_smoother.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugins
install(FILES ${PROJECT_NAME}_nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)