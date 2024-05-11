cmake_minimum_required(VERSION 3.0.2)
project(pose_corrector)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_filters
)

catkin_python_setup()

#===========================================================
# Generate services in the 'srv' folder
#===========================================================
# add_service_files(
#   FILES
# )
#===========================================================
# Generate added messages and services with 
# any dependencies listed here
#===========================================================
# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# Declare catkin package
#===========================================================
# Catkin package specific configurations
#===========================================================
catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  message_filters 
  # LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/pose_corrector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#===========================================================
# Install
#===========================================================
install(
  DIRECTORY bags rviz launch
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
#===========================================================

include_directories(${catkin_INCLUDE_DIRS} include)
add_executable(pose_corrector src/pose_corrector_node.cpp src/utils.cpp)
target_link_libraries(pose_corrector ${catkin_LIBRARIES})

