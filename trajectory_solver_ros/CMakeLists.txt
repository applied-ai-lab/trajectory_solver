cmake_minimum_required(VERSION 3.0.2)
project(trajectory_solver_ros)

find_package(catkin REQUIRED COMPONENTS 
             rospy
             std_msgs
             geometry_msgs
             message_generation             
             trajectory_solver)

catkin_python_setup()

# catkin_install_python(PROGRAMS
#     scripts/osqp_least_squares.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  EndPointPoseMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# install(DIRECTORY resources/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)
FILE(GLOB python_examples "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
