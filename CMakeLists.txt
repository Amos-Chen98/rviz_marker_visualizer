cmake_minimum_required(VERSION 3.0.2)
project(rviz_marker_visualizer)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/visualizer_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
