cmake_minimum_required(VERSION 3.1)
project(trajectory_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
COMPONENTS
		roscpp
		bspline
    control_arch
		geometry_utils
		forward_arc_primitives
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS
    CATKIN_DEPENDS
			roscpp
			bspline
			control_arch
			geometry_utils
			forward_arc_primitives
)

include_directories(include
      ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
