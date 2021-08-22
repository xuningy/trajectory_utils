cmake_minimum_required(VERSION 3.1)
project(trajectory_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
COMPONENTS
		roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include  ${EIGEN3_INCLUDE_DIRS}
    DEPENDS
		LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
			roscpp
)

include_directories(include
      ${catkin_INCLUDE_DIRS}
			${EIGEN3_INCLUDE_DIR}
)

link_directories(
	${catkin_LIBRARY_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
	src/trajectory_calculators/PolynomialCalculator.cpp
	src/trajectory_calculators/TimeOptimalPolynomialCalculator.cpp
)

add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
