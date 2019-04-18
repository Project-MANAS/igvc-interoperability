cmake_minimum_required(VERSION 3.5)
project(interoperability)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        nav_msgs
        robot_localization
        dwa_local_planner
        waypoint_server
)

catkin_package(CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        nav_msgs
        robot_localization
        dwa_local_planner
        waypoint_server
)

set(JAUS_INCLUDE_PATH "openjaus/include")
set(JAUS_LIB_PATH "openjaus/lib")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${JAUS_INCLUDE_PATH}
)

link_directories(
        ${JAUS_LIB_PATH}
)

add_library(${PROJECT_NAME}
        src/NavigationAndReportingComponent.cpp
)

add_executable(iop_node src/iop_node.cpp)
target_link_libraries(iop_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        openjaus
        openjaus-core_v1_1
        openjaus-mobility_v1_0
)
add_dependencies(iop_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

add_executable(cvt_node src/CVTComponent.cpp)
target_link_libraries(cvt_node
        ${catkin_LIBRARIES}
        openjaus
        openjaus-core_v1_1
        openjaus-mobility_v1_0
)
add_dependencies(cvt_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
